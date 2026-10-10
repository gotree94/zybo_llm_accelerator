# Zybo Z7-20 기반 INT8 트랜스포머 디코더 가속기 구현 가이드

> 환경 설정 → 모델/데이터 준비 → PL(Verilog) → 시뮬레이션 → Vivado 블록 디자인 → Vitis/FreeRTOS 펌웨어 → C# 클라이언트 → 검증/최적화

---

## 0. 먼저 읽을 것 (범위와 설계 결정)

원 보고서는 결과 중심이라 **레지스터 맵, 스트림 프로토콜, 가중치 메모리 배치**가 빠져 있습니다. 이 문서는 그 빈칸을 아래처럼 채워 **처음부터 끝까지 재현 가능한** 구현으로 만들었습니다. 보고서와 다른 부분은 의도적인 선택입니다.

| 항목 | 보고서 | 이 가이드의 선택 | 이유 |
|---|---|---|---|
| 데이터 경로 | S_AXI_ACP | **S_AXI_HP0 + 명시적 Flush/Invalidate** | ACP는 AxUSER/AxCACHE 설정이 까다로워 첫 구현에 부적합. 정상 동작 후 ACP로 바꾸는 것은 §15에 정리 |
| 헤드 처리 | 펌웨어가 8헤드를 순차 전송 | **PL 내부에서 8헤드(행 블록 32개씩)를 순차 계산** | 전송 횟수를 줄이고 정렬 문제를 구조적으로 제거 |
| 스트림 폭 | 512 B 프레임 | **64비트 스트림(1비트 = INT8 8개 = SIMD 8레인)** | 1 beat = 8 MAC, 행 하나 = 32 beat로 보고서의 "32 clock beats"와 일치 |
| 제어 | 미기재 | **AXI GPIO(듀얼 채널) 하나**로 ctrl/status | AXI-Lite 슬레이브 보일러플레이트 제거 |
| 모델 의미 | 어텐션/ReLU | **헤드 분할 선형층 + ReLU + 포화 요양자화** (소프트맥스 어텐션은 §15 확장) | 구현 가능한 최소 구조부터 검증 |

**중요한 한계 (미리 알아둘 것)**
- 가중치는 학습된 값이 아니라 **난수**로 생성합니다. 하드웨어 동작(비트 단위 일치)을 검증하는 용도이며, 의미 있는 문장은 나오지 않습니다. 의미 있는 출력은 PyTorch 등으로 학습·양자화한 가중치를 같은 배치(§4)로 내보내면 됩니다.
- 성능은 보고서의 8.2 ms를 그대로 보장하지 않습니다. 이 설계의 이론값은 **토큰당 약 2 ms**이며(§13), 요청 지연 = 토큰 수 × 토큰당 시간 + 네트워크 왕복입니다.

---

## 1. 시스템 아키텍처

```
┌────────────────────────── Zynq-7020 ───────────────────────────┐
│ PS (Cortex-A9 + FreeRTOS + lwIP)             PL (100 MHz)       │
│                                                                  │
│  TCP :5001 서버 ─ 토크나이저 ─ 샘플러                            │
│        │                                                         │
│  llm_hw.c ──AXI-Lite──► AXI GPIO ─ctrl──► ┌──────────────┐      │
│     │                          ◄─status── │  LlmEngine   │      │
│     │ DDR3 0x1F000000                      │  ping/pong   │      │
│     ▼ (가중치 20×64KB)                     │  8-lane SIMD │      │
│  AXI DMA MM2S ──64b AXIS────────────────► │  ReLU+clamp  │      │
│  AXI DMA S2MM ◄──64b AXIS───────────────  └──────────────┘      │
│     │                                                            │
│  S_AXI_HP0 ◄── DMA 마스터 (DDR 직접 접근)                        │
└──────────────────────────────────────────────────────────────────┘
          │ Ethernet (RGMII, MIO)
          ▼
   PC: C# WinForms 클라이언트 (TCP 192.168.1.10:5001)
```

**연산 정의 (한 레이어)**

```
y[r] = clamp( ReLU( (Σ_c W[r][c]·x[c]) >> SHIFT ), 0, 127 )     r, c = 0..255
```
- 입력 x, 가중치 W는 INT8, 누산기는 32비트, 출력은 0~127 INT8(ReLU이므로 음수 없음).
- 20레이어를 연달아 적용하며 **y가 다음 레이어의 x가 됩니다(PL 내부 피드백, DMA 불필요)**.
- 헤드 h는 출력 행 `32h … 32h+31`을 담당합니다.
- 토큰 생성: 컨텍스트(최대 256 토큰 ID)를 x로 넣고, 20레이어 후 `y[0..127]`을 로짓으로 보고 샘플링합니다.

**레지스터 맵 (AXI GPIO)**

| GPIO | 비트 | 이름 | 설명 |
|---|---|---|---|
| ch1 (출력, ctrl) | 0 | START | 0→1 상승 에지에서 연산 시작 |
| | 1 | OUT | 0→1 상승 에지에서 현재 x 벡터를 M_AXIS로 송출 |
| | 2 | MODE_X | 0: 스트림 수신=가중치 뱅크, 1: 스트림 수신=x 벡터 |
| | 3 | WBANK | 가중치를 쓸 뱅크(0=ping, 1=pong) |
| | 4 | CBANK | 연산에 쓸 뱅크 (START 시점에 래치) |
| | 15:8 | SHIFT | 요양자화 우측 시프트 (START 시점에 래치) |
| ch2 (입력, status) | 0 | BUSY | 연산 중 |
| | 1 | DONE | 연산 완료(스티키, START 에지에서 클리어) |

---

## 2. 준비물

**하드웨어**
- Digilent **Zybo Z7-20** (XC7Z020-1CLG400C), 마이크로 USB 케이블(전원+JTAG+UART), 이더넷 케이블
- (선택) microSD 카드: 부팅/가중치 적재를 JTAG 없이 하고 싶을 때

**소프트웨어 (권장 조합)**
| 도구 | 버전 | 비고 |
|---|---|---|
| AMD/Xilinx Vivado + Vitis (Classic) | **2022.2 또는 2023.1** | 이 문서의 코드는 `XPAR_*_DEVICE_ID` 매크로 방식 기준. 2024.1 이상은 BaseAddress 방식으로 바뀌어 초기화 코드 수정 필요 |
| Digilent board files | 최신 | Zybo Z7-20 프리셋 |
| Python | 3.9+ + numpy | 가중치 생성, 골든 모델 |
| Visual Studio 2022 | .NET Framework 4.8 워크로드 | Chart 컨트롤이 기본 포함되어 NuGet 불필요 |
| 터미널 | Tera Term / PuTTY | UART 115200-8-N-1 |

---

## 3. 환경 설정

### 3.1 Vivado/Vitis 설치
1. Vitis 통합 설치 패키지에서 **Vitis**(Vivado 포함)와 **Zynq-7000** 디바이스를 선택해 설치합니다.
2. 설치 마지막의 **케이블 드라이버 설치**를 실행합니다 (Windows는 자동, Linux는 `<설치경로>/data/xicom/cable_drivers/lin64/install_script/install_drivers/install_drivers` 를 sudo로 실행).

### 3.2 Zybo Z7-20 보드 파일
```bash
git clone https://github.com/Digilent/vivado-boards.git
# 아래 폴더의 내용을 Vivado 설치 경로로 복사
#   vivado-boards/new/board_files/*  →  <Vivado설치경로>/data/boards/board_files/
```
Vivado를 재시작한 뒤 프로젝트 생성 시 보드 목록에 `Zybo Z7-20`이 보여야 합니다.

### 3.3 Zybo 점퍼/연결
- 전원 점퍼(JP6): USB 전원, 부팅 모드 점퍼(JP5): **JTAG**
- 마이크로 USB(PROG/UART) ↔ PC, 이더넷 ↔ PC(직결) 또는 공유기
- PC 이더넷 어댑터를 **고정 IP `192.168.1.100 / 255.255.255.0`** 으로 설정 (보드는 `192.168.1.10`)

### 3.4 작업 폴더 구조
```
zybo_llm/
├─ tools/            llm_tools.py
├─ rtl/              LlmEngine.v, tb_llm_engine.v
├─ vivado/           (Vivado 프로젝트)
├─ vitis/            (Vitis 워크스페이스)
│   └─ llm_app/src/  main.c, llm_hw.c/.h, tokenizer.c/.h, sampler.c/.h, infer_server.c/.h
├─ client/           LlmClient (Visual Studio 솔루션)
└─ out/              weights.bin, layer0.hex, x.hex, y_gold.hex
```

---

## 4. 모델/데이터 준비 (Python)

### 4.1 가중치 배치 규칙 (PL·펌웨어와 반드시 일치)
- `weights.bin` = `[layer(20)][row(256)][col(256)]` **row-major, INT8**, 총 **1,310,720 B**
- 각 레이어 = 65,536 B (= 8,192 beats × 8 B). 한 beat는 **리틀엔디안**: 메모리의 가장 낮은 주소 바이트가 `TDATA[7:0]` (= 레인 0)
- 행 r, 열 묶음 c(0..31)의 beat 번호 = `r*32 + c` = BRAM 주소 `{r[7:0], c[4:0]}`

### 4.2 `tools/llm_tools.py`
```python
#!/usr/bin/env python3
"""가중치 생성 + 비트 정확 골든 모델 + 테스트벤치 벡터 생성"""
import sys
import numpy as np

D, L = 256, 20
SHIFT, SIGMA, SEED = 7, 11.0, 2026   # SIGMA는 2^SHIFT * 0.088 근처 (활성 크기 유지용)


def make_weights():
    rng = np.random.default_rng(SEED)
    w = np.round(rng.normal(0, SIGMA, (L, D, D)))
    return np.clip(w, -127, 127).astype(np.int8)


def layer(x, w, shift=SHIFT):
    acc = w.astype(np.int32) @ x.astype(np.int32)          # 32비트 누산
    return np.clip(acc >> shift, 0, 127).astype(np.int8)   # >> 는 산술 시프트, 음수는 0으로


def forward(x, W, shift=SHIFT, verbose=False):
    for l in range(L):
        x = layer(x, W[l], shift)
        if verbose:
            print(f"layer {l:2d}: nonzero={np.count_nonzero(x):3d}/256 "
                  f"max={int(x.max()):3d} mean={x.mean():6.2f}")
    return x


def hex64(a):
    """int8 배열 → 리틀엔디안 64비트 워드 hex 문자열 목록 (바이트0이 최하위)"""
    a = np.ascontiguousarray(a).view(np.uint8).reshape(-1, 8)
    return ["".join(f"{b:02x}" for b in row[::-1]) for row in a]


def selftest_x():
    return (np.arange(D) % 128).astype(np.int8)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "all"
    W = make_weights()
    if cmd in ("gen", "all"):
        W.tofile("weights.bin")
        x = selftest_x()
        open("layer0.hex", "w").write("\n".join(hex64(W[0])) + "\n")
        open("x.hex", "w").write("\n".join(hex64(x)) + "\n")
        open("y_gold.hex", "w").write("\n".join(hex64(layer(x, W[0]))) + "\n")
        print("weights.bin:", W.nbytes, "bytes")           # 1310720 이어야 함
    if cmd in ("golden", "all"):
        y = forward(selftest_x(), W, verbose=True)
        print("SELFTEST first 32 bytes:", y[:32].tobytes().hex())
```

### 4.3 실행
```bash
cd out
python ../tools/llm_tools.py all
```
- `weights.bin`이 **1310720 bytes**인지 확인합니다.
- 레이어별 `nonzero`가 0에 수렴하거나 `max`가 127로 포화되면 SHIFT/SIGMA가 맞지 않는 것입니다. 20레이어 후에도 `nonzero`가 수십~백 단위로 유지되도록 조정합니다. SHIFT를 바꾸면 **펌웨어의 `HW_SHIFT`도 같이** 바꿉니다.
- 마지막 줄 `SELFTEST first 32 bytes: ...` 는 §12에서 보드 결과와 비교합니다.

---

## 5. PL 설계: `rtl/LlmEngine.v`

### 5.1 설계 요약
- **두 개의 BRAM 뱅크**(각 8192×64b ≈ 64 KB, 합계 약 30 BRAM36). 한쪽을 읽어 연산하는 동안 다른 쪽에 다음 레이어를 스트림으로 적재합니다(ping-pong).
- 8개의 8×8 곱셈기(DSP48E1) → 합산 → 32비트 누산 → 시프트 → ReLU/포화 → 8바이트씩 묶어 x 메모리의 **반대쪽 절반**에 기록. 레이어가 끝나면 `xsel`을 토글하여 y가 곧바로 다음 x가 됩니다(복사 없음).
- 파이프라인: 주소 발행 → BRAM 읽기 → 레지스터 → 곱셈 → 8항 합 → 누산. 행당 32클럭, 레이어당 8192클럭(+약 6).

### 5.2 코드
```verilog
`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// LlmEngine : 256x256 INT8 matrix-vector engine, 8-lane SIMD, ping-pong banks
// ---------------------------------------------------------------------------
module bram_sdp #(parameter AW = 13, parameter DW = 64) (
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);
    (* ram_style = "block" *) reg [DW-1:0] mem [0:(1<<AW)-1];
    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end
endmodule

module LlmEngine (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m_axis, ASSOCIATED_RESET rstn, FREQ_HZ 100000000" *)
    input  wire        clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rstn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        rstn,

    input  wire [31:0] ctrl,
    output wire [31:0] status,

    input  wire [63:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,

    output reg  [63:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast
);
    // ------------------------------------------------------------ ctrl decode
    wire       c_start = ctrl[0];
    wire       c_out   = ctrl[1];
    wire       c_modex = ctrl[2];
    wire       c_wbank = ctrl[3];
    wire       c_cbank = ctrl[4];
    wire [7:0] c_shift = ctrl[15:8];

    reg start_q, out_q;
    always @(posedge clk) begin
        if (!rstn) begin start_q <= 1'b0; out_q <= 1'b0; end
        else       begin start_q <= c_start; out_q <= c_out; end
    end
    wire start_pulse = c_start & ~start_q;
    wire out_pulse   = c_out   & ~out_q;

    // ------------------------------------------------- stream write (weights/x)
    assign s_axis_tready = 1'b1;               // BRAM/LUTRAM 쓰기는 항상 수용
    reg [12:0] wptr;                           // TLAST 에서 0으로 복귀
    always @(posedge clk) begin
        if (!rstn) wptr <= 13'd0;
        else if (s_axis_tvalid) wptr <= s_axis_tlast ? 13'd0 : wptr + 13'd1;
    end
    wire we_a = s_axis_tvalid & ~c_modex & ~c_wbank;
    wire we_b = s_axis_tvalid & ~c_modex &  c_wbank;

    // ------------------------------------------------------------ weight banks
    reg  [7:0]  row;
    reg  [4:0]  col;
    wire [12:0] raddr = {row, col};
    wire [63:0] rd_a, rd_b;

    bram_sdp bank_ping (.clk(clk), .we(we_a), .waddr(wptr), .wdata(s_axis_tdata),
                        .raddr(raddr), .rdata(rd_a));
    bram_sdp bank_pong (.clk(clk), .we(we_b), .waddr(wptr), .wdata(s_axis_tdata),
                        .raddr(raddr), .rdata(rd_b));

    // ----------------------------------------------- activation memory (x / y)
    // 64 x 64b : xsel 이 읽기 절반, ~xsel 이 쓰기 절반 (레이어 종료 시 토글)
    reg [63:0] xmem [0:63];
    reg        xsel;
    reg        y_we;
    reg [5:0]  y_waddr;
    reg [63:0] y_word;
    always @(posedge clk) begin
        if (y_we)                          xmem[y_waddr]            <= y_word;
        else if (s_axis_tvalid & c_modex)  xmem[{xsel, wptr[4:0]}]  <= s_axis_tdata;
    end

    // ------------------------------------------------------- control sequencer
    reg       busy, done, run, cbank_q;
    reg [7:0] shift_q;
    reg       y_last;

    always @(posedge clk) begin
        if (!rstn) begin
            busy <= 1'b0; done <= 1'b0; run <= 1'b0;
            row <= 8'd0; col <= 5'd0; cbank_q <= 1'b0; shift_q <= 8'd0; xsel <= 1'b0;
        end else begin
            if (start_pulse && !busy) begin
                busy <= 1'b1; done <= 1'b0; run <= 1'b1;
                row <= 8'd0; col <= 5'd0;
                cbank_q <= c_cbank; shift_q <= c_shift;
            end else if (run) begin
                if (col == 5'd31) begin
                    col <= 5'd0;
                    if (row == 8'd255) run <= 1'b0;
                    else               row <= row + 8'd1;
                end else begin
                    col <= col + 5'd1;
                end
            end
            if (y_we && y_last) begin          // 마지막 y 워드가 기록되는 사이클
                busy <= 1'b0; done <= 1'b1; xsel <= ~xsel;
            end
        end
    end

    // --------------------------------------------------------- MAC pipeline
    reg        v1, l1;  reg [4:0] col1;
    reg        vA, lA;  reg [63:0] wdA, xwA;
    reg        vB, lB;  (* use_dsp = "yes" *) reg signed [15:0] prod [0:7];
    reg        vC, lC;  reg signed [18:0] sumC;
    integer i;

    always @(posedge clk) begin
        if (!rstn) begin
            v1 <= 0; l1 <= 0; col1 <= 0; vA <= 0; lA <= 0; wdA <= 0; xwA <= 0;
            vB <= 0; lB <= 0; vC <= 0; lC <= 0; sumC <= 0;
        end else begin
            // stage 0 -> 1 : BRAM 데이터는 다음 사이클에 유효
            v1   <= run;
            l1   <= (col == 5'd31);
            col1 <= col;
            // stage 1 -> A : 가중치 뱅크 선택 + x 워드 읽기
            vA   <= v1;
            lA   <= l1;
            wdA  <= cbank_q ? rd_b : rd_a;
            xwA  <= xmem[{xsel, col1}];
            // stage A -> B : 8레인 곱셈
            vB   <= vA;
            lB   <= lA;
            for (i = 0; i < 8; i = i + 1)
                prod[i] <= $signed(wdA[8*i +: 8]) * $signed(xwA[8*i +: 8]);
            // stage B -> C : 8항 합
            vC   <= vB;
            lC   <= lB;
            sumC <= prod[0] + prod[1] + prod[2] + prod[3]
                  + prod[4] + prod[5] + prod[6] + prod[7];
        end
    end

    // ------------------------------- accumulate / requantize / ReLU / pack
    reg signed [31:0] acc;
    reg [63:0]        yshift;
    reg [7:0]         orow;

    wire signed [31:0] fin = acc + sumC;
    wire signed [31:0] shf = fin >>> shift_q;
    wire        [7:0]  ybyte = (fin <= 0)  ? 8'd0   :     // ReLU
                               (shf > 127) ? 8'd127 :     // 포화 클램프
                                             shf[7:0];

    always @(posedge clk) begin
        if (!rstn || (start_pulse && !busy)) begin
            acc <= 0; orow <= 0; yshift <= 0;
            y_we <= 0; y_last <= 0; y_waddr <= 0; y_word <= 0;
        end else begin
            y_we   <= 1'b0;
            y_last <= 1'b0;
            if (vC) begin
                if (lC) begin
                    acc    <= 0;
                    yshift <= {ybyte, yshift[63:8]};       // 먼저 온 바이트가 최하위
                    orow   <= orow + 8'd1;
                    if (orow[2:0] == 3'd7) begin
                        y_we    <= 1'b1;
                        y_word  <= {ybyte, yshift[63:8]};
                        y_waddr <= {~xsel, orow[7:3]};
                        y_last  <= (orow == 8'd255);
                    end
                end else begin
                    acc <= acc + sumC;
                end
            end
        end
    end

    // ------------------------------------------------- output stream (x -> DMA)
    reg       out_active;
    reg [4:0] oidx;
    always @(posedge clk) begin
        if (!rstn) begin
            out_active <= 0; oidx <= 0;
            m_axis_tdata <= 0; m_axis_tvalid <= 0; m_axis_tlast <= 0;
        end else begin
            if (out_pulse && !out_active) begin
                out_active <= 1'b1; oidx <= 5'd0;
            end
            if (out_active) begin
                if (!m_axis_tvalid || m_axis_tready) begin
                    m_axis_tdata  <= xmem[{xsel, oidx}];
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= (oidx == 5'd31);
                    oidx          <= oidx + 5'd1;
                    if (oidx == 5'd31) out_active <= 1'b0;
                end
            end else if (m_axis_tvalid && m_axis_tready) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end
        end
    end

    assign status = {30'd0, done, busy};
endmodule
```

**설계 포인트 / 주의**
- **연산 중에는 `cbank`로 읽는 뱅크에 쓰지 마십시오.** 펌웨어가 `WBANK != CBANK`를 보장합니다(§9).
- 가중치/x 적재 중에는 `MODE_X`, `WBANK`를 바꾸지 마십시오(DMA 전송이 끝난 뒤 변경).
- `TLAST`가 쓰기 포인터를 0으로 되돌리므로, 전송 길이는 **반드시 정확히** 65,536 B(가중치) / 256 B(x)여야 합니다.
- AXI DMA의 `TKEEP`는 연결하지 않아도 됩니다(항상 전 바이트 유효).

---

## 6. RTL 시뮬레이션

### 6.1 `rtl/tb_llm_engine.v` (레이어 1개를 골든과 비교)
```verilog
`timescale 1ns / 1ps
module tb_llm_engine;
    reg clk = 0, rstn = 0;
    always #5 clk = ~clk;

    reg  [31:0] ctrl = 0;
    wire [31:0] status;
    reg  [63:0] s_tdata = 0; reg s_tvalid = 0, s_tlast = 0; wire s_tready;
    wire [63:0] m_tdata;     wire m_tvalid, m_tlast;        reg  m_tready = 1;

    LlmEngine dut (
        .clk(clk), .rstn(rstn), .ctrl(ctrl), .status(status),
        .s_axis_tdata(s_tdata), .s_axis_tvalid(s_tvalid),
        .s_axis_tready(s_tready), .s_axis_tlast(s_tlast),
        .m_axis_tdata(m_tdata), .m_axis_tvalid(m_tvalid),
        .m_axis_tready(m_tready), .m_axis_tlast(m_tlast));

    reg [63:0] wmem [0:8191];
    reg [63:0] xin  [0:31];
    reg [63:0] gold [0:31];
    integer i, k, errors;

    // 출력 비교
    always @(posedge clk) if (m_tvalid && m_tready) begin
        if (m_tdata !== gold[k]) begin
            errors = errors + 1;
            $display("MISMATCH beat %0d: got %h exp %h", k, m_tdata, gold[k]);
        end
        k = k + 1;
    end

    initial begin
        errors = 0; k = 0;
        $readmemh("layer0.hex", wmem);
        $readmemh("x.hex",      xin);
        $readmemh("y_gold.hex", gold);
        repeat (5) @(posedge clk);
        rstn <= 1;
        repeat (5) @(posedge clk);

        // 1) x 적재 : MODE_X=1, SHIFT=7  -> ctrl = 0x0704
        ctrl <= 32'h0000_0704;
        for (i = 0; i < 32; i = i + 1) begin
            @(posedge clk); s_tdata <= xin[i]; s_tvalid <= 1; s_tlast <= (i == 31);
        end
        @(posedge clk); s_tvalid <= 0; s_tlast <= 0;

        // 2) 가중치 적재 : MODE_X=0, WBANK=0 -> ctrl = 0x0700
        @(posedge clk); ctrl <= 32'h0000_0700;
        for (i = 0; i < 8192; i = i + 1) begin
            @(posedge clk); s_tdata <= wmem[i]; s_tvalid <= 1; s_tlast <= (i == 8191);
        end
        @(posedge clk); s_tvalid <= 0; s_tlast <= 0;

        // 3) 연산 시작 (CBANK=0)
        @(posedge clk); ctrl <= 32'h0000_0701;
        @(posedge clk); ctrl <= 32'h0000_0700;
        wait (status[1]);                    // DONE
        $display("compute done at %0t", $time);

        // 4) 출력
        @(posedge clk); ctrl <= 32'h0000_0702;
        @(posedge clk); ctrl <= 32'h0000_0700;
        repeat (100) @(posedge clk);

        if (errors == 0 && k == 32) $display("PASS: 32 beats matched");
        else                        $display("FAIL: errors=%0d beats=%0d", errors, k);
        $finish;
    end
endmodule
```

### 6.2 실행 방법
1. Vivado에서 새 RTL 프로젝트(파트 `xc7z020clg400-1`) 생성 → `LlmEngine.v`를 Design Source, `tb_llm_engine.v`를 Simulation Source로 추가.
2. `out/`의 `layer0.hex`, `x.hex`, `y_gold.hex` 3개를 시뮬레이션 실행 폴더(`<proj>.sim/sim_1/behav/xsim/`)로 복사.
3. **Run Simulation → Run Behavioral Simulation**, 약 1 ms 시뮬레이션 후 콘솔에서 `PASS: 32 beats matched`를 확인합니다.
4. 불일치 시 점검 순서: hex 바이트 순서(§4.1) → SHIFT 값(0x0700 = 7) → `y_gold.hex`와 `layer0.hex` 같은 시드인지.

---

## 7. Vivado 블록 디자인

### 7.1 프로젝트 생성
- Create Project → RTL Project → **Boards 탭에서 Zybo Z7-20 선택**
- `LlmEngine.v` 추가(Add Sources)

### 7.2 블록 디자인 구성 (Create Block Design: `design_1`)
1. **ZYNQ7 Processing System** 추가 → *Run Block Automation (Apply Board Preset 체크)*.
2. PS 더블클릭 → 다음을 확인/설정합니다.
   - **PS-PL Configuration → HP Slave AXI Interface → `S AXI HP0 interface` 체크** (Data Width 64)
   - **GP Master AXI Interface → `M AXI GP0`** 체크
   - **Clock Configuration → PL Fabric Clocks → FCLK_CLK0 = 100 MHz**
   - **Peripheral I/O Pins**: ENET 0(RGMII), UART 1, (SD 0) 활성 → 보드 프리셋이 처리
3. **AXI Direct Memory Access** 추가 (`axi_dma_0`)
   - Enable Scatter Gather Engine: **해제** (Simple 모드)
   - Width of Buffer Length Register: **23**
   - Enable Read Channel: 체크, **Memory Map Data Width 64, Stream Data Width 64, Max Burst Size 16**
   - Enable Write Channel: 체크, 동일하게 64 / 64 / 16
   - Allow Unaligned Transfers: 해제
4. **AXI GPIO** 추가 (`axi_gpio_0`)
   - **Enable Dual Channel** 체크
   - Channel 1: GPIO Width **32**, **All Outputs**
   - Channel 2: GPIO Width **32**, **All Inputs**
5. **Add Module** → `LlmEngine` 추가(`LlmEngine_0`). 포트 이름 규칙 덕분에 `s_axis`, `m_axis`가 AXI-Stream 인터페이스로 인식되어야 합니다.
6. **연결**
   | 출발 | 도착 |
   |---|---|
   | `axi_dma_0/M_AXIS_MM2S` | `LlmEngine_0/s_axis` |
   | `LlmEngine_0/m_axis` | `axi_dma_0/S_AXIS_S2MM` |
   | `axi_gpio_0/gpio_io_o[31:0]` | `LlmEngine_0/ctrl[31:0]` |
   | `LlmEngine_0/status[31:0]` | `axi_gpio_0/gpio2_io_i[31:0]` |
   | `axi_dma_0/M_AXI_MM2S`, `M_AXI_S2MM` | (SmartConnect) → `PS7/S_AXI_HP0` |
   | `PS7/M_AXI_GP0` | (AXI Interconnect) → `axi_dma_0/S_AXI_LITE`, `axi_gpio_0/S_AXI` |
   | `FCLK_CLK0` | 모든 클럭 (`LlmEngine_0/clk` 포함) |
   | `proc_sys_reset/peripheral_aresetn` | `LlmEngine_0/rstn` 및 AXI 리셋들 |
   
   *Run Connection Automation*으로 AXI 계열을 자동 연결한 뒤, `ctrl`/`status`/스트림 연결만 수동으로 확인합니다. DMA 인터럽트(`mm2s_introut`, `s2mm_introut`)는 **폴링 방식이라 연결하지 않습니다.**
7. **Address Editor**에서 모든 슬레이브 주소가 할당되었는지 확인 → **Validate Design (F6)** 에서 에러 0.

### 7.3 빌드
1. Sources에서 `design_1` 우클릭 → **Create HDL Wrapper (Let Vivado manage)**
2. **Generate Bitstream** (합성+구현). 100 MHz에서 타이밍 위반은 거의 없어야 하며, WNS ≥ 0 확인.
3. 예상 사용량: DSP 8 내외(곱셈기), BRAM36 약 30/140, LUT는 수 % 수준.
4. **File → Export → Export Hardware… → Include bitstream** → `design_1_wrapper.xsa` 생성.

---

## 8. Vitis 플랫폼 & BSP

1. Vitis 실행 → 워크스페이스 `vitis/` 지정.
2. **File → New → Platform Project** → `.xsa` 선택 → OS: **freertos10_xilinx**, Processor: `ps7_cortexa9_0`.
3. 플랫폼의 **Board Support Package → Modify BSP Settings**
   - **Overview**: `lwip213`(보이지 않으면 `lwip212`/`lwip211`) 체크
   - **freertos10_xilinx**: `total_heap_size` = **1048576** (1 MB), 필요 시 `check_for_stack_overflow` = 2
   - **lwip2xx**: `api_mode` = **SOCKET_API**, `dhcp` = false (정적 IP)
4. 플랫폼 빌드(Build).
5. **File → New → Application Project** → 위 플랫폼 선택 → 템플릿 **`FreeRTOS lwIP Echo Server`**.
   - 템플릿의 `main.c`/`platform_*.c`(네트워크 기동, 정적 IP `192.168.1.10`)는 **유지**합니다.
   - `echo.c`는 삭제하고, 아래 §9의 파일을 `src/`에 추가합니다.
6. **링커 스크립트 수정** (`lscript.ld`): 가중치 영역(0x1F000000~)을 앱이 침범하지 않도록 DDR 영역을 줄입니다.
   ```
   ps7_ddr_0_MEMORY : ORIGIN = 0x00100000, LENGTH = 0x1EF00000
   ```
7. **컴파일러 설정**: 샘플러의 `expf` 사용을 위해 `-lm`이 링크되는지 확인합니다(기본 포함). 최적화는 `-O2`.

---

## 9. 펌웨어 코드

> **원칙 (보고서의 교훈)**: 연산 루프 안에서 `xil_printf` 금지(115200 baud UART는 한 줄에 수 ms). 디버그 출력은 `#define LLM_DEBUG 0`으로 막아 두고 필요할 때만 켭니다. 고정 지연(`usleep`) 대신 **상태 레지스터 폴링**을 씁니다.

### 9.1 `llm_hw.h`
```c
#ifndef LLM_HW_H
#define LLM_HW_H

#include <stdint.h>

#define LLM_D            256
#define LLM_LAYERS       20
#define LLM_LAYER_BYTES  (LLM_D * LLM_D)                 /* 65536 */
#define LLM_WEIGHT_BASE  0x1F000000U                     /* DDR3 상위 영역 */
#define LLM_VOCAB        128
#define HW_SHIFT         7                               /* llm_tools.py 의 SHIFT 와 일치 */

int llm_hw_init(void);
/* x_in[256] -> 20레이어 -> y_out[256]. 성공 0, 실패 -1 */
int llm_hw_run(const int8_t *x_in, int8_t *y_out, uint8_t shift);

#endif
```

### 9.2 `llm_hw.c`
```c
#include <string.h>
#include "xil_cache.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xaxidma.h"
#include "xgpio.h"
#include "llm_hw.h"

#define LLM_DEBUG 0

/* ctrl 비트 (§1 레지스터 맵) */
#define CTRL_START   (1u << 0)
#define CTRL_OUT     (1u << 1)
#define CTRL_MODE_X  (1u << 2)
#define CTRL_WBANK   (1u << 3)
#define CTRL_CBANK   (1u << 4)
#define CTRL_SHIFT(s) (((uint32_t)(s) & 0xFFu) << 8)
#define ST_BUSY      (1u << 0)
#define ST_DONE      (1u << 1)

#define POLL_LIMIT   20000000u

static XAxiDma dma;
static XGpio   gpio;

/* DMA 버퍼: 32바이트 정렬 (캐시라인 단위로 Flush/Invalidate) */
static int8_t io_buf[LLM_D] __attribute__((aligned(32)));

static inline void set_ctrl(uint32_t v) { XGpio_DiscreteWrite(&gpio, 1, v); }
static inline uint32_t get_status(void) { return XGpio_DiscreteRead(&gpio, 2); }

int llm_hw_init(void)
{
    XAxiDma_Config *cfg;

    if (XGpio_Initialize(&gpio, XPAR_AXI_GPIO_0_DEVICE_ID) != XST_SUCCESS) return -1;
    XGpio_SetDataDirection(&gpio, 1, 0x00000000u);   /* ch1 = 출력 */
    XGpio_SetDataDirection(&gpio, 2, 0xFFFFFFFFu);   /* ch2 = 입력 */
    set_ctrl(0);

    cfg = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
    if (!cfg) return -1;
    if (XAxiDma_CfgInitialize(&dma, cfg) != XST_SUCCESS) return -1;
    if (XAxiDma_HasSg(&dma)) return -1;              /* Simple 모드여야 함 */

    XAxiDma_IntrDisable(&dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    XAxiDma_IntrDisable(&dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
    return 0;
}

/* 채널 상태 레지스터 직접 폴링: 에러 -> -1, 유휴(완료) -> 0 */
static int dma_wait(int direction)
{
    uint32_t off = (direction == XAXIDMA_DMA_TO_DEVICE) ? XAXIDMA_TX_OFFSET
                                                         : XAXIDMA_RX_OFFSET;
    uint32_t guard = POLL_LIMIT;
    while (guard--) {
        uint32_t sr = XAxiDma_ReadReg(dma.RegBase, off + XAXIDMA_SR_OFFSET);
        if (sr & XAXIDMA_ERR_ALL_MASK) {
#if LLM_DEBUG
            xil_printf("DMA error SR=0x%08x dir=%d\r\n", sr, direction);
#endif
            return -1;
        }
        if (sr & XAXIDMA_IDLE_MASK) return 0;
    }
    return -1;
}

static int dma_to_dev(const void *p, uint32_t len)
{
    if (XAxiDma_SimpleTransfer(&dma, (UINTPTR)p, len, XAXIDMA_DMA_TO_DEVICE) != XST_SUCCESS)
        return -1;
    return 0;
}

static int wait_done(void)
{
    uint32_t guard = POLL_LIMIT;
    while (guard--) {
        if (get_status() & ST_DONE) return 0;
    }
    return -1;
}

static inline const int8_t *layer_ptr(int l)
{
    return (const int8_t *)(LLM_WEIGHT_BASE + (uint32_t)l * LLM_LAYER_BYTES);
}

int llm_hw_run(const int8_t *x_in, int8_t *y_out, uint8_t shift)
{
    const uint32_t base = CTRL_SHIFT(shift);
    int l;

    /* 1) x 벡터 적재 (MODE_X=1) */
    memcpy(io_buf, x_in, LLM_D);
    Xil_DCacheFlushRange((UINTPTR)io_buf, LLM_D);
    set_ctrl(base | CTRL_MODE_X);
    if (dma_to_dev(io_buf, LLM_D) || dma_wait(XAXIDMA_DMA_TO_DEVICE)) return -1;

    /* 2) 레이어 0 가중치 -> ping 뱅크 (MODE_X=0, WBANK=0) */
    set_ctrl(base);
    if (dma_to_dev(layer_ptr(0), LLM_LAYER_BYTES) || dma_wait(XAXIDMA_DMA_TO_DEVICE)) return -1;

    /* 3) 20레이어: 연산(cbank) 과 다음 레이어 적재(반대 뱅크) 를 겹침 */
    for (l = 0; l < LLM_LAYERS; l++) {
        uint32_t cb  = (l & 1) ? CTRL_CBANK : 0;
        uint32_t cur = base | cb;

        set_ctrl(cur | CTRL_START);          /* 0->1 : 연산 시작 */
        set_ctrl(cur);                       /* START 내림 */

        if (l + 1 < LLM_LAYERS) {
            uint32_t wb = ((l + 1) & 1) ? CTRL_WBANK : 0;   /* 항상 cbank 의 반대 */
            set_ctrl(cur | wb);              /* MODE_X=0, WBANK=반대 뱅크 */
            if (dma_to_dev(layer_ptr(l + 1), LLM_LAYER_BYTES) ||
                dma_wait(XAXIDMA_DMA_TO_DEVICE)) return -1;
        }
        if (wait_done()) return -1;
    }

    /* 4) 결과 수신: S2MM 을 먼저 무장(arm) 한 뒤 OUT 에지 발생 */
    set_ctrl(base);
    Xil_DCacheFlushRange((UINTPTR)io_buf, LLM_D);       /* 더티 라인이 나중에 덮어쓰지 않도록 */
    if (XAxiDma_SimpleTransfer(&dma, (UINTPTR)io_buf, LLM_D, XAXIDMA_DEVICE_TO_DMA)
        != XST_SUCCESS) return -1;
    set_ctrl(base | CTRL_OUT);
    set_ctrl(base);
    if (dma_wait(XAXIDMA_DEVICE_TO_DMA)) return -1;
    Xil_DCacheInvalidateRange((UINTPTR)io_buf, LLM_D);

    memcpy(y_out, io_buf, LLM_D);
    return 0;
}
```
> 매크로 이름(`XPAR_AXI_DMA_0_DEVICE_ID`, `XPAR_AXI_GPIO_0_DEVICE_ID`)은 플랫폼의 `xparameters.h`에서 실제 이름을 확인해 맞추십시오.

### 9.3 `tokenizer.h / tokenizer.c`
```c
/* tokenizer.h */
#ifndef TOKENIZER_H
#define TOKENIZER_H
#include <stdint.h>

#define TOK_PAD   0
#define TOK_STOP  119
#define TOK_UNK   127

/* 텍스트 -> ids[cap] (남는 자리는 [PAD]=0). 활성 토큰 수 반환. cap 초과 시 마지막 cap 개 유지 */
int         tok_encode(const char *text, int8_t *ids, int cap);
const char *tok_word(int id);

#endif
```
```c
/* tokenizer.c */
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "tokenizer.h"

/* ID 1~19, 48, 119 는 보고서 로그와 동일. 나머지는 "<id>" 로 출력 */
static const char *vocab[128] = {
    [0]  = "[PAD]", [1]  = "the",   [2]  = "cat",    [3]  = "jumped",
    [4]  = "over",  [5]  = "mat",   [6]  = "dog",    [7]  = "sat",
    [8]  = "on",    [9]  = "mouse", [10] = "ran",    [11] = "a",
    [12] = "and",   [13] = "bird",  [14] = "flew",   [15] = "tree",
    [16] = "under", [17] = "boy",   [18] = "girl",   [19] = "walked",
    [48] = "away",  [119] = "end.", [127] = "[UNK]",
};

static int tok_lookup(const char *w)
{
    int i;
    for (i = 1; i < 128; i++)
        if (vocab[i] && strcmp(vocab[i], w) == 0) return i;
    return TOK_UNK;
}

const char *tok_word(int id)
{
    static char tmp[12];
    if (id >= 0 && id < 128 && vocab[id]) return vocab[id];
    snprintf(tmp, sizeof tmp, "<%d>", id);
    return tmp;
}

int tok_encode(const char *text, int8_t *ids, int cap)
{
    int  n = 0, wl = 0;
    char word[32];
    const char *p;

    memset(ids, 0, (size_t)cap);                       /* 전체를 [PAD] 로 초기화 */
    for (p = text;; p++) {
        char c = *p;
        if (c && !isspace((unsigned char)c)) {
            if (wl < 31) word[wl++] = (char)tolower((unsigned char)c);  /* 대소문자 무시 */
        } else {
            if (wl) {
                int id;
                word[wl] = 0; wl = 0;
                id = tok_lookup(word);
                if (n < cap) ids[n++] = (int8_t)id;
                else { memmove(ids, ids + 1, (size_t)cap - 1); ids[cap - 1] = (int8_t)id; }
            }
            if (!c) break;
        }
    }
    return n;
}
```

### 9.4 `sampler.h / sampler.c` (Temperature 0~100 %)
```c
/* sampler.h */
#ifndef SAMPLER_H
#define SAMPLER_H
#include <stdint.h>
void sampler_seed(void);
int  sample_token(const int8_t *logits, int n, int temp_pct);
#endif
```
```c
/* sampler.c */
#include <math.h>
#include "xtime_l.h"
#include "sampler.h"

#define TOPK 8

static uint32_t rng = 0x2545F491u;

static uint32_t xorshift32(void)
{
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return rng;
}

/* Cortex-A9 글로벌 타이머(0xF8F00200 계열) 를 노이즈 시드로 사용 */
void sampler_seed(void)
{
    XTime t;
    XTime_GetTime(&t);
    rng ^= (uint32_t)t ^ (uint32_t)(t >> 32);
    if (rng == 0) rng = 1;
}

int sample_token(const int8_t *logits, int n, int temp_pct)
{
    int   idx[TOPK];
    int   val[TOPK];
    int   i, j, k = 0;
    float w[TOPK], sum = 0.0f, tau, r;

    /* [PAD](0) 제외 상위 K 선택 (동점이면 낮은 인덱스 우선) */
    for (j = 0; j < TOPK; j++) { idx[j] = -1; val[j] = -1000; }
    for (i = 1; i < n; i++) {
        int v = logits[i];
        if (v > val[TOPK - 1]) {
            j = TOPK - 1;
            while (j > 0 && v > val[j - 1]) { val[j] = val[j - 1]; idx[j] = idx[j - 1]; j--; }
            val[j] = v; idx[j] = i;
        }
    }
    if (temp_pct <= 0) return idx[0];                  /* 결정론적 (argmax) */

    tau = (float)temp_pct * 0.16f;                     /* 100% -> 로짓 단위 16 */
    for (k = 0; k < TOPK; k++) {
        w[k] = expf((float)(val[k] - val[0]) / tau);
        sum += w[k];
    }
    r = ((float)(xorshift32() >> 8) / 16777216.0f) * sum;
    for (k = 0; k < TOPK; k++) {
        if (r < w[k]) return idx[k];
        r -= w[k];
    }
    return idx[0];
}
```

### 9.5 `infer_server.h / infer_server.c` (TCP 서버)

프로토콜 (텍스트, 한 줄 = 한 메시지, ASCII):
- 요청: `TEMP:45|the cat mouse dog mat over\n`  /  `SELFTEST\n`
- 응답: `RES|<hw_us>|<토큰수>|<생성 문장>\n`  /  `SELFTEST|<hw_us>|<y[0..31] hex>\n`

```c
/* infer_server.h */
#ifndef INFER_SERVER_H
#define INFER_SERVER_H
void inference_server_thread(void *arg);
#endif
```
```c
/* infer_server.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "xtime_l.h"
#include "xil_printf.h"
#include "llm_hw.h"
#include "tokenizer.h"
#include "sampler.h"
#include "infer_server.h"

#define SERVER_PORT     5001
#define LINE_MAX_LEN    768
#define MAX_NEW_TOKENS  16

static uint32_t elapsed_us(XTime t0, XTime t1)
{
    return (uint32_t)(((t1 - t0) * 1000000ULL) / COUNTS_PER_SECOND);
}

static int generate(const char *prompt, int temp, char *out, size_t outsz,
                    uint32_t *hw_us, int *ntok)
{
    int8_t x[LLM_D], y[LLM_D];
    int    n = tok_encode(prompt, x, LLM_D);      /* 스택 스크래치패드로 복사된 상태에서 처리 */
    size_t pos = 0;
    int    step;

    out[0] = 0; *hw_us = 0; *ntok = 0;
    if (n == 0) return 0;                          /* 빈 프롬프트 */

    for (step = 0; step < MAX_NEW_TOKENS; step++) {
        XTime t0, t1;
        int tok;
        const char *wd;

        XTime_GetTime(&t0);
        if (llm_hw_run(x, y, HW_SHIFT)) return -1;
        XTime_GetTime(&t1);
        *hw_us += elapsed_us(t0, t1);

        tok = sample_token(y, LLM_VOCAB, temp);
        wd  = tok_word(tok);
        pos += (size_t)snprintf(out + pos, outsz - pos, "%s%s", pos ? " " : "", wd);
        if (pos >= outsz) pos = outsz - 1;
        (*ntok)++;

        if (tok == TOK_STOP) break;                /* 종료 토큰 */

        if (n < LLM_D) x[n++] = (int8_t)tok;       /* 컨텍스트에 추가 / 슬라이딩 */
        else { memmove(x, x + 1, LLM_D - 1); x[LLM_D - 1] = (int8_t)tok; }
    }
    return 0;
}

static void send_str(int sd, const char *s) { lwip_write(sd, s, strlen(s)); }

static void process_line(int sd, char *line)
{
    char reply[640];

    if (strncmp(line, "SELFTEST", 8) == 0) {
        int8_t x[LLM_D], y[LLM_D];
        XTime t0, t1;
        int i, len;
        for (i = 0; i < LLM_D; i++) x[i] = (int8_t)(i % 128);
        XTime_GetTime(&t0);
        if (llm_hw_run(x, y, HW_SHIFT)) { send_str(sd, "ERR|hw\n"); return; }
        XTime_GetTime(&t1);
        len = snprintf(reply, sizeof reply, "SELFTEST|%lu|", (unsigned long)elapsed_us(t0, t1));
        for (i = 0; i < 32; i++) len += snprintf(reply + len, sizeof reply - len, "%02x", (uint8_t)y[i]);
        snprintf(reply + len, sizeof reply - len, "\n");
        send_str(sd, reply);
        return;
    }

    if (strncmp(line, "TEMP:", 5) == 0) {
        char *bar = strchr(line, '|');
        int temp; char text[560]; uint32_t hw_us; int nt;
        if (!bar) { send_str(sd, "ERR|format\n"); return; }
        *bar = 0;
        temp = atoi(line + 5);
        if (temp < 0) temp = 0;
        if (temp > 100) temp = 100;
        if (generate(bar + 1, temp, text, sizeof text, &hw_us, &nt)) {
            send_str(sd, "ERR|hw\n"); return;
        }
        snprintf(reply, sizeof reply, "RES|%lu|%d|%s\n", (unsigned long)hw_us, nt, text);
        send_str(sd, reply);
        return;
    }
    send_str(sd, "ERR|unknown\n");
}

static void handle_client(int sd)
{
    char line[LINE_MAX_LEN];
    char rbuf[256];
    int  len = 0, n, i;

    for (;;) {
        n = lwip_recv(sd, rbuf, sizeof rbuf, 0);
        if (n <= 0) break;                         /* 연결 종료/오류 */
        for (i = 0; i < n; i++) {
            char c = rbuf[i];
            if (c == '\r') continue;
            if (c == '\n') { line[len] = 0; process_line(sd, line); len = 0; }
            else if (len < LINE_MAX_LEN - 1) line[len++] = c;
        }
    }
}

void inference_server_thread(void *arg)
{
    int sd, nsd;
    struct sockaddr_in addr, remote;
    socklen_t size = sizeof remote;
    (void)arg;

    if (llm_hw_init()) { xil_printf("llm_hw_init failed\r\n"); vTaskDelete(NULL); return; }
    sampler_seed();

    sd = lwip_socket(AF_INET, SOCK_STREAM, 0);
    memset(&addr, 0, sizeof addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(SERVER_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (lwip_bind(sd, (struct sockaddr *)&addr, sizeof addr) < 0) { xil_printf("bind failed\r\n"); vTaskDelete(NULL); return; }
    lwip_listen(sd, 0);
    xil_printf("LLM server listening on port %d\r\n", SERVER_PORT);

    for (;;) {
        nsd = lwip_accept(sd, (struct sockaddr *)&remote, &size);
        if (nsd < 0) continue;
        handle_client(nsd);
        lwip_close(nsd);
    }
}
```
> `vTaskDelete`를 쓰므로 `#include "FreeRTOS.h"`, `#include "task.h"`가 필요하면 추가하십시오(템플릿 헤더에서 이미 포함되는 경우가 많습니다).

### 9.6 `main.c` 수정 (템플릿 기준)
템플릿의 `main.c`에서 두 가지만 바꿉니다.
```c
#include "infer_server.h"           /* 추가 */
...
/* network_thread() 안, 에코 서버를 만들던 줄을 교체 */
sys_thread_new("llm_srv", inference_server_thread, 0,
               4096,                /* 스택(워드 단위, 약 16 KB) */
               DEFAULT_THREAD_PRIO);
```
- `echo_application_thread` 생성 줄과 `print_echo_app_header()` 호출은 제거합니다.
- 스택 4096 워드는 `generate()`/`handle_client()`의 지역 배열(약 2~3 KB) + lwIP 호출 깊이를 충분히 덮습니다. 스택 오버플로 의심 시 `check_for_stack_overflow`를 켜고 크기를 늘립니다.

---

## 10. 가중치 적재 및 실행

가중치는 앱이 시작하기 **전에** DDR `0x1F000000`에 올라가 있어야 합니다.

### 10.1 방법 A: Vitis 디버거 + XSCT 콘솔 (권장)
1. 앱 프로젝트 우클릭 → **Debug As → Launch on Hardware (Single Application Debug)** → `main` 진입에서 정지.
2. **Xilinx → XSCT Console** 열기(현재 ARM 코어 타깃이 선택된 상태):
   ```tcl
   dow -data C:/zybo_llm/out/weights.bin 0x1F000000
   mrd 0x1F000000 4        ;# 값이 비어있지 않은지 확인
   con                     ;# 실행 재개
   ```
3. UART 터미널(115200)에 `LLM server listening on port 5001`이 보이면 성공.

### 10.2 방법 B: 순수 XSCT 스크립트
```tcl
connect
targets -set -filter {name =~ "APU*"}
rst -system
fpga -file design_1_wrapper.bit
targets -set -filter {name =~ "ARM*#0"}
source ps7_init.tcl
ps7_init
ps7_post_config
dow llm_app.elf
dow -data weights.bin 0x1F000000
con
```
(`ps7_init.tcl`은 플랫폼의 `hw/` 폴더에 있습니다.)

### 10.3 네트워크 확인
```
> ping 192.168.1.10
```
응답이 없으면 §14를 참고합니다.

---

## 11. C# WinForms 클라이언트

Visual Studio 2022 → **Windows Forms App (.NET Framework)**, 대상 **.NET Framework 4.8**, 프로젝트명 `LlmClient`. 컨트롤을 코드로 만들어 디자이너 파일 없이 동작합니다. (C# 7.3 호환 문법)

### 11.1 `Program.cs`
```csharp
using System;
using System.Windows.Forms;

namespace LlmClient
{
    static class Program
    {
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MainForm());
        }
    }
}
```

### 11.2 `MainForm.cs`
```csharp
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;

namespace LlmClient
{
    public class MainForm : Form
    {
        private readonly TextBox txtHost = new TextBox { Text = "192.168.1.10", Width = 110 };
        private readonly NumericUpDown numTemperature =
            new NumericUpDown { Minimum = 0, Maximum = 100, Value = 45, Width = 60 };
        private readonly Button btnConnect = new Button { Text = "Connect", AutoSize = true };
        private readonly Button btnSend = new Button { Text = "Send", AutoSize = true, Enabled = false };
        private readonly Button btnSelfTest = new Button { Text = "SelfTest", AutoSize = true, Enabled = false };
        private readonly Button btnExportTelemetry = new Button { Text = "Export CSV", AutoSize = true };
        private readonly TextBox txtPrompt = new TextBox
            { Multiline = true, Dock = DockStyle.Fill, Text = "the cat mouse dog mat over" };
        private readonly TextBox txtOutput = new TextBox
            { Multiline = true, ReadOnly = true, ScrollBars = ScrollBars.Vertical, Dock = DockStyle.Fill };
        private readonly Chart chartLatency = new Chart { Dock = DockStyle.Fill };
        private readonly Series seriesLatency = new Series("Round-trip (ms)")
            { ChartType = SeriesChartType.Line, BorderWidth = 2, MarkerStyle = MarkerStyle.Circle };

        private TcpClient _client;
        private NetworkStream _stream;
        private StreamReader _reader;
        private readonly List<string> _log = new List<string>();
        private int _count;

        public MainForm()
        {
            Text = "Zybo LLM Accelerator Client";
            Width = 900; Height = 720;

            var top = new FlowLayoutPanel { Dock = DockStyle.Fill, AutoSize = true, WrapContents = false };
            top.Controls.Add(new Label { Text = "Host", AutoSize = true, Padding = new Padding(0, 6, 0, 0) });
            top.Controls.Add(txtHost);
            top.Controls.Add(new Label { Text = "Temp %", AutoSize = true, Padding = new Padding(8, 6, 0, 0) });
            top.Controls.Add(numTemperature);
            top.Controls.Add(btnConnect);
            top.Controls.Add(btnSend);
            top.Controls.Add(btnSelfTest);
            top.Controls.Add(btnExportTelemetry);

            var area = new ChartArea("main");
            area.AxisX.Title = "Inference #";
            area.AxisY.Title = "ms";
            chartLatency.ChartAreas.Add(area);
            chartLatency.Series.Add(seriesLatency);
            chartLatency.Legends.Add(new Legend("l"));

            var layout = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 4 };
            layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 20));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 30));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 50));
            layout.Controls.Add(top, 0, 0);
            layout.Controls.Add(txtPrompt, 0, 1);
            layout.Controls.Add(txtOutput, 0, 2);
            layout.Controls.Add(chartLatency, 0, 3);
            Controls.Add(layout);

            btnConnect.Click += BtnConnect_Click;
            btnSend.Click += BtnSend_Click;
            btnSelfTest.Click += BtnSelfTest_Click;
            btnExportTelemetry.Click += BtnExport_Click;
            FormClosing += (s, e) => { if (_client != null) _client.Close(); };
        }

        private async void BtnConnect_Click(object sender, EventArgs e)
        {
            try
            {
                if (_client != null) { _client.Close(); _client = null; }
                _client = new TcpClient { NoDelay = true };
                await _client.ConnectAsync(txtHost.Text.Trim(), 5001);
                _stream = _client.GetStream();
                _reader = new StreamReader(_stream, Encoding.ASCII);
                btnSend.Enabled = true; btnSelfTest.Enabled = true;
                Append("[connected]");
            }
            catch (Exception ex) { Append("[connect error] " + ex.Message); }
        }

        private async System.Threading.Tasks.Task<string> RoundTrip(string line, Stopwatch sw)
        {
            byte[] data = Encoding.ASCII.GetBytes(line + "\n");
            sw.Restart();                                   // 소켓 쓰기 직전부터
            await _stream.WriteAsync(data, 0, data.Length);
            string resp = await _reader.ReadLineAsync();     // 응답 헤더 수신까지
            sw.Stop();
            return resp;
        }

        private async void BtnSend_Click(object sender, EventArgs e)
        {
            if (_stream == null) return;
            btnSend.Enabled = false;
            try
            {
                int temp = (int)numTemperature.Value;
                string prompt = txtPrompt.Text.Replace('\r', ' ').Replace('\n', ' ');
                var sw = new Stopwatch();
                string resp = await RoundTrip("TEMP:" + temp + "|" + prompt, sw);
                double ms = sw.Elapsed.TotalMilliseconds;

                if (resp == null) { Append("[disconnected]"); return; }
                string[] p = resp.Split(new[] { '|' }, 4);
                if (p.Length == 4 && p[0] == "RES")
                {
                    double hwMs = double.Parse(p[1]) / 1000.0;
                    Append(string.Format("[{0:F2} ms, hw {1:F2} ms, {2} tok] {3}", ms, hwMs, p[2], p[3]));
                    _count++;
                    seriesLatency.Points.AddXY(_count, ms);
                    _log.Add(string.Format("{0},{1:o},{2},{3:F3},{4:F3},{5},\"{6}\"",
                        _count, DateTime.Now, temp, ms, hwMs, p[2], p[3].Replace("\"", "\"\"")));
                }
                else Append("[server] " + resp);
            }
            catch (Exception ex) { Append("[error] " + ex.Message); }
            finally { btnSend.Enabled = true; }
        }

        private async void BtnSelfTest_Click(object sender, EventArgs e)
        {
            if (_stream == null) return;
            try
            {
                var sw = new Stopwatch();
                string resp = await RoundTrip("SELFTEST", sw);
                Append("[selftest " + sw.Elapsed.TotalMilliseconds.ToString("F2") + " ms] " + resp);
            }
            catch (Exception ex) { Append("[error] " + ex.Message); }
        }

        private void BtnExport_Click(object sender, EventArgs e)
        {
            using (var dlg = new SaveFileDialog { Filter = "CSV|*.csv", FileName = "telemetry.csv" })
            {
                if (dlg.ShowDialog() != DialogResult.OK) return;
                var lines = new List<string> { "index,timestamp,temp,roundtrip_ms,hw_ms,tokens,text" };
                lines.AddRange(_log);
                File.WriteAllLines(dlg.FileName, lines, Encoding.UTF8);
                Append("[exported] " + dlg.FileName);
            }
        }

        private void Append(string s) { txtOutput.AppendText(s + Environment.NewLine); }
    }
}
```
- 프롬프트는 **ASCII 영문**만 지원합니다(어휘 128개).
- .NET 6/8로 만들 경우 Chart를 쓰려면 NuGet `System.Windows.Forms.DataVisualization`(프리릴리스)을 추가해야 합니다.

---

## 12. 단계별 검증 절차

| 단계 | 확인 항목 | 합격 기준 |
|---|---|---|
| 1 | Python 골든 | `weights.bin` = 1,310,720 B, 20레이어 후에도 `nonzero` 유지 |
| 2 | RTL 시뮬레이션 | `PASS: 32 beats matched` |
| 3 | 비트스트림 | 타이밍 WNS ≥ 0, Validate 에러 0 |
| 4 | UART | `LLM server listening on port 5001` |
| 5 | `ping 192.168.1.10` | 응답 정상 |
| 6 | **SELFTEST (비트 정확성)** | 클라이언트 `SelfTest` 응답의 hex가 Python 출력 `SELFTEST first 32 bytes:`와 **완전히 일치** |
| 7 | Send | `RES|…` 응답, 같은 프롬프트/Temp=0에서 결과가 항상 동일 |
| 8 | Temp 변화 | Temp > 0 에서 반복 전송 시 결과가 달라짐 |

> **6단계가 핵심입니다.** 이것이 일치하면 DMA 정렬, 엔디안, 뱅크 핑퐁, 요양자화가 모두 맞다는 뜻입니다. 이후 문제는 대부분 네트워크/토크나이저 쪽입니다.
> (난수 가중치에서는 Temp=0 출력이 `[PAD]`를 제외한 한 토큰으로 수렴할 수 있습니다. 하드웨어 오류가 아니라 모델이 학습되지 않은 탓입니다.)

---

## 13. 성능 분석과 최적화 체크리스트

**이론값 (100 MHz)**
| 항목 | 값 |
|---|---|
| 레이어당 연산 | 256행 × 32 beat = 8,192 클럭 ≈ **82 µs** |
| 레이어당 가중치 적재 (64 KB @ 8 B/clk) | ≈ 82 µs (연산과 **겹침**) |
| 토큰당 (20 레이어) | ≈ 1.7 ms + 펌웨어/폴링 오버헤드 → **약 2 ms** |
| 16 토큰 요청 | 약 30 ms + 네트워크 왕복 |

토큰당 시간은 `RES`의 `hw_ms`를 토큰 수로 나눠 확인합니다. 비교용으로 SELFTEST의 `hw_us`(= 1회 20레이어 실행)를 보면 이론값과 맞는지 바로 알 수 있습니다.

**체크리스트 (보고서 최적화 이력에서 얻은 교훈)**
1. 연산 루프 안에서 `xil_printf` **금지**: 한 번에 수 ms. 보고서의 7초 병목이 정확히 이것이었습니다.
2. `usleep` 대신 **DMA 상태 레지스터/DONE 비트 폴링**.
3. DMA 버퍼 **32 B 정렬** + 전송 전 Flush / 수신 후 Invalidate.
4. 큰 버퍼는 스택이 아닌 `static`(가중치는 DDR 고정 주소), 프롬프트 처리용 작은 버퍼만 스레드 스택.
5. 컨텍스트가 짧아도 항상 256으로 [PAD] 채움 → 하드웨어 입력 크기 고정.
6. 최적화가 필요하면: 레이어 간 오버헤드(GPIO 쓰기 3~4회)를 줄이거나, 소프트웨어 대신 PL에 레이어 시퀀서를 넣는 순서로 진행.

---

## 14. 트러블슈팅

| 증상 | 원인 후보 | 조치 |
|---|---|---|
| `llm_hw_init failed` | `xparameters.h`의 DEVICE_ID 매크로명 불일치, DMA에 SG가 켜짐 | 매크로명 확인, DMA 설정에서 SG 해제 |
| DMA 에러 (SR의 DMASlvErr/DMADecErr) | HP0 비활성, 주소 매핑 누락, 가중치 주소 범위 밖 | PS 설정과 Address Editor 확인, `0x1F000000`이 DDR 범위(0~0x3FFFFFFF) 내인지 |
| DMA는 끝나는데 `DONE` 영원히 안 옴 | `ctrl`/`status` 배선 반대, 리셋 극성, 클럭 미연결 | ILA/시뮬레이션으로 `start_pulse`·`busy` 확인, `rstn`이 `peripheral_aresetn`인지 |
| `wait_done` 타임아웃 | START가 에지로 인식 안 됨 | `ctrl[0]`이 PS GPIO에서 정상 변화하는지 (`XGpio_DiscreteWrite` 채널 번호 1 확인) |
| SELFTEST 불일치 | 엔디안/배치 불일치, SHIFT 불일치, 가중치 미적재 | `mrd 0x1F000000`으로 적재 확인, `HW_SHIFT`와 Python SHIFT 일치, §4.1 배치 재확인 |
| 첫 요청은 되는데 이후 깨짐 | 뱅크 충돌(쓰기=연산 뱅크), 캐시 | `WBANK`가 항상 `CBANK`의 반대인지, Invalidate 호출 위치 |
| 연결 직후 끊김 / 간헐적 끊김 | 스레드 스택 부족, 힙 부족 | 스택 4096+ 워드, `total_heap_size` 증가, 스택 오버플로 훅 활성 |
| `ping` 불가 | IP 대역 불일치, 링크 미성립 | PC 고정 IP 확인, UART의 IP/링크 메시지 확인, 케이블/포트 점검 |
| `expf` 링크 에러 | libm 누락 | 링커 설정에 `-lm` 추가 |
| 결과가 항상 같은 단어 | 학습 안 된 난수 가중치, 로짓 전부 0 | 정상 현상. SHIFT를 줄여 활성이 0으로 죽지 않게 하거나 학습된 가중치 사용 |

---

## 15. 확장 로드맵

1. **ACP로 전환**: PS에서 `S_AXI_ACP` 활성화, DMA 마스터를 ACP에 연결하고 `ARUSER/AWUSER`를 1로 고정(xlconstant), AxCACHE 조건은 UG585 ACP 장을 확인합니다. 성공하면 Flush/Invalidate를 제거해도 되는지 검증합니다.
2. **진짜 어텐션**: 레이어당 Wq/Wk/Wv/Wo(4×64 KB)로 가중치를 확장하고, KV 캐시용 BRAM, 소프트맥스(LUT 기반 exp + 정수 정규화)를 PL에 추가합니다. 헤드 분할은 지금처럼 32차원씩 8개를 유지합니다.
3. **LayerNorm/잔차 연결**: 정수 근사 RMSNorm, 잔차 덧셈(포화)을 요양자화 단 앞에 삽입합니다.
4. **학습된 모델**: PyTorch에서 d=256, 8헤드 소형 모델을 학습 → INT8 QAT/PTQ → 레이어별 SHIFT 산출 → `weights.bin` 규격(§4.1)으로 내보내기. 레이어별 SHIFT는 START 시점에 `ctrl[15:8]`을 바꾸면 이미 지원됩니다.
5. **처리량 개선**: 8레인 → 16/32레인으로 확장(스트림 폭 128/256비트), 행 2개 동시 처리, DSP 패킹(2 MAC/DSP).
6. **무 JTAG 부팅**: FSBL + QSPI/SD 부팅, `xilffs`로 SD의 `weights.bin`을 `0x1F000000`에 로드.

---

## 부록 A. 파일/프로토콜 요약

**가중치 파일**: `[20][256][256]` INT8 row-major, 레이어 = 65,536 B, 64비트 beat 리틀엔디안
**DMA 전송 크기**: x = 256 B(32 beat), 가중치 = 65,536 B(8192 beat), 결과 = 256 B(32 beat)
**ctrl 예시** (SHIFT=7): x 적재 `0x0704` / 가중치→ping `0x0700` / 가중치→pong `0x0708` / 연산 시작(ping) `0x0701`, (pong) `0x0711` / 출력 `0x0702`
**TCP**: 포트 5001, 요청 `TEMP:<0-100>|<text>\n`, 응답 `RES|<hw_us>|<n>|<text>\n`
