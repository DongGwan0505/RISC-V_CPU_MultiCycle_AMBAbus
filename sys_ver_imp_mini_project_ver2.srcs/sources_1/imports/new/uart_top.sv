`timescale 1ns / 1ps

module UART_Periph (
    input  logic        PCLK, PRESET,
    input  logic [31:0] PADDR,                  // 바이트 주소
    input  logic        PWRITE, PENABLE, PSEL,
    input  logic [31:0] PWDATA,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    input  logic        rx,
    output logic        tx,

    // ★ 추가: 최근 전송 시작한 TX 바이트 (LED로 보낼 값)
    output logic [7:0]  tx_byte_dbg
);
    logic uart_en, tx_wr_pulse, rx_rd_pulse;
    logic [7:0] tx_wdata, rx_rdata;
    logic rx_empty, tx_full, tx_busy;
    logic rx_done_pulse, tx_overrun_pulse;

    APB_SlaveIntf_UART U_APB_IF (
        .PCLK(PCLK), .PRESET(PRESET),
        .PADDR(PADDR), .PWRITE(PWRITE), .PENABLE(PENABLE), .PWDATA(PWDATA), .PSEL(PSEL),
        .PRDATA(PRDATA), .PREADY(PREADY),
        .uart_en(uart_en), .tx_wr_pulse(tx_wr_pulse), .tx_wdata(tx_wdata),
        .rx_rdata(rx_rdata), .rx_rd_pulse(rx_rd_pulse),
        .rx_empty(rx_empty), .tx_full(tx_full), .tx_busy(tx_busy),
        .rx_done_pulse(rx_done_pulse), .tx_overrun_pulse(tx_overrun_pulse)
    );

    uart_apb_core U_CORE (
        .clk(PCLK), .rst(PRESET), .rx(rx), .tx(tx), .uart_en(uart_en),
        .tx_wr(tx_wr_pulse), .tx_wdata(tx_wdata), .tx_full(tx_full), .tx_busy(tx_busy),
        .tx_overrun_pulse(tx_overrun_pulse),
        .rx_rd(rx_rd_pulse), .rx_rdata(rx_rdata), .rx_empty(rx_empty), .rx_done_pulse(rx_done_pulse),

        // ★ 코어에서 나온 디버그 바이트를 그대로 노출
        .tx_byte_dbg(tx_byte_dbg)
    );
endmodule

/*
module UART_Periph (
    input  logic        PCLK, PRESET,
    input  logic [31:0] PADDR,                  // 바이트 주소
    input  logic        PWRITE, PENABLE, PSEL,
    input  logic [31:0] PWDATA,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    input  logic        rx,
    output logic        tx
);
    logic uart_en, tx_wr_pulse, rx_rd_pulse;
    logic [7:0] tx_wdata, rx_rdata;
    logic rx_empty, tx_full, tx_busy;
    logic rx_done_pulse, tx_overrun_pulse;

    APB_SlaveIntf_UART U_APB_IF (
        .PCLK(PCLK), .PRESET(PRESET),
        .PADDR(PADDR), .PWRITE(PWRITE), .PENABLE(PENABLE), .PWDATA(PWDATA), .PSEL(PSEL),
        .PRDATA(PRDATA), .PREADY(PREADY),
        .uart_en(uart_en), .tx_wr_pulse(tx_wr_pulse), .tx_wdata(tx_wdata),
        .rx_rdata(rx_rdata), .rx_rd_pulse(rx_rd_pulse),
        .rx_empty(rx_empty), .tx_full(tx_full), .tx_busy(tx_busy),
        .rx_done_pulse(rx_done_pulse), .tx_overrun_pulse(tx_overrun_pulse)
    );

    uart_apb_core U_CORE (
        .clk(PCLK), .rst(PRESET), .rx(rx), .tx(tx), .uart_en(uart_en),
        .tx_wr(tx_wr_pulse), .tx_wdata(tx_wdata), .tx_full(tx_full), .tx_busy(tx_busy),
        .tx_overrun_pulse(tx_overrun_pulse),
        .rx_rd(rx_rd_pulse), .rx_rdata(rx_rdata), .rx_empty(rx_empty), .rx_done_pulse(rx_done_pulse)
    );
endmodule
*/

module APB_SlaveIntf_UART (
    input  logic        PCLK, PRESET,
    input  logic [31:0] PADDR,        // 바이트 주소
    input  logic        PWRITE, PENABLE, PSEL,
    input  logic [31:0] PWDATA,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    output logic        uart_en,
    output logic        tx_wr_pulse,
    output logic [7:0]  tx_wdata,
    input  logic [7:0]  rx_rdata,
    output logic        rx_rd_pulse,
    input  logic        rx_empty, tx_full, tx_busy,
    input  logic        rx_done_pulse, tx_overrun_pulse
);
    wire apb_access = PSEL & PENABLE;
    wire wr_cycle   = apb_access &  PWRITE;
    wire rd_cycle   = apb_access & ~PWRITE;
    assign PREADY = apb_access;
    assign tx_wdata = PWDATA[7:0];

    logic [31:0] CR, SR, TXD_SH, RXD_LATCH;
    logic rx_done_sticky, tx_overrun_sticky;

    assign uart_en = CR[0];

    // 펄스 생성
    always_comb begin
        tx_wr_pulse = (wr_cycle && (PADDR[3:2]==2'd2));
        rx_rd_pulse = (rd_cycle && (PADDR[3:2]==2'd3));
    end

    // SR 미러
    wire [31:0] SR_next = {27'b0, tx_overrun_sticky, rx_done_sticky, tx_busy, tx_full, rx_empty};

    always_ff @(posedge PCLK or posedge PRESET) begin
        if (PRESET) begin
            CR<=32'h1; SR<=0; TXD_SH<=0; RXD_LATCH<=0;
            rx_done_sticky<=0; tx_overrun_sticky<=0;
        end else begin
            SR         <= SR_next;
            RXD_LATCH  <= {24'b0, rx_rdata};
            if (rx_done_pulse)    rx_done_sticky    <= 1'b1;
            if (tx_overrun_pulse) tx_overrun_sticky <= 1'b1;

            if (apb_access) begin
                if (PWRITE) begin
                    case (PADDR[3:2])
                        2'd0: CR      <= PWDATA;     // CR
                        2'd2: TXD_SH  <= PWDATA;     // TXD shadow
                        2'd1: begin                  // SR write-1-to-clear
                            rx_done_sticky    <= rx_done_sticky    & ~PWDATA[3];
                            tx_overrun_sticky <= tx_overrun_sticky & ~PWDATA[4];
                        end
                    endcase
                end else begin
                    if (PADDR[3:2]==2'd1) begin
                        rx_done_sticky    <= 1'b0;   // SR read → sticky clear
                        tx_overrun_sticky <= 1'b0;
                    end
                end
            end
        end
    end

    always_comb begin
        case (PADDR[3:2])
            2'd0: PRDATA = CR;
            2'd1: PRDATA = SR;
            2'd2: PRDATA = TXD_SH;
            2'd3: PRDATA = RXD_LATCH;
            default: PRDATA = 32'h0;
        endcase
    end
endmodule

module uart_apb_core (
    input  logic       clk,
    input  logic       rst,
    // external pins
    input  logic       rx,
    output logic       tx,
    // control
    input  logic       uart_en,         // CR[0]
    // TX side (APB → UART)
    input  logic       tx_wr,           // 1-cycle pulse: write to TX FIFO
    input  logic [7:0] tx_wdata,
    output logic       tx_full,
    output logic       tx_busy,
    output logic       tx_overrun_pulse,// set when wr while full
    // RX side (UART → APB)
    input  logic       rx_rd,           // 1-cycle pulse: pop from RX FIFO
    output logic [7:0] rx_rdata,        // front data (combinational)
    output logic       rx_empty,
    output logic       rx_done_pulse,   // 1-cycle pulse from UART RX

    // ★ 추가: 이번 프레임에 실제로 송신되는 바이트(LED로 보낼 값)
    output logic [7:0] tx_byte_dbg
);
    // baud tick
    wire b_tick;
    baud_tick_gen #(
      .CLK_HZ(100_000_000),  // XDC 기준: 100 MHz
      .BAUD  (9600),         // ComPortMaster 설정과 동일
      .OS    (16)
    ) U_BAUD_TICK (
      .clk(clk),
      .rst(rst),
      .baud_tick(b_tick)
    );

    // UART RX
    wire       w_rx_done, w_rx_busy;
    wire [7:0] w_rx_byte;

    uart_rx U_UART_RX (
        .clk(clk), .rst(rst | ~uart_en),
        .b_tick(b_tick),
        .rx(rx),
        .rx_data(w_rx_byte),
        .rx_busy(w_rx_busy),
        .rx_done(w_rx_done)
    );

    // 1-cycle pulse shaping for rx_done
    reg rx_done_d;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) rx_done_d <= 1'b0;
        else     rx_done_d <= w_rx_done;
    end
    assign rx_done_pulse = w_rx_done & ~rx_done_d;

    // RX FIFO
    wire [7:0] rx_fifo_rdata;
    wire       rx_fifo_full;
    fifo U_RX_FIFO (
        .clk(clk), .rst(rst | ~uart_en),
        .wr(rx_done_pulse),                 // push when a byte is received
        .rd(rx_rd),                         // pop when APB reads RXD
        .wdata(w_rx_byte),
        .rdata(rx_fifo_rdata),
        .full(rx_fifo_full),
        .empty(rx_empty)
    );
    assign rx_rdata = rx_fifo_rdata;

    // TX FIFO
    wire [7:0] tx_fifo_rdata;
    wire       tx_fifo_empty;
    fifo U_TX_FIFO (
        .clk(clk), .rst(rst | ~uart_en),
        .wr(tx_wr & ~tx_full),              // push only if not full
        .rd(~tx_fifo_empty & ~tx_busy),     // feed UART TX when ready
        .wdata(tx_wdata),
        .rdata(tx_fifo_rdata),
        .full(tx_full),
        .empty(tx_fifo_empty)
    );
    assign tx_overrun_pulse = (tx_wr & tx_full); // one-cycle event

    // UART TX
    wire w_tx_start = ~tx_fifo_empty & ~tx_busy;

    // ★ 전송 "시작" 순간의 바이트를 래치 → 프레임 동안 LED에 안정적으로 표시
    always_ff @(posedge clk or posedge rst) begin
        if (rst)              tx_byte_dbg <= 8'h00;
        else if (~uart_en)    tx_byte_dbg <= 8'h00;
        else if (w_tx_start)  tx_byte_dbg <= tx_fifo_rdata;
        // 그 외에는 유지
    end

    uart_tx #(.STOP_BITS(1)) U_UART_TX (
        .clk(clk), .rst(rst | ~uart_en),
        .b_tick(b_tick),
        .tx_start(w_tx_start),
        .tx_data(tx_fifo_rdata),
        .busy(tx_busy),
        .done(),
        .tx(tx)
    );
endmodule


/*
module uart_apb_core (
    input  logic       clk,
    input  logic       rst,
    // external pins
    input  logic       rx,
    output logic       tx,
    // control
    input  logic       uart_en,         // CR[0]
    // TX side (APB → UART)
    input  logic       tx_wr,           // 1-cycle pulse: write to TX FIFO
    input  logic [7:0] tx_wdata,
    output logic       tx_full,
    output logic       tx_busy,
    output logic       tx_overrun_pulse,// set when wr while full
    // RX side (UART → APB)
    input  logic       rx_rd,           // 1-cycle pulse: pop from RX FIFO
    output logic [7:0] rx_rdata,        // front data (combinational)
    output logic       rx_empty,
    output logic       rx_done_pulse    // 1-cycle pulse from UART RX
);
    // baud tick
    wire b_tick;
    baud_tick_gen #(
      .CLK_HZ(100_000_000),  // XDC 기준: 100 MHz
      .BAUD  (9600),         // ComPortMaster 설정과 동일
      .OS    (16)
    ) U_BAUD_TICK (
      .clk(clk),
      .rst(rst),
      .baud_tick(b_tick)
    );

    // UART RX
    wire       w_rx_done, w_rx_busy;
    wire [7:0] w_rx_byte;

    uart_rx U_UART_RX (
        .clk(clk), .rst(rst | ~uart_en),
        .b_tick(b_tick),
        .rx(rx),
        .rx_data(w_rx_byte),
        .rx_busy(w_rx_busy),
        .rx_done(w_rx_done)
    );

    // 1-cycle pulse shaping for rx_done
    reg rx_done_d;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) rx_done_d <= 1'b0;
        else     rx_done_d <= w_rx_done;
    end
    assign rx_done_pulse = w_rx_done & ~rx_done_d;

    // RX FIFO
    wire [7:0] rx_fifo_rdata;
    wire       rx_fifo_full;
    fifo U_RX_FIFO (
        .clk(clk), .rst(rst | ~uart_en),
        .wr(rx_done_pulse),                 // push when a byte is received
        .rd(rx_rd),                         // pop when APB reads RXD
        .wdata(w_rx_byte),
        .rdata(rx_fifo_rdata),
        .full(rx_fifo_full),
        .empty(rx_empty)
    );
    assign rx_rdata = rx_fifo_rdata;

    // TX FIFO
    wire [7:0] tx_fifo_rdata;
    wire       tx_fifo_empty;
    fifo U_TX_FIFO (
        .clk(clk), .rst(rst | ~uart_en),
        .wr(tx_wr & ~tx_full),              // push only if not full
        .rd(~tx_fifo_empty & ~tx_busy),     // feed UART TX when ready
        .wdata(tx_wdata),
        .rdata(tx_fifo_rdata),
        .full(tx_full),
        .empty(tx_fifo_empty)
    );
    assign tx_overrun_pulse = (tx_wr & tx_full); // one-cycle event

    // UART TX
    wire w_tx_start = ~tx_fifo_empty & ~tx_busy;
        uart_tx #(.STOP_BITS(1)) U_UART_TX (
        .clk(clk), .rst(rst | ~uart_en),
        .b_tick(b_tick),
        .tx_start(w_tx_start),
        .tx_data(tx_fifo_rdata),
        .busy(tx_busy),
        .done(),
        .tx(tx)
    );
endmodule
*/

module uart_rx(
    input  logic       clk,
    input  logic       rst,
    input  logic       rx,       // idle=1
    input  logic       b_tick,   // 16x oversample tick
    output logic       rx_busy,
    output logic       rx_done,  // 1-cycle pulse
    output logic [7:0] rx_data
);

  localparam int OS_TICKS = 16;
  localparam int HALF_BIT = OS_TICKS/2;   // 8
  localparam int BIT_LAST = OS_TICKS-1;   // 15
  typedef enum logic [1:0] {IDLE, START, DATA, STOP} state_t;

  // 입력 동기화
  logic rx_m, rx_s, rx_s_q;
  always_ff @(posedge clk) begin
    rx_m   <= rx;
    rx_s   <= rx_m;
    rx_s_q <= rx_s;
  end
  wire start_fall = (rx_s_q == 1'b1) && (rx_s == 1'b0);

  // 상태/카운터/데이터
  state_t state, state_n;
  logic [3:0] tick_cnt, tick_cnt_n; // 0..15
  logic [2:0] bit_idx,  bit_idx_n;  // 0..7
  logic [7:0] data_r,   data_r_n;
  logic       rx_done_n;

  assign rx_busy = (state != IDLE);
  assign rx_data = data_r;

  // Sequential
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      state    <= IDLE;
      tick_cnt <= '0;
      bit_idx  <= '0;
      data_r   <= '0;
      rx_done  <= 1'b0;
    end else begin
      state    <= state_n;
      tick_cnt <= tick_cnt_n;
      bit_idx  <= bit_idx_n;
      data_r   <= data_r_n;
      rx_done  <= rx_done_n;  // rx_done은 여기서만 드라이브 (1클록 펄스)
    end
  end

  // Combinational
  always_comb begin
    state_n    = state;
    tick_cnt_n = tick_cnt;
    bit_idx_n  = bit_idx;
    data_r_n   = data_r;
    rx_done_n  = 1'b0;

    unique case (state)
      IDLE: begin
        tick_cnt_n = '0;
        bit_idx_n  = '0;
        if (start_fall) begin
          state_n    = START;
          tick_cnt_n = '0;
        end
      end

      // 반 비트(8) 대기 후 데이터 비트 중앙 정렬
      START: begin
        if (b_tick) begin
          if (tick_cnt == HALF_BIT-1) begin      // ==7
            state_n    = DATA;
            tick_cnt_n = '0;
            bit_idx_n  = '0;
          end else tick_cnt_n = tick_cnt + 1;
        end
      end

      // 매 16틱 끝(==15)에서 샘플 (비트 중앙 보장)
      DATA: begin
        if (b_tick) begin
          if (tick_cnt == BIT_LAST) begin        // ==15
            data_r_n[bit_idx] = rx_s;            // LSB-first
            tick_cnt_n = '0;
            if (bit_idx == 3'd7) begin
              bit_idx_n = '0;
              state_n   = STOP;
            end else bit_idx_n = bit_idx + 1;
          end else tick_cnt_n = tick_cnt + 1;
        end
      end

      // 스톱 비트 끝(==15)에서 완료 펄스 + 다음 스타트 레벨(low)면 즉시 START로
      STOP: begin
        if (b_tick) begin
          if (tick_cnt == BIT_LAST) begin        // ==15
            tick_cnt_n = '0;
            rx_done_n  = 1'b1;                   // 1클록 완료 펄스
            // ★ 경계 보강: 다음 프레임 스타트가 이미 시작됐으면 START로 점프
            state_n    = (rx_s == 1'b0) ? START : IDLE;
          end else tick_cnt_n = tick_cnt + 1;
        end
      end

      default: begin
        state_n    = IDLE;
        tick_cnt_n = '0;
        bit_idx_n  = '0;
      end
    endcase
  end

endmodule

module uart_tx(
    input  logic       clk,
    input  logic       rst,
    input  logic       b_tick,      // 16x oversample tick
    input  logic       tx_start,    // 1=송신 시작 요청
    input  logic [7:0] tx_data,
    output logic       busy,        // 현재 송신 중
    output logic       done,        // 1클록 펄스 (프레임 완료)
    output logic       tx           // UART TX line (idle=1)
);
    parameter int OS_TICKS  = 16;   // 오버샘플 배수
    parameter int STOP_BITS = 2;    // 스톱 비트 수 (1 또는 2)

    typedef enum logic [1:0] {IDLE, START, DATA, STOP} state_t;

    localparam int BIT_LAST  = OS_TICKS-1;                  // 15
    localparam int STOP_LAST = STOP_BITS*OS_TICKS - 1;      // 1bit:15, 2bit:31
    localparam int TICK_W    = (STOP_BITS==1) ? $clog2(OS_TICKS)
                                             : $clog2(OS_TICKS*STOP_BITS);

    // 상태/카운터/시프트 레지스터
    state_t                 state, state_n;
    logic [TICK_W-1:0]      tick_cnt, tick_cnt_n;
    logic [2:0]             bit_cnt, bit_cnt_n;
    logic [7:0]             shreg,   shreg_n;

    // 출력 레지스터와 그 다음 값
    logic tx_r,   tx_r_n;
    logic busy_n;
    logic done_n;

    // ====== Sequential: 오직 여기에서만 출력/상태 레지스터 갱신 ======
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state    <= IDLE;
            tick_cnt <= '0;
            bit_cnt  <= '0;
            shreg    <= '0;
            tx_r     <= 1'b1;
            busy     <= 1'b0;
            done     <= 1'b0;
        end else begin
            state    <= state_n;
            tick_cnt <= tick_cnt_n;
            bit_cnt  <= bit_cnt_n;
            shreg    <= shreg_n;
            tx_r     <= tx_r_n;
            busy     <= busy_n;   // ★ busy는 여기서만 드라이브
            done     <= done_n;   // ★ done도 여기서만 드라이브 (1클록 펄스)
        end
    end

    // ====== Combinational: 다음 상태/출력(next) 계산만 ======
    always_comb begin
        // 기본 유지
        state_n    = state;
        tick_cnt_n = tick_cnt;
        bit_cnt_n  = bit_cnt;
        shreg_n    = shreg;
        tx_r_n     = tx_r;

        // 출력 next 기본값
        busy_n     = (state != IDLE); // 현재 상태 기준으로 busy 표시
        done_n     = 1'b0;            // 기본은 펄스 없음

        unique case (state)
          IDLE: begin
            tx_r_n = 1'b1;
            busy_n = 1'b0;
            if (tx_start) begin
              shreg_n    = tx_data;
              state_n    = START;
              tick_cnt_n = '0;
              busy_n     = 1'b1;
            end
          end

          // 스타트 비트(=0) 1비트 기간
          START: begin
            tx_r_n = 1'b0;
            if (b_tick) begin
              if (tick_cnt == BIT_LAST) begin
                tick_cnt_n = '0;
                bit_cnt_n  = '0;
                state_n    = DATA;
              end else tick_cnt_n = tick_cnt + 1;
            end
          end

          // 데이터 8비트 (LSB-first)
          DATA: begin
            tx_r_n = shreg[0];
            if (b_tick) begin
              if (tick_cnt == BIT_LAST) begin
                tick_cnt_n = '0;
                if (bit_cnt == 3'd7) begin
                  bit_cnt_n = '0;
                  state_n   = STOP;
                end else begin
                  bit_cnt_n = bit_cnt + 1;
                  shreg_n   = {1'b0, shreg[7:1]}; // right shift
                end
              end else tick_cnt_n = tick_cnt + 1;
            end
          end

          // 스톱 비트: STOP_BITS 비트 동안 '1' 유지
          STOP: begin
            tx_r_n = 1'b1;
            if (b_tick) begin
              if (tick_cnt == STOP_LAST) begin
                tick_cnt_n = '0;
                state_n    = IDLE;
                busy_n     = 1'b0;    // 다음 사이클부터 idle
                done_n     = 1'b1;    // 완료 펄스
              end else tick_cnt_n = tick_cnt + 1;
            end
          end

          default: begin
            state_n    = IDLE;
            tick_cnt_n = '0;
            bit_cnt_n  = '0;
            tx_r_n     = 1'b1;
            busy_n     = 1'b0;
            done_n     = 1'b0;
          end
        endcase
    end

    // 출력 배선
    assign tx = tx_r;

endmodule

module baud_tick_gen #(
    parameter int CLK_HZ = 100_000_000,
    parameter int BAUD   = 9600,
    parameter int OS     = 16               // oversample
)(
    input  logic clk,
    input  logic rst,
    output logic baud_tick                  // 16× 오버샘플 tick (1클록 펄스)
);
    // 100MHz/(9600*16) ≈ 651.041… → 651
    localparam int DIV = CLK_HZ / (BAUD*OS);
    localparam int W   = $clog2(DIV);

    logic [W-1:0] cnt;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt       <= '0;
            baud_tick <= 1'b0;
        end else begin
            if (cnt == DIV-1) begin          // ★ DIV-1과 비교
                cnt       <= '0;
                baud_tick <= 1'b1;           // 1클록 펄스
            end else begin
                cnt       <= cnt + 1;
                baud_tick <= 1'b0;
            end
        end
    end
endmodule


module fifo(
    input  logic       clk,
    input  logic       rst,
    input  logic       wr,
    input  logic       rd,
    input  logic [7:0] wdata,
    output logic [7:0] rdata,
    output logic       full,
    output logic       empty
);
    logic wr_en;
    logic [2:0] waddr;
    logic [2:0] raddr;

    assign wr_en = wr & ~full;

    //이런 식으로 모듈을 인스턴스 할 시 같은 이름을 가진 것들끼리 매칭시켜준다.
    register_file U_REG_FILE (
        .*,
        .wr(wr_en)
    );

    fifo_ctrl_unit U_fifo_CU (.*);

endmodule

module register_file 
#(parameter AWIDTH = 3)
(
    input  logic                      clk,
    input  logic                      wr,
    input  logic [7:0]                wdata,
    input  logic [AWIDTH-1:0] waddr,
    input  logic [AWIDTH-1:0] raddr,
    output logic [7:0]                rdata
);
    
    logic [7:0] ram [0:2**AWIDTH-1];

    assign rdata = ram[raddr];

    always_ff @(posedge clk) begin
        if (wr) begin
            ram[waddr] <= wdata;
        end
    end

endmodule

module fifo_ctrl_unit 
#(parameter AWIDTH = 3)
(
    input  logic                clk,
    input  logic                rst,
    input  logic                wr,
    input  logic                rd,
    output logic                full,
    output logic                empty,
    output logic [AWIDTH - 1:0] raddr,
    output logic [AWIDTH - 1:0] waddr
);
    logic [AWIDTH-1:0] c_waddr, n_waddr;
    logic [AWIDTH-1:0] c_raddr, n_raddr;

    logic c_full, n_full;
    logic c_empty, n_empty;

    assign empty = c_empty;
    assign full  = c_full;
    assign raddr = c_raddr;
    assign waddr = c_waddr;

    //state reg
    always_ff @( posedge clk, posedge rst) begin
        if (rst) begin
            c_waddr <= 0;
            c_raddr <= 0;
            c_full  <= 0;
            c_empty <= 1;
        end else begin
            c_waddr <= n_waddr;
            c_raddr <= n_raddr;
            c_full  <= n_full;
            c_empty <= n_empty;
        end
    end

    //next CL
    always_comb begin
        n_waddr = c_waddr;
        n_raddr = c_raddr;
        n_full  = c_full;
        n_empty = c_empty;
        case ({wr, rd})
            2'b01: begin
                if (!c_empty) begin
                    n_raddr = c_raddr + 1;
                    n_full = 0;
                    if (c_waddr == n_raddr) begin 
                    //"이번에 읽기 동작을 해서 읽기 주소를 1 증가시켰더니(n_raddr), 그 값이 현재 쓰기 주소(c_waddr)와 같아졌는가?"
                    //만약 이 조건이 참(true)이라면, 그건 마지막 남은 데이터를 방금 읽었다는 뜻입니다. 따라서 FIFO는 이제 비게 됩니다.
                        n_empty = 1;
                    end
                end
            end //pop
            2'b10: begin
                if (!c_full) begin
                    n_waddr = c_waddr + 1;
                    n_empty = 0;
                    if (n_waddr == c_raddr) begin
                    //"쓰기 주소를 1 증가시켰더니(n_waddr), 그 값이 현재 읽기 주소(c_raddr)와 같아졌는가?"
                    //만약 이 조건이 참(true)이라면, 이는 마지막 남은 빈 공간에 데이터를 채웠다는 것을 의미합니다. 따라서 FIFO는 이제 꽉 차게 됩니다
                        n_full = 1;
                    end
                end
            end //push 
            2'b11: begin
                if (c_full) begin
                    //pop
                    n_raddr = c_raddr + 1;
                    n_full = 0;
                end else if (c_empty) begin
                    //push
                    n_waddr = c_waddr + 1;
                    n_empty = 0;
                end else begin
                    n_raddr = c_raddr + 1;
                    n_waddr = c_waddr + 1;
                end
            end //push, pop
        endcase
    end
endmodule