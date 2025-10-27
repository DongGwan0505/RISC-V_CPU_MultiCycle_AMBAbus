`timescale 1ns / 1ps

module MCU (
    input  logic       clk,
    input  logic       reset,
    // External Port
    output logic [7:0] gpo,
    input  logic [7:0] gpi,
    inout  logic [7:0] gpio,
    input  logic       rx,
    output logic       tx
);

    // --------------------------------
    // 새로 추가한 내부 신호
    // --------------------------------
    logic [7:0] gpo_bus;            // GPO_Periph가 구동하는 내부 버스
    logic [7:0] uart_tx_byte_dbg;   // UART가 실제로 전송 시작한 바이트

    // 기존 배선
    wire  PCLK   = clk;
    wire  PRESET = reset;

    // Internal Interface Signals
    logic        transfer;
    logic        ready;
    logic        write;
    logic [31:0] addr;
    logic [31:0] wdata;
    logic [31:0] rdata;

    logic [31:0] instrCode;
    logic [31:0] instrMemAddr;
    logic        busWe;
    logic [31:0] busAddr;
    logic [31:0] busWData;
    logic [31:0] busRData;

    // APB Interface Signals
    logic [31:0] PADDR;
    logic        PWRITE;
    logic        PENABLE;
    logic [31:0] PWDATA;

    logic        PSEL_RAM;
    logic        PSEL_GPO;
    logic        PSEL_GPI;
    logic        PSEL_GPIO;
    logic        PSEL_UART;

    logic [31:0] PRDATA_RAM;
    logic [31:0] PRDATA_GPO;
    logic [31:0] PRDATA_GPI;
    logic [31:0] PRDATA_GPIO;
    logic [31:0] PRDATA_UART;

    logic        PREADY_RAM;
    logic        PREADY_GPO;
    logic        PREADY_GPI;
    logic        PREADY_GPIO;
    logic        PREADY_UART;

    assign write    = busWe;
    assign addr     = busAddr;
    assign wdata    = busWData;
    assign busRData = rdata;

    // --------------------------------
    // Submodules
    // --------------------------------
    ROM U_ROM (
        .addr(instrMemAddr),
        .data(instrCode)
    );

    CPU_RV32I U_RV32I (.*);

    APB_Master U_APB_Master (
        .*,
        .PSEL0  (PSEL_RAM),
        .PSEL1  (PSEL_GPO),
        .PSEL2  (PSEL_GPI),
        .PSEL3  (PSEL_GPIO),
        .PSEL4  (PSEL_UART),
        .PRDATA0(PRDATA_RAM),
        .PRDATA1(PRDATA_GPO),
        .PRDATA2(PRDATA_GPI),
        .PRDATA3(PRDATA_GPIO),
        .PRDATA4(PRDATA_UART),
        .PREADY0(PREADY_RAM),
        .PREADY1(PREADY_GPO),
        .PREADY2(PREADY_GPI),
        .PREADY3(PREADY_GPIO),
        .PREADY4(PREADY_UART)
    );

    RAM U_RAM (
        .*,
        .PSEL  (PSEL_RAM),
        .PRDATA(PRDATA_RAM),
        .PREADY(PREADY_RAM)
    );

    // ★ GPO_Periph는 외부 핀을 직접 구동하지 않고 내부 버스에만 출력
    GPO_Periph U_GPO_Periph (
        .*,
        .PSEL  (PSEL_GPO),
        .PRDATA(PRDATA_GPO),
        .PREADY(PREADY_GPO),
        .gpo   (gpo_bus)   // << 변경: 원래 gpo 대신 gpo_bus로 받음
    );

    GPI_periph U_GPI_periph(
        .*,
        .PSEL  (PSEL_GPI),
        .PRDATA(PRDATA_GPI),
        .PREADY(PREADY_GPI)
    );

    GPIO_Periph U_GPIO_Periph(
        .*,
        .PSEL   (PSEL_GPIO),
        .PRDATA (PRDATA_GPIO),
        .PREADY (PREADY_GPIO)
    );

    // ★ UART_Periph에서 디버그 바이트를 받아옴
    UART_Periph U_UART_Periph(
        .*,
        .PSEL       (PSEL_UART),
        .PRDATA     (PRDATA_UART),
        .PREADY     (PREADY_UART),
        .rx         (rx),
        .tx         (tx),
        .tx_byte_dbg(uart_tx_byte_dbg)  // << 추가 포트
    );

    // --------------------------------
    // 최종 외부 핀 gpo 매핑
    //   - LED가 Active-Low면 ~uart_tx_byte_dbg 사용
    // --------------------------------
    assign gpo = uart_tx_byte_dbg;

endmodule

/*
`timescale 1ns / 1ps

module MCU (
    input  logic       clk,
    input  logic       reset,
    // External Port
    output logic [7:0] gpo,
    input  logic [7:0] gpi,  //외부 데이터
    inout  logic [7:0] gpio,
    input  logic       rx,
    output logic       tx
);

    wire         PCLK = clk;
    wire         PRESET = reset;
    // Internal Interface Signals
    logic        transfer;
    logic        ready;
    logic        write;
    logic [31:0] addr;
    logic [31:0] wdata;
    logic [31:0] rdata;

    logic [31:0] instrCode;
    logic [31:0] instrMemAddr;
    logic        busWe;
    logic [31:0] busAddr;
    logic [31:0] busWData;
    logic [31:0] busRData;
    // APB Interface Signals
    logic [31:0] PADDR;
    logic        PWRITE;
    logic        PENABLE;
    logic [31:0] PWDATA;

    logic        PSEL_RAM;
    logic        PSEL_GPO;
    logic        PSEL_GPI;
    logic        PSEL_GPIO;
    logic        PSEL_UART;

    logic [31:0] PRDATA_RAM;
    logic [31:0] PRDATA_GPO;
    logic [31:0] PRDATA_GPI;
    logic [31:0] PRDATA_GPIO;
    logic [31:0] PRDATA_UART;

    logic        PREADY_RAM;
    logic        PREADY_GPO;
    logic        PREADY_GPI;
    logic        PREADY_GPIO;
    logic        PREADY_UART;

    assign write = busWe;
    assign addr = busAddr;
    assign wdata = busWData;
    assign busRData = rdata;
    // 눈으로 보기용: TX/RX 라인을 LED로 미러
    //assign tx = rx;
    //wire dbg_tx_wr   = U_UART_Periph.U_APB_IF.tx_wr_pulse;  // TXD 레지스터 쓰기 1클록 펄스
    //wire dbg_psel_u  = PSEL_UART;                           // UART PSEL
    //wire dbg_pen     = PENABLE;                             // APB PENABLE
    //wire dbg_pwrite  = PWRITE;                              // APB PWRITE
    //assign gpo[7] = dbg_psel_u;   // UART 선택되면 LED7 켬
    //assign gpo[6] = dbg_tx_wr;    // TXD 쓰기 순간 LED6 토글
    //assign gpo[5] = dbg_pwrite;   // APB write일 때 LED5
    //assign gpo[4] = dbg_pen;      // APB enable일 때 LED4
    //assign gpo[0] = tx;  // LED0 (U16)
    //assign gpo[1] = rx;  // LED1 (E19)

    ROM U_ROM (
        .addr(instrMemAddr),
        .data(instrCode)
    );

    CPU_RV32I U_RV32I (.*);

    APB_Master U_APB_Master (
        .*,
        .PSEL0  (PSEL_RAM),
        .PSEL1  (PSEL_GPO),
        .PSEL2  (PSEL_GPI),
        .PSEL3  (PSEL_GPIO),
        .PSEL4  (PSEL_UART),
        .PRDATA0(PRDATA_RAM),
        .PRDATA1(PRDATA_GPO),
        .PRDATA2(PRDATA_GPI),
        .PRDATA3(PRDATA_GPIO),
        .PRDATA4(PRDATA_UART),
        .PREADY0(PREADY_RAM),
        .PREADY1(PREADY_GPO),
        .PREADY2(PREADY_GPI),
        .PREADY3(PREADY_GPIO),
        .PREADY4(PREADY_UART)
    );

    RAM U_RAM (
        .*,
        .PSEL  (PSEL_RAM),
        .PRDATA(PRDATA_RAM),
        .PREADY(PREADY_RAM)
    );

    GPO_Periph U_GPO_Periph (
        .*,
        .PSEL  (PSEL_GPO),
        .PRDATA(PRDATA_GPO),
        .PREADY(PREADY_GPO),
        .gpo   (gpo)
    );

    GPI_periph U_GPI_periph(
        .*,
        .PSEL  (PSEL_GPI),
        .PRDATA(PRDATA_GPI),
        .PREADY(PREADY_GPI)
    );

    GPIO_Periph U_GPIO_Periph(
        .*,
        .PSEL   (PSEL_GPIO),
        .PRDATA (PRDATA_GPIO),
        .PREADY (PREADY_GPIO)
    );

    UART_Periph U_UART_Periph(
        .*,
        .PSEL  (PSEL_UART),
        .PRDATA(PRDATA_UART),
        .PREADY(PREADY_UART),
        // external pins
        .rx(rx),
        .tx(tx)
    );

endmodule
*/