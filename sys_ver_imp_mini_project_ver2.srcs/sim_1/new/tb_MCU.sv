`timescale 1ns / 1ps

module tb_MCU_UART;
  // 클록/리셋
  logic clk;
  logic reset;

  // 외부 포트
  logic [7:0] gpo;
  logic [7:0] gpi = 8'h00;
  tri   [7:0] gpio;
  logic rx;
  logic tx;

  // DUT
  MCU dut (
    .clk(clk),
    .reset(reset),
    .gpo(gpo),
    .gpi(gpi),
    .gpio(gpio),
    .rx(rx),
    .tx(tx)
  );

  // 100 MHz 클록 생성
  always #5 clk = ~clk;

  // 초기 조건
  initial begin
    clk   = 0;
    reset = 1;
    rx    = 1; // idle 상태 (UART 라인은 high)
    #100;
    reset = 0;
  end

  // =============================
  // UART 송신 시뮬레이터 (PC 역할)
  // =============================
  task uart_send_byte(input byte data);
    integer i;
    // Start bit
    rx = 0;
    #(104166); // 1/9600 = 104.166us per bit → 100 MHz = 104166 ns
    // Data bits (LSB first)
    for (i = 0; i < 8; i++) begin
      rx = data[i];
      #(104166);
    end
    // Stop bit
    rx = 1;
    #(104166);
  endtask

  // 시뮬 시나리오
  initial begin
    @(negedge reset);
    #200000; // 리셋 안정화
    uart_send_byte(8'h41); // 'A'
    #2_000_000;
    uart_send_byte(8'h42); // 'B'
    #5_000_000;
    $finish;
  end

endmodule
