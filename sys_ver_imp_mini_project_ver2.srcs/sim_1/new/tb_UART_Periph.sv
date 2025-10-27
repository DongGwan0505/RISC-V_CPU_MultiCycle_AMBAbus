`timescale 1ns/1ps

module tb_UART_Periph;

  // --------------------------------
  // 클록/리셋
  // --------------------------------
  logic clk;
  logic rst;

  localparam real CLK_PERIOD_NS = 20.0; // 50 MHz
  initial begin
    clk = 0;
    forever #(CLK_PERIOD_NS/2.0) clk = ~clk;
  end

  // --------------------------------
  // APB 신호
  // --------------------------------
  logic [3:0]  PADDR;    // 로컬 주소(워드 정렬) [3:2]: 0=CR,1=SR,2=TXD,3=RXD
  logic        PWRITE;
  logic        PENABLE;
  logic [31:0] PWDATA;
  logic        PSEL;
  logic [31:0] PRDATA;
  logic        PREADY;

  // --------------------------------
  // UART 핀 (루프백)
  // --------------------------------
  wire tx;
  wire rx;
  assign rx = tx;

  // --------------------------------
  // DUT
  // --------------------------------
  UART_Periph U_DUT (
    .PCLK   (clk),
    .PRESET (rst),
    .PADDR  (PADDR),
    .PWRITE (PWRITE),
    .PENABLE(PENABLE),
    .PWDATA (PWDATA),
    .PSEL   (PSEL),
    .PRDATA (PRDATA),
    .PREADY (PREADY),
    .rx     (rx),
    .tx     (tx)
  );

  // --------------------------------
  // 시뮬 가속용 보오레이트
  // --------------------------------
  defparam U_DUT.U_UART_CORE.U_BAUD_TICK.CLK_HZ = 50_000_000;
  defparam U_DUT.U_UART_CORE.U_BAUD_TICK.BAUD   = 1_000_000;
  defparam U_DUT.U_UART_CORE.U_BAUD_TICK.OS     = 16;

  // --------------------------------
  // APB 접근 task (ACCESS 1클록 보장!)
  // --------------------------------
  task apb_write(input [3:0] addr, input [31:0] data);
    begin
      // SETUP
      @(posedge clk);
      PADDR   <= addr;
      PWRITE  <= 1'b1;
      PWDATA  <= data;
      PSEL    <= 1'b1;
      PENABLE <= 1'b0;

      // ACCESS (정확히 1클록)
      @(posedge clk);
      PENABLE <= 1'b1;

      // 같은 클록의 끝에서 ready가 1(이 슬레이브는 1사이클)이므로
      // 다음 클록에서 바로 TEARDOWN
      @(posedge clk);
      PSEL    <= 1'b0;
      PENABLE <= 1'b0;
      PWRITE  <= 1'b0;
      PADDR   <= '0;
      PWDATA  <= '0;
    end
  endtask

  task apb_read(input [3:0] addr, output [31:0] data);
    begin
      // SETUP
      @(posedge clk);
      PADDR   <= addr;
      PWRITE  <= 1'b0;
      PSEL    <= 1'b1;
      PENABLE <= 1'b0;

      // ACCESS (정확히 1클록)
      @(posedge clk);
      PENABLE <= 1'b1;

      // ACCESS 클록에서의 조합 출력 샘플링
      // (PREADY=1 이므로 PRDATA는 유효)
      // 약간의 델타를 주고 읽고 싶다면 #1step 등을 더해도 됨
      data = PRDATA;

      // TEARDOWN
      @(posedge clk);
      PSEL    <= 1'b0;
      PENABLE <= 1'b0;
      PADDR   <= '0;
    end
  endtask

  // --------------------------------
  // 레지스터 로컬 주소
  // --------------------------------
  localparam [3:0] ADDR_CR  = 4'h0;
  localparam [3:0] ADDR_SR  = 4'h4;
  localparam [3:0] ADDR_TXD = 4'h8;
  localparam [3:0] ADDR_RXD = 4'hC;

  // SR bit helpers
  function bit sr_rx_empty (input [31:0] sr); return sr[0]; endfunction
  function bit sr_tx_full  (input [31:0] sr); return sr[1]; endfunction

  // --------------------------------
  // 보조: RX FIFO purge (empty 될 때까지 pop)
  // --------------------------------
  task purge_rx_fifo();
    reg [31:0] sr, dummy;
    int guard;
    guard = 0;
    forever begin
      apb_read(ADDR_SR, sr);
      if (sr_rx_empty(sr)) break;
      apb_read(ADDR_RXD, dummy); // pop
      guard = guard + 1;
      if (guard > 64) begin
        $display("[TB] purge guard break");
        break;
      end
    end
  endtask

  // --------------------------------
  // 보조: RX empty/non-empty 대기
  // --------------------------------
  task wait_rx_empty(input int max_iter, output bit ok);
    reg [31:0] sr;
    int t;
    ok = 1'b0;
    for (t=0; t<max_iter; t++) begin
      apb_read(ADDR_SR, sr);
      if (sr_rx_empty(sr)) begin ok = 1'b1; return; end
    end
  endtask

  task wait_rx_nonempty(input int max_iter, output bit ok);
    reg [31:0] sr;
    int t;
    ok = 1'b0;
    for (t=0; t<max_iter; t++) begin
      apb_read(ADDR_SR, sr);
      if (!sr_rx_empty(sr)) begin ok = 1'b1; return; end
    end
  endtask

  // --------------------------------
  // 랜덤 10바이트 송수신 테스트 ($random 기반)
  // --------------------------------
  reg [31:0] rddata, sr;
  reg [7:0]  tx_byte, rx_byte;
  integer seed;
  int pass_cnt = 0, fail_cnt = 0;
  int i;

  initial begin
    // 초기화
    PADDR = 0; PWRITE = 0; PENABLE = 0; PWDATA = 0; PSEL = 0;

    rst = 1;
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (5) @(posedge clk);

    // UART enable
    apb_write(ADDR_CR, 32'h0000_0001);

    // 시작 시 RX FIFO 정리
    purge_rx_fifo();

    seed = 32'h1234_ABCD;
    $display("=== UART Loopback Random 10-Byte Test (Vivado 2020 safe) ===");

    for (i=0; i<50; i++) begin
      bit ok;

      // A) 새 전송 전에 RX FIFO empty 보장
      wait_rx_empty(2000, ok);
      if (!ok) begin
        $error("[WARN] (idx=%0d) RX didn't become empty before TX", i);
      end

      // B) TX not-full 대기
      do begin
        apb_read(ADDR_SR, rddata); sr = rddata;
      end while (sr_tx_full(sr));

      // C) 송신
      tx_byte = $random(seed) & 8'hFF;
      $display("[TB] (%0d) TXD <- 0x%02h", i, tx_byte);
      apb_write(ADDR_TXD, {24'h0, tx_byte});

      // D) 이번에 보낸 바이트가 들어올 때까지 대기
      wait_rx_nonempty(60000, ok);
      if (!ok) begin
        $error("[FAIL] (idx=%0d) Timeout waiting RX data", i);
        fail_cnt++;
        continue;
      end

      // E) 수신 pop & 비교
      apb_read(ADDR_RXD, rddata);
      rx_byte = rddata[7:0];
      $display("[TB] (%0d) RXD -> 0x%02h", i, rx_byte);

      if (rx_byte === tx_byte) pass_cnt++; else begin
        $error("[FAIL] (idx=%0d) Mismatch: sent 0x%02h, got 0x%02h",
               i, tx_byte, rx_byte);
        fail_cnt++;
      end

      // F) 다음 케이스 간섭 방지용 정리
      purge_rx_fifo();
    end

    // 요약
    $display("======================================");
    $display(" Random %0d-Byte Loopback Summary", pass_cnt+fail_cnt );
    $display("   PASS: %0d", pass_cnt);
    $display("   FAIL: %0d", fail_cnt);
    $display("======================================");

    if (fail_cnt == 0)
      $display("************** ALL PASS **************");
    else begin
      $error("Some cases FAILED (fail_cnt=%0d)", fail_cnt);
      $fatal;
    end

    #(10*CLK_PERIOD_NS);
    $finish;
  end

endmodule
