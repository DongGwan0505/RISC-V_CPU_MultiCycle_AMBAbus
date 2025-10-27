`timescale 1ns / 1ps
module APB_Master (
    input  logic        PCLK, PRESET,
    output logic [31:0] PADDR, PWDATA,
    output logic        PWRITE, PENABLE,
    output logic        PSEL0, PSEL1, PSEL2, PSEL3, PSEL4,
    input  logic [31:0] PRDATA0, PRDATA1, PRDATA2, PRDATA3, PRDATA4,
    input  logic        PREADY0, PREADY1, PREADY2, PREADY3, PREADY4,
    input  logic        transfer, write,
    output logic        ready,
    input  logic [31:0] addr, wdata,
    output logic [31:0] rdata
);
    logic [4:0]  pselx;
    logic [2:0]  mux_sel;
    logic        decoder_en;
    logic [31:0] a_reg, a_nxt, w_reg, w_nxt;
    logic        we_reg, we_nxt;

    // ★ 워드주소 -> 바이트주소 변환
    logic [31:0] addr_byte;
    assign addr_byte = {a_reg[31:2], 2'b00};

    assign PSEL0 = pselx[0];
    assign PSEL1 = pselx[1];
    assign PSEL2 = pselx[2];
    assign PSEL3 = pselx[3];
    assign PSEL4 = pselx[4];

    typedef enum logic[1:0]{IDLE,SETUP,ACCESS} st_t;
    st_t st, st_n;

    // seq
    always_ff @(posedge PCLK or posedge PRESET) begin
        if (PRESET) begin
            st     <= IDLE;
            a_reg  <= '0;
            w_reg  <= '0;
            we_reg <= 1'b0;
        end else begin
            st     <= st_n;
            a_reg  <= a_nxt;
            w_reg  <= w_nxt;
            we_reg <= we_nxt;
        end
    end

    // comb
    always_comb begin
        st_n       = st;
        decoder_en = 1'b0;
        PENABLE    = 1'b0;

        a_nxt  = a_reg;
        w_nxt  = w_reg;
        we_nxt = we_reg;

        // ★ 바이트 주소로 APB 출력
        PADDR  = addr_byte;
        PWRITE = we_reg;
        PWDATA = w_reg;

        unique case (st)
            IDLE: begin
                if (transfer) begin
                    st_n   = SETUP;
                    a_nxt  = addr;     // CPU가 준 "워드" 주소 latch
                    w_nxt  = wdata;
                    we_nxt = write;
                end
            end

            SETUP: begin
                decoder_en = 1'b1;     // SETUP에도 select assert
                st_n       = ACCESS;
            end

            ACCESS: begin
                decoder_en = 1'b1;
                PENABLE    = 1'b1;
                if (ready) st_n = IDLE;
            end
        endcase
    end

    // ★ 디코더에도 바이트주소 전달
    APB_Decoder U_DEC (
        .en     (decoder_en),
        .sel    (addr_byte),
        .y      (pselx),
        .mux_sel(mux_sel)
    );

    APB_Mux U_MUX (
        .sel    (mux_sel),
        .rdata0 (PRDATA0), .rdata1(PRDATA1), .rdata2(PRDATA2), .rdata3(PRDATA3), .rdata4(PRDATA4),
        .ready0 (PREADY0), .ready1(PREADY1), .ready2(PREADY2), .ready3(PREADY3), .ready4(PREADY4),
        .rdata  (rdata),
        .ready  (ready)
    );
endmodule

module APB_Decoder (
    input  logic        en,
    input  logic [31:0] sel,      // 바이트 주소 (PADDR와 동일)
    output logic [4:0]  y,
    output logic [2:0]  mux_sel
);
    always_comb begin
        y       = 5'b00000;
        mux_sel = 3'd0;
        if (en) begin
            // 상위 12비트(31:20)는 항상 0x1000_xxxx 영역
            if (sel[31:20] == 12'h100) begin
                unique case (sel[15:12])
                    4'h0: begin y=5'b00001; mux_sel=3'd0; end // RAM
                    4'h1: begin y=5'b00010; mux_sel=3'd1; end // GPO
                    4'h2: begin y=5'b00100; mux_sel=3'd2; end // GPI
                    4'h3: begin y=5'b01000; mux_sel=3'd3; end // GPIO
                    4'h4: begin y=5'b10000; mux_sel=3'd4; end // UART
                    default: ;
                endcase
            end
        end
    end
endmodule


/*
module APB_Decoder (
    input  logic        en,
    input  logic [31:0] sel,      // 바이트 주소
    output logic [ 4:0] y,
    output logic [ 2:0] mux_sel
);
    always_comb begin
        y = 5'b00000;
        if (en) begin
            casex (sel)
                32'h1000_0xxx: y = 5'b00001; // RAM
                32'h1000_1xxx: y = 5'b00010; // GPO
                32'h1000_2xxx: y = 5'b00100; // GPI
                32'h1000_3xxx: y = 5'b01000; // GPIO
                32'h1000_4xxx: y = 5'b10000; // UART
                default:       y = 5'b00000;
            endcase
        end
    end

    always_comb begin
        mux_sel = 3'd0;           // 기본값 0 (X 전파 방지)
        if (en) begin
            casex (sel)
                32'h1000_0xxx: mux_sel = 3'd0;
                32'h1000_1xxx: mux_sel = 3'd1;
                32'h1000_2xxx: mux_sel = 3'd2;
                32'h1000_3xxx: mux_sel = 3'd3;
                32'h1000_4xxx: mux_sel = 3'd4;
                default:       mux_sel = 3'd0;
            endcase
        end
    end
endmodule
*/

module APB_Mux (
    input  logic [ 2:0] sel,
    input  logic [31:0] rdata0,
    input  logic [31:0] rdata1,
    input  logic [31:0] rdata2,
    input  logic [31:0] rdata3,
    input  logic [31:0] rdata4,
    input  logic        ready0,
    input  logic        ready1,
    input  logic        ready2,
    input  logic        ready3,
    input  logic        ready4,
    output logic [31:0] rdata,
    output logic        ready
);

    always_comb begin
        rdata = 32'b0;
        case (sel)
            3'd0: rdata = rdata0;
            3'd1: rdata = rdata1;
            3'd2: rdata = rdata2;
            3'd3: rdata = rdata3;
            3'd4: rdata = rdata4;
        endcase
    end

    always_comb begin
        ready = 1'b0;
        case (sel)
            3'd0: ready = ready0;
            3'd1: ready = ready1;
            3'd2: ready = ready2;
            3'd3: ready = ready3;
            3'd4: ready = ready4;
        endcase
    end
endmodule

/*
`timescale 1ns / 1ps

module APB_Master (
    // global signals
    input  logic        PCLK,
    input  logic        PRESET,
    // APB Interface Signals
    output logic [31:0] PADDR,
    output logic        PWRITE,
    output logic        PENABLE,
    output logic [31:0] PWDATA,
    output logic        PSEL0,
    output logic        PSEL1,
    output logic        PSEL2,
    output logic        PSEL3,
    output logic        PSEL4,
    input  logic [31:0] PRDATA0,
    input  logic [31:0] PRDATA1,
    input  logic [31:0] PRDATA2,
    input  logic [31:0] PRDATA3,
    input  logic [31:0] PRDATA4,
    input  logic        PREADY0,
    input  logic        PREADY1,
    input  logic        PREADY2,
    input  logic        PREADY3,
    input  logic        PREADY4,
    // Internal Interface Signals
    input  logic        transfer,
    output logic        ready,
    input  logic        write,
    input  logic [31:0] addr,
    input  logic [31:0] wdata,
    output logic [31:0] rdata
);
    logic [4:0] pselx;
    logic [2:0] mux_sel;
    logic decoder_en;
    logic [31:0] temp_addr_reg, temp_addr_next, temp_wdata_reg, temp_wdata_next;
    logic temp_write_reg, temp_write_next;

    assign PSEL0 = pselx[0];
    assign PSEL1 = pselx[1];
    assign PSEL2 = pselx[2];
    assign PSEL3 = pselx[3];
    assign PSEL4 = pselx[4];

    typedef enum {
        IDLE,
        SETUP,
        ACCESS
    } apb_state_e;

    apb_state_e state, state_next;

    always_ff @(posedge PCLK, posedge PRESET) begin
        if (PRESET) begin
            state          <= IDLE;
            temp_addr_reg  <= 0;
            temp_wdata_reg <= 0;
            temp_write_reg <= 0;
        end else begin
            state          <= state_next;
            temp_addr_reg  <= temp_addr_next;
            temp_wdata_reg <= temp_wdata_next;
            temp_write_reg <= temp_write_next;
        end
    end

    always_comb begin
        state_next      = state;
        decoder_en      = 1'b0;
        PENABLE         = 1'b0;
        temp_addr_next  = temp_addr_reg;
        temp_wdata_next = temp_wdata_reg;
        temp_write_next = temp_write_reg;
        PADDR           = temp_addr_reg;
        PWRITE          = temp_write_reg;
        PWDATA          = temp_wdata_reg;
        case (state)
            IDLE: begin
                decoder_en = 1'b0;
                if (transfer) begin
                    state_next = SETUP;
                    temp_addr_next = addr; // latching
                    temp_wdata_next = wdata;
                    temp_write_next = write;
                end
            end
            SETUP: begin
                decoder_en = 1'b1;
                PENABLE    = 1'b0;
                PADDR      = temp_addr_reg;
                PWRITE     = temp_write_reg;
                state_next = ACCESS;
                if (temp_write_reg) begin
                    PWDATA = temp_wdata_reg;
                end
            end
            ACCESS: begin
                decoder_en = 1'b1;
                PENABLE    = 1'b1;
                if (ready) begin
                    state_next = IDLE;
                end
            end
        endcase
    end

    
    APB_Decoder U_APB_Decoder (
        .en     (decoder_en),
        .sel    (temp_addr_reg),
        .y      (pselx),
        .mux_sel(mux_sel)
    );

    APB_Mux U_APB_Mux (
        .sel   (mux_sel),
        .rdata0(PRDATA0),
        .rdata1(PRDATA1),
        .rdata2(PRDATA2),
        .rdata3(PRDATA3),
        .rdata4(PRDATA4),
        .ready0(PREADY0),
        .ready1(PREADY1),
        .ready2(PREADY2),
        .ready3(PREADY3),
        .ready4(PREADY4),
        .rdata (rdata),
        .ready (ready)
    );

endmodule

module APB_Decoder (
    input  logic        en,
    input  logic [31:0] sel,
    output logic [ 4:0] y,
    output logic [ 2:0] mux_sel
);
    always_comb begin
        y = 5'b0000;
        if (en) begin
            casex (sel)
                32'h1000_0xxx: y = 5'b00001;
                32'h1000_1xxx: y = 5'b00010;
                32'h1000_2xxx: y = 5'b00100;
                32'h1000_3xxx: y = 5'b01000;
                32'h1000_4xxx: y = 5'b10000;
            endcase
        end
    end

    always_comb begin
        mux_sel = 3'dx;
        if (en) begin
            casex (sel)
                32'h1000_0xxx: mux_sel = 3'd0;
                32'h1000_1xxx: mux_sel = 3'd1;
                32'h1000_2xxx: mux_sel = 3'd2;
                32'h1000_3xxx: mux_sel = 3'd3;
                32'h1000_4xxx: mux_sel = 3'd4;
            endcase
        end
    end
endmodule

module APB_Mux (
    input  logic [ 2:0] sel,
    input  logic [31:0] rdata0,
    input  logic [31:0] rdata1,
    input  logic [31:0] rdata2,
    input  logic [31:0] rdata3,
    input  logic [31:0] rdata4,
    input  logic        ready0,
    input  logic        ready1,
    input  logic        ready2,
    input  logic        ready3,
    input  logic        ready4,
    output logic [31:0] rdata,
    output logic        ready
);

    always_comb begin
        rdata = 32'b0;
        case (sel)
            3'd0: rdata = rdata0;
            3'd1: rdata = rdata1;
            3'd2: rdata = rdata2;
            3'd3: rdata = rdata3;
            3'd4: rdata = rdata4;
        endcase
    end

    always_comb begin
        ready = 1'b0;
        case (sel)
            3'd0: ready = ready0;
            3'd1: ready = ready1;
            3'd2: ready = ready2;
            3'd3: ready = ready3;
            3'd4: ready = ready4;
        endcase
    end
endmodule
*/