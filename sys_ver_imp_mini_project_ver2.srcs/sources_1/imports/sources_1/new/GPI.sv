`timescale 1ns / 1ps

module GPI_periph (
    // global signals
    input  logic        PCLK,
    input  logic        PRESET,
    // APB Interface Signals
    input  logic [ 2:0] PADDR,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic [31:0] PWDATA,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    //Internal Port    
    input  logic [7:0] gpi  //외부 데이터
);
    logic [7:0] cr;  //control register
    logic [7:0] idr; //input data register (외부에서 통과되어 넣어지는)

    APB_SlaveIntf_GPI U_APB_SlaveIntf_GPI(.*);
    GPI U_GPI(.*);

endmodule

module APB_SlaveIntf_GPI (
    // global signals
    input  logic        PCLK,
    input  logic        PRESET,
    // APB Interface Signals
    input  logic [ 2:0] PADDR,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic [31:0] PWDATA,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    //Internal Port
    output logic [7:0] cr, //밖으로 나가는 컨트롤 신호 (control register)
    input  logic [7:0] idr
);
    logic [31:0] slv_reg0, slv_reg1;

    assign cr = slv_reg0[7:0];
    //assign slv_reg1 = {24'b0, idr}; //방법1 : idr 값 들어온 것을 매칭시켜 해당 값을 출력으로 내보내는 방법

    always_ff @(posedge PCLK, posedge PRESET) begin
        if (PRESET) begin
            slv_reg0 <= 0;
            slv_reg1 <= 0;
        end else begin
            PREADY <= 1'b0;
            if (PSEL && PENABLE) begin
                PREADY <= 1'b1;
                if (PWRITE) begin
                    //Save
                    case (PADDR[2])
                        1'd0: slv_reg0 <= PWDATA;
                        1'd1: ; //저장하면 안된다.
                    endcase
                end else begin
                    //Load
                    case (PADDR[2])
                        1'd0: PRDATA <= slv_reg0;
                        1'd1: PRDATA <= {24'b0, idr}; //방법2
                    endcase

                end
            end
        end
    end
endmodule


module GPI (
    input  logic [7:0] cr,  //control register
    output logic [7:0] idr, //input data register (외부에서 통과되어 넣어지는)
    input  logic [7:0] gpi  //외부 데이터
);
    
    genvar i;
    generate
        for (i = 0 ; i < 8; i++) begin
            assign idr[i] = cr[i] ? gpi[i] : 1'bz;
        end
    endgenerate
    
endmodule