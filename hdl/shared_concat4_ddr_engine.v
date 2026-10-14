// shared_concat4_ddr_engine.v
// Runtime-configurable concat of four same-shape CHW int8 tensors along channel dim, with per-bank requant.
// FIX204_COMBO_NONCONV64: add RD64/WR64 fast path.
module shared_concat4_ddr_engine #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire [15:0] ch,
    input  wire [15:0] h,
    input  wire [15:0] w,
    input  wire [ADDR_W-1:0] base0,
    input  wire [ADDR_W-1:0] base1,
    input  wire [ADDR_W-1:0] base2,
    input  wire [ADDR_W-1:0] base3,
    input  wire [ADDR_W-1:0] output_base,
    input  wire [31:0] m0,
    input  wire [31:0] m1,
    input  wire [31:0] m2,
    input  wire [31:0] m3,
    input  wire [4:0]  mult_shift,

    output wire mem_rd_req,
    output wire [ADDR_W-1:0] mem_rd_addr,
    input  wire mem_rd_ready,
    input  wire mem_rd_valid,
    input  wire [DATA_W-1:0] mem_rd_data,

    output wire mem_rd64_req,               // FIX204_COMBO_NONCONV64
    output wire [ADDR_W-1:0] mem_rd64_addr, // FIX204_COMBO_NONCONV64
    input  wire mem_rd64_ready,             // FIX204_COMBO_NONCONV64
    input  wire mem_rd64_valid,             // FIX204_COMBO_NONCONV64
    input  wire [63:0] mem_rd64_data,       // FIX204_COMBO_NONCONV64

    output wire mem_wr_req,
    output wire [ADDR_W-1:0] mem_wr_addr,
    output wire [DATA_W-1:0] mem_wr_data,
    input  wire mem_wr_ready,

    output wire mem_wr64_req,               // FIX204_COMBO_NONCONV64
    output wire [ADDR_W-1:0] mem_wr64_addr, // FIX204_COMBO_NONCONV64
    output wire [63:0] mem_wr64_data,       // FIX204_COMBO_NONCONV64
    input  wire mem_wr64_ready,             // FIX204_COMBO_NONCONV64

    output wire busy,
    output wire done
);
    localparam S_IDLE=4'd0, S_REQ=4'd1, S_WAIT=4'd2, S_WRITE=4'd3, S_DONE=4'd4;
    localparam S_REQ64=4'd5, S_WAIT64=4'd6, S_PACK64=4'd7, S_WRITE64=4'd8;

    reg [3:0] state;
    reg [1:0] bank, bank_hold;
    reg [31:0] off, out_idx;
    reg [DATA_W-1:0] data_hold;
    reg [63:0] data64_hold, pack64_hold;
    reg [2:0] pack64_idx;

    wire [31:0] one_n = {16'd0,ch} * ({16'd0,h} * {16'd0,w});
    wire fast64_en = (one_n[2:0] == 3'd0);

    reg [ADDR_W-1:0] src_base;
    reg [31:0] cur_mult;
    always @(*) begin
        case (bank)
            2'd0: src_base = base0;
            2'd1: src_base = base1;
            2'd2: src_base = base2;
            default: src_base = base3;
        endcase
    end
    always @(*) begin
        case (bank_hold)
            2'd0: cur_mult = m0;
            2'd1: cur_mult = m1;
            2'd2: cur_mult = m2;
            default: cur_mult = m3;
        endcase
    end

    function signed [ACC_W-1:0] requant_mult_round;
        input signed [ACC_W-1:0] din;
        input [31:0] mult;
        input [4:0] shift;
        reg signed [63:0] prod;
        reg signed [63:0] bias;
        begin
            prod = din * $signed({1'b0, mult});
            bias = (64'sd1 <<< (shift-1));
            if (prod >= 0) requant_mult_round = (prod + bias) >>> shift;
            else requant_mult_round = -(((-prod) + bias) >>> shift);
        end
    endfunction
    function [7:0] sat_i8_acc;
        input signed [ACC_W-1:0] din;
        begin
            if (din < -128) sat_i8_acc = 8'h80;
            else if (din > 127) sat_i8_acc = 8'h7f;
            else sat_i8_acc = din[7:0];
        end
    endfunction

    wire signed [ACC_W-1:0] d_s = {{(ACC_W-8){data_hold[7]}}, data_hold};
    wire signed [ACC_W-1:0] d_q = requant_mult_round(d_s, cur_mult, mult_shift);
    wire last = (bank == 2'd3) && (off == one_n-1);

    wire [7:0] pack64_byte =
        (pack64_idx==3'd0) ? data64_hold[ 7: 0] :
        (pack64_idx==3'd1) ? data64_hold[15: 8] :
        (pack64_idx==3'd2) ? data64_hold[23:16] :
        (pack64_idx==3'd3) ? data64_hold[31:24] :
        (pack64_idx==3'd4) ? data64_hold[39:32] :
        (pack64_idx==3'd5) ? data64_hold[47:40] :
        (pack64_idx==3'd6) ? data64_hold[55:48] : data64_hold[63:56];
    wire signed [ACC_W-1:0] pack64_s = {{(ACC_W-8){pack64_byte[7]}}, pack64_byte};
    wire [7:0] pack64_out_byte = sat_i8_acc(requant_mult_round(pack64_s, cur_mult, mult_shift));

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    assign mem_rd_req = (state == S_REQ);
    assign mem_rd_addr = src_base + off[ADDR_W-1:0];
    assign mem_rd64_req = (state == S_REQ64);
    assign mem_rd64_addr = src_base + off[ADDR_W-1:0];

    assign mem_wr_req = (state == S_WRITE);
    assign mem_wr_addr = output_base + out_idx[ADDR_W-1:0];
    assign mem_wr_data = sat_i8_acc(d_q);
    assign mem_wr64_req = (state == S_WRITE64);
    assign mem_wr64_addr = output_base + out_idx[ADDR_W-1:0];
    assign mem_wr64_data = pack64_hold;

    task advance; begin
        out_idx <= out_idx + 1;
        if (off == one_n-1) begin off <= 0; bank <= bank + 1'b1; end
        else off <= off + 1;
    end endtask

    task advance64; begin
        if ((off + 32'd8) >= one_n) begin
            off <= 0;
            bank <= bank + 1'b1;
        end else begin
            off <= off + 32'd8;
        end
        out_idx <= out_idx + 32'd8;
    end endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; bank <= 0; bank_hold <= 0; off <= 0; out_idx <= 0; data_hold <= 0;
            data64_hold <= 0; pack64_hold <= 0; pack64_idx <= 0;
        end else begin
            case (state)
                S_IDLE: if (start) begin
                    bank <= 0; bank_hold <= 0; off <= 0; out_idx <= 0; pack64_idx <= 0;
                    state <= (one_n == 0) ? S_DONE : (fast64_en ? S_REQ64 : S_REQ);
                end

                S_REQ: if (mem_rd_ready) begin bank_hold <= bank; state <= S_WAIT; end
                S_WAIT: if (mem_rd_valid) begin data_hold <= mem_rd_data; state <= S_WRITE; end
                S_WRITE: if (mem_wr_ready) begin if (last) state <= S_DONE; else begin advance(); state <= S_REQ; end end

                S_REQ64: if (mem_rd64_ready) begin bank_hold <= bank; state <= S_WAIT64; end
                S_WAIT64: if (mem_rd64_valid) begin data64_hold <= mem_rd64_data; pack64_hold <= 64'd0; pack64_idx <= 0; state <= S_PACK64; end
                S_PACK64: begin
                    case (pack64_idx)
                        3'd0: pack64_hold[ 7: 0] <= pack64_out_byte;
                        3'd1: pack64_hold[15: 8] <= pack64_out_byte;
                        3'd2: pack64_hold[23:16] <= pack64_out_byte;
                        3'd3: pack64_hold[31:24] <= pack64_out_byte;
                        3'd4: pack64_hold[39:32] <= pack64_out_byte;
                        3'd5: pack64_hold[47:40] <= pack64_out_byte;
                        3'd6: pack64_hold[55:48] <= pack64_out_byte;
                        3'd7: pack64_hold[63:56] <= pack64_out_byte;
                    endcase
                    if (pack64_idx == 3'd7) begin pack64_idx <= 0; state <= S_WRITE64; end
                    else begin pack64_idx <= pack64_idx + 1'b1; end
                end
                S_WRITE64: if (mem_wr64_ready) begin
                    if ((bank == 2'd3) && ((off + 32'd8) >= one_n)) state <= S_DONE;
                    else begin advance64(); state <= S_REQ64; end
                end

                S_DONE: state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
