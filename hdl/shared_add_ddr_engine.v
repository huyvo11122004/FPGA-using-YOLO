// shared_add_ddr_engine.v
// Runtime-configurable elementwise signed-int8 add with per-input requant.
// FIX204_COMBO_NONCONV64: add RD64/RD64/WR64 fast path.
module shared_add_ddr_engine #(
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
    input  wire [ADDR_W-1:0] input_a_base,
    input  wire [ADDR_W-1:0] input_b_base,
    input  wire [ADDR_W-1:0] output_base,
    input  wire [31:0] a_mult,
    input  wire [31:0] b_mult,
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
    localparam S_IDLE=4'd0, S_RA=4'd1, S_WA=4'd2, S_RB=4'd3, S_WB=4'd4, S_WR=4'd5, S_DONE=4'd6;
    localparam S_RA64=4'd7, S_WA64=4'd8, S_RB64=4'd9, S_WB64=4'd10, S_PACK64=4'd11, S_WR64=4'd12;

    reg [3:0] state;
    reg [31:0] idx;
    reg [7:0] a_hold, b_hold;
    reg [63:0] a64_hold, b64_hold, pack64_hold;
    reg [2:0] pack64_idx;

    wire [31:0] n = {16'd0,ch} * ({16'd0,h} * {16'd0,w});
    wire fast64_en = (n[2:0] == 3'd0);

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

    wire signed [ACC_W-1:0] a_s = {{(ACC_W-8){a_hold[7]}}, a_hold};
    wire signed [ACC_W-1:0] b_s = {{(ACC_W-8){b_hold[7]}}, b_hold};
    wire signed [ACC_W-1:0] a_q = requant_mult_round(a_s, a_mult, mult_shift);
    wire signed [ACC_W-1:0] b_q = requant_mult_round(b_s, b_mult, mult_shift);
    wire signed [ACC_W-1:0] sum_q = a_q + b_q;

    wire [7:0] a64_byte =
        (pack64_idx==3'd0) ? a64_hold[ 7: 0] :
        (pack64_idx==3'd1) ? a64_hold[15: 8] :
        (pack64_idx==3'd2) ? a64_hold[23:16] :
        (pack64_idx==3'd3) ? a64_hold[31:24] :
        (pack64_idx==3'd4) ? a64_hold[39:32] :
        (pack64_idx==3'd5) ? a64_hold[47:40] :
        (pack64_idx==3'd6) ? a64_hold[55:48] : a64_hold[63:56];
    wire [7:0] b64_byte =
        (pack64_idx==3'd0) ? b64_hold[ 7: 0] :
        (pack64_idx==3'd1) ? b64_hold[15: 8] :
        (pack64_idx==3'd2) ? b64_hold[23:16] :
        (pack64_idx==3'd3) ? b64_hold[31:24] :
        (pack64_idx==3'd4) ? b64_hold[39:32] :
        (pack64_idx==3'd5) ? b64_hold[47:40] :
        (pack64_idx==3'd6) ? b64_hold[55:48] : b64_hold[63:56];

    wire signed [ACC_W-1:0] a64_s = {{(ACC_W-8){a64_byte[7]}}, a64_byte};
    wire signed [ACC_W-1:0] b64_s = {{(ACC_W-8){b64_byte[7]}}, b64_byte};
    wire [7:0] pack64_out_byte = sat_i8_acc(requant_mult_round(a64_s, a_mult, mult_shift) +
                                            requant_mult_round(b64_s, b_mult, mult_shift));

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    assign mem_rd_req = (state == S_RA) || (state == S_RB);
    assign mem_rd_addr = (state == S_RA) ? (input_a_base + idx[ADDR_W-1:0]) : (input_b_base + idx[ADDR_W-1:0]);
    assign mem_rd64_req = (state == S_RA64) || (state == S_RB64);
    assign mem_rd64_addr = (state == S_RA64) ? (input_a_base + idx[ADDR_W-1:0]) : (input_b_base + idx[ADDR_W-1:0]);

    assign mem_wr_req = (state == S_WR);
    assign mem_wr_addr = output_base + idx[ADDR_W-1:0];
    assign mem_wr_data = sat_i8_acc(sum_q);
    assign mem_wr64_req = (state == S_WR64);
    assign mem_wr64_addr = output_base + idx[ADDR_W-1:0];
    assign mem_wr64_data = pack64_hold;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; idx <= 0; a_hold <= 0; b_hold <= 0;
            a64_hold <= 0; b64_hold <= 0; pack64_hold <= 0; pack64_idx <= 0;
        end else begin
            case (state)
                S_IDLE: if (start) begin
                    idx <= 0; pack64_idx <= 0; pack64_hold <= 0;
                    state <= (n == 0) ? S_DONE : (fast64_en ? S_RA64 : S_RA);
                end

                S_RA: if (mem_rd_ready) state <= S_WA;
                S_WA: if (mem_rd_valid) begin a_hold <= mem_rd_data; state <= S_RB; end
                S_RB: if (mem_rd_ready) state <= S_WB;
                S_WB: if (mem_rd_valid) begin b_hold <= mem_rd_data; state <= S_WR; end
                S_WR: if (mem_wr_ready) begin if (idx == n-1) state <= S_DONE; else begin idx <= idx + 1; state <= S_RA; end end

                S_RA64: if (mem_rd64_ready) state <= S_WA64;
                S_WA64: if (mem_rd64_valid) begin a64_hold <= mem_rd64_data; state <= S_RB64; end
                S_RB64: if (mem_rd64_ready) state <= S_WB64;
                S_WB64: if (mem_rd64_valid) begin b64_hold <= mem_rd64_data; pack64_hold <= 64'd0; pack64_idx <= 0; state <= S_PACK64; end
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
                    if (pack64_idx == 3'd7) begin pack64_idx <= 0; state <= S_WR64; end
                    else pack64_idx <= pack64_idx + 1'b1;
                end
                S_WR64: if (mem_wr64_ready) begin
                    if ((idx + 32'd8) >= n) state <= S_DONE;
                    else begin idx <= idx + 32'd8; state <= S_RA64; end
                end

                S_DONE: state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
