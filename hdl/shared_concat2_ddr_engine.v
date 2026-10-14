// shared_concat2_ddr_engine.v
// Runtime-configurable concat of two CHW int8 tensors along channel dim, with per-branch requant to output scale.
// FIX202_CAT2_RD64_WR64: 8-byte RD64/WR64 fast path plus old byte fallback.
module shared_concat2_ddr_engine #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire [15:0] a_ch,
    input  wire [15:0] b_ch,
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

    output wire mem_rd64_req,               // FIX202_CAT2_RD64_WR64
    output wire [ADDR_W-1:0] mem_rd64_addr, // FIX202_CAT2_RD64_WR64
    input  wire mem_rd64_ready,             // FIX202_CAT2_RD64_WR64
    input  wire mem_rd64_valid,             // FIX202_CAT2_RD64_WR64
    input  wire [63:0] mem_rd64_data,       // FIX202_CAT2_RD64_WR64

    output wire mem_wr_req,
    output wire [ADDR_W-1:0] mem_wr_addr,
    output wire [DATA_W-1:0] mem_wr_data,
    input  wire mem_wr_ready,

    output wire mem_wr64_req,               // FIX202_CAT2_RD64_WR64
    output wire [ADDR_W-1:0] mem_wr64_addr, // FIX202_CAT2_RD64_WR64
    output wire [63:0] mem_wr64_data,       // FIX202_CAT2_RD64_WR64
    input  wire mem_wr64_ready,             // FIX202_CAT2_RD64_WR64

    output wire busy,
    output wire done
);
    localparam S_IDLE=4'd0, S_REQ=4'd1, S_WAIT=4'd2, S_WRITE=4'd3, S_DONE=4'd4;
    localparam S_REQ64=4'd5, S_WAIT64=4'd6, S_PACK64=4'd7, S_WRITE64=4'd8; // FIX202_CAT2_RD64_WR64

    reg [3:0] state;
    reg [31:0] idx, idx_hold;
    reg [DATA_W-1:0] data_hold;
    reg use_a_hold;
    reg [63:0] data64_hold, pack64_hold;
    reg [2:0] pack64_idx;

    wire [31:0] spatial = {16'd0,h} * {16'd0,w};
    wire [31:0] a_n = {16'd0,a_ch} * spatial;
    wire [31:0] b_n = {16'd0,b_ch} * spatial;
    wire [31:0] out_n = a_n + b_n;
    wire use_a = (idx < a_n);
    wire [31:0] src_off = use_a ? idx : (idx - a_n);
    wire fast64_en = (out_n[2:0] == 3'd0) && (a_n[2:0] == 3'd0) && (b_n[2:0] == 3'd0); // FIX202_CAT2_RD64_WR64

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
            else           requant_mult_round = -(((-prod) + bias) >>> shift);
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
    wire signed [ACC_W-1:0] d_q = requant_mult_round(d_s, use_a_hold ? a_mult : b_mult, mult_shift);

    wire [7:0] pack64_byte =
        (pack64_idx==3'd0) ? data64_hold[ 7: 0] :
        (pack64_idx==3'd1) ? data64_hold[15: 8] :
        (pack64_idx==3'd2) ? data64_hold[23:16] :
        (pack64_idx==3'd3) ? data64_hold[31:24] :
        (pack64_idx==3'd4) ? data64_hold[39:32] :
        (pack64_idx==3'd5) ? data64_hold[47:40] :
        (pack64_idx==3'd6) ? data64_hold[55:48] : data64_hold[63:56];

    wire signed [ACC_W-1:0] pack64_s = {{(ACC_W-8){pack64_byte[7]}}, pack64_byte};
    wire signed [ACC_W-1:0] pack64_q = requant_mult_round(pack64_s, use_a_hold ? a_mult : b_mult, mult_shift);
    wire [7:0] pack64_out_byte = sat_i8_acc(pack64_q);

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    assign mem_rd_req  = (state == S_REQ);
    assign mem_rd_addr = (use_a ? input_a_base : input_b_base) + src_off[ADDR_W-1:0];

    assign mem_rd64_req  = (state == S_REQ64); // FIX202_CAT2_RD64_WR64
    assign mem_rd64_addr = (use_a ? input_a_base : input_b_base) + src_off[ADDR_W-1:0]; // FIX202_CAT2_RD64_WR64

    assign mem_wr_req  = (state == S_WRITE);
    assign mem_wr_addr = output_base + idx[ADDR_W-1:0];
    assign mem_wr_data = sat_i8_acc(d_q);

    assign mem_wr64_req  = (state == S_WRITE64); // FIX202_CAT2_RD64_WR64
    assign mem_wr64_addr = output_base + idx_hold[ADDR_W-1:0]; // FIX202_CAT2_RD64_WR64
    assign mem_wr64_data = pack64_hold; // FIX202_CAT2_RD64_WR64

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            idx <= 32'd0; idx_hold <= 32'd0;
            data_hold <= {DATA_W{1'b0}};
            use_a_hold <= 1'b0;
            data64_hold <= 64'd0; pack64_hold <= 64'd0; pack64_idx <= 3'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        idx <= 32'd0; idx_hold <= 32'd0;
                        data64_hold <= 64'd0; pack64_hold <= 64'd0; pack64_idx <= 3'd0;
                        use_a_hold <= 1'b0;
                        if (out_n == 0) state <= S_DONE;
                        else state <= fast64_en ? S_REQ64 : S_REQ;
                    end
                end

                S_REQ: if (mem_rd_ready) begin use_a_hold <= use_a; state <= S_WAIT; end
                S_WAIT: if (mem_rd_valid) begin data_hold <= mem_rd_data; state <= S_WRITE; end
                S_WRITE: if (mem_wr_ready) begin
                    if (idx == out_n-1) state <= S_DONE;
                    else begin idx <= idx + 32'd1; state <= S_REQ; end
                end

                S_REQ64: if (mem_rd64_ready) begin
                    use_a_hold <= use_a;
                    idx_hold <= idx;
                    state <= S_WAIT64;
                end

                S_WAIT64: if (mem_rd64_valid) begin
                    data64_hold <= mem_rd64_data;
                    pack64_hold <= 64'd0;
                    pack64_idx <= 3'd0;
                    state <= S_PACK64;
                end

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
                    if (pack64_idx == 3'd7) begin
                        pack64_idx <= 3'd0;
                        state <= S_WRITE64;
                    end else begin
                        pack64_idx <= pack64_idx + 3'd1;
                        state <= S_PACK64;
                    end
                end

                S_WRITE64: if (mem_wr64_ready) begin
                    if ((idx + 32'd8) >= out_n) state <= S_DONE;
                    else begin idx <= idx + 32'd8; state <= S_REQ64; end
                end

                S_DONE: state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
