// shared_upsample_ddr_engine.v
// Runtime-configurable nearest-neighbor x2 upsample, CHW uint8.
// FIX204_COMBO_NONCONV64: add RD64/WR64 fast path for in_w multiple of 8.
module shared_upsample_ddr_engine #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire [15:0] ch_n,
    input  wire [15:0] in_h,
    input  wire [15:0] in_w,
    input  wire [ADDR_W-1:0] input_base,
    input  wire [ADDR_W-1:0] output_base,

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
    localparam S_REQ64=4'd5, S_WAIT64=4'd6, S_WR64_0=4'd7, S_WR64_1=4'd8, S_WR64_2=4'd9, S_WR64_3=4'd10;

    reg [3:0] state;
    integer c, oy, ox;
    reg [31:0] out_idx;
    reg [DATA_W-1:0] data_hold;
    reg [63:0] data64_hold;

    wire [31:0] out_h = {16'd0, in_h} << 1;
    wire [31:0] out_w = {16'd0, in_w} << 1;
    wire [31:0] in_spatial = {16'd0,in_h} * {16'd0,in_w};
    wire [31:0] out_spatial = out_h * out_w;

    // Byte fallback uses output coordinates.
    wire [31:0] iy_b = oy >> 1;
    wire [31:0] ix_b = ox >> 1;
    wire [31:0] in_offset_b = c * in_spatial + iy_b * {16'd0,in_w} + ix_b;
    wire last_out = (c == ch_n-1) && (oy == out_h-1) && (ox == out_w-1);

    // Fast path uses input coordinates. ox steps by 8 input pixels.
    wire fast64_en = (in_w[2:0] == 3'd0) && (in_w != 16'd0) && (in_h != 16'd0);
    wire [31:0] in_offset64 = c * in_spatial + oy * {16'd0,in_w} + ox;
    wire [31:0] out_offset64 = c * out_spatial + ((oy << 1) * out_w) + (ox << 1);
    wire [ADDR_W-1:0] out_addr0 = out_offset64;
    wire [ADDR_W-1:0] out_addr1 = out_offset64 + 32'd8;
    wire [ADDR_W-1:0] out_addr2 = out_offset64 + out_w;
    wire [ADDR_W-1:0] out_addr3 = out_offset64 + out_w + 32'd8;
    wire last_blk64 = (c == ch_n-1) && (oy == in_h-1) && ((ox + 8) >= in_w);

    function [63:0] dup4_lo;
        input [63:0] d;
        begin
            dup4_lo = {d[31:24],d[31:24],d[23:16],d[23:16],d[15:8],d[15:8],d[7:0],d[7:0]};
        end
    endfunction
    function [63:0] dup4_hi;
        input [63:0] d;
        begin
            dup4_hi = {d[63:56],d[63:56],d[55:48],d[55:48],d[47:40],d[47:40],d[39:32],d[39:32]};
        end
    endfunction

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    assign mem_rd_req = (state == S_REQ);
    assign mem_rd_addr = input_base + in_offset_b[ADDR_W-1:0];
    assign mem_rd64_req = (state == S_REQ64);
    assign mem_rd64_addr = input_base + in_offset64[ADDR_W-1:0];

    assign mem_wr_req = (state == S_WRITE);
    assign mem_wr_addr = output_base + out_idx[ADDR_W-1:0];
    assign mem_wr_data = data_hold;

    assign mem_wr64_req = (state == S_WR64_0) || (state == S_WR64_1) || (state == S_WR64_2) || (state == S_WR64_3);
    assign mem_wr64_addr = output_base + ((state == S_WR64_0) ? out_addr0 :
                                          (state == S_WR64_1) ? out_addr1 :
                                          (state == S_WR64_2) ? out_addr2 : out_addr3);
    assign mem_wr64_data = ((state == S_WR64_0) || (state == S_WR64_2)) ? dup4_lo(data64_hold) : dup4_hi(data64_hold);

    task advance_out; begin
        out_idx <= out_idx + 1;
        if (ox == out_w-1) begin
            ox <= 0;
            if (oy == out_h-1) begin
                oy <= 0;
                if (c == ch_n-1) c <= 0; else c <= c + 1;
            end else oy <= oy + 1;
        end else ox <= ox + 1;
    end endtask

    task advance_blk64; begin
        if ((ox + 8) >= in_w) begin
            ox <= 0;
            if (oy == in_h-1) begin
                oy <= 0;
                if (c == ch_n-1) c <= 0; else c <= c + 1;
            end else oy <= oy + 1;
        end else begin
            ox <= ox + 8;
        end
    end endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; c <= 0; oy <= 0; ox <= 0; out_idx <= 0; data_hold <= 0; data64_hold <= 0;
        end else begin
            case (state)
                S_IDLE: if (start) begin
                    c <= 0; oy <= 0; ox <= 0; out_idx <= 0; data64_hold <= 0;
                    state <= (ch_n == 0) ? S_DONE : (fast64_en ? S_REQ64 : S_REQ);
                end

                S_REQ: if (mem_rd_ready) state <= S_WAIT;
                S_WAIT: if (mem_rd_valid) begin data_hold <= mem_rd_data; state <= S_WRITE; end
                S_WRITE: if (mem_wr_ready) begin
                    if (last_out) state <= S_DONE; else begin advance_out(); state <= S_REQ; end
                end

                S_REQ64: if (mem_rd64_ready) state <= S_WAIT64;
                S_WAIT64: if (mem_rd64_valid) begin data64_hold <= mem_rd64_data; state <= S_WR64_0; end
                S_WR64_0: if (mem_wr64_ready) state <= S_WR64_1;
                S_WR64_1: if (mem_wr64_ready) state <= S_WR64_2;
                S_WR64_2: if (mem_wr64_ready) state <= S_WR64_3;
                S_WR64_3: if (mem_wr64_ready) begin
                    if (last_blk64) state <= S_DONE; else begin advance_blk64(); state <= S_REQ64; end
                end

                S_DONE: state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
