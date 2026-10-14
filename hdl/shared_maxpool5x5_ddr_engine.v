// shared_maxpool5x5_ddr_engine.v
// Runtime-configurable 5x5 stride1 pad2 maxpool for CHW signed-int8.
module shared_maxpool5x5_ddr_engine #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire [15:0] ch_n,
    input  wire [15:0] h,
    input  wire [15:0] w,
    input  wire [ADDR_W-1:0] input_base,
    input  wire [ADDR_W-1:0] output_base,
    output wire mem_rd_req,
    output wire [ADDR_W-1:0] mem_rd_addr,
    input  wire mem_rd_ready,
    input  wire mem_rd_valid,
    input  wire [DATA_W-1:0] mem_rd_data,
    output wire mem_wr_req,
    output wire [ADDR_W-1:0] mem_wr_addr,
    output wire [DATA_W-1:0] mem_wr_data,
    input  wire mem_wr_ready,
    output wire busy,
    output wire done
);
    localparam S_IDLE=4'd0, S_PREP=4'd1, S_SCAN=4'd2, S_REQ=4'd3, S_WAIT=4'd4, S_WRITE=4'd5, S_DONE=4'd6;
    reg [3:0] state;
    integer c, oy, ox, ky, kx;
    reg signed [7:0] max_val;
    wire signed [31:0] iy_calc = oy + ky - 2;
    wire signed [31:0] ix_calc = ox + kx - 2;
    wire in_bound = (iy_calc >= 0) && (iy_calc < h) && (ix_calc >= 0) && (ix_calc < w);
    wire [31:0] spatial = {16'd0,h} * {16'd0,w};
    wire [31:0] in_offset = c * spatial + iy_calc * {16'd0,w} + ix_calc;
    wire [31:0] out_offset = c * spatial + oy * {16'd0,w} + ox;
    wire last_k = (ky == 4) && (kx == 4);
    wire last_output = (c == ch_n-1) && (oy == h-1) && (ox == w-1);
    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);
    assign mem_rd_req = (state == S_REQ);
    assign mem_rd_addr = input_base + in_offset[ADDR_W-1:0];
    assign mem_wr_req = (state == S_WRITE);
    assign mem_wr_addr = output_base + out_offset[ADDR_W-1:0];
    assign mem_wr_data = max_val[7:0];
    task advance_k; begin
        if (kx == 4) begin kx <= 0; if (ky == 4) ky <= 0; else ky <= ky + 1; end
        else kx <= kx + 1;
    end endtask
    task advance_output; begin
        if (ox == w-1) begin ox <= 0; if (oy == h-1) begin oy <= 0; if (c == ch_n-1) c <= 0; else c <= c + 1; end else oy <= oy + 1; end
        else ox <= ox + 1;
    end endtask
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin state <= S_IDLE; c <= 0; oy <= 0; ox <= 0; ky <= 0; kx <= 0; max_val <= -128; end else begin
            case (state)
                S_IDLE: if (start) begin c <= 0; oy <= 0; ox <= 0; state <= S_PREP; end
                S_PREP: begin ky <= 0; kx <= 0; max_val <= -128; state <= S_SCAN; end
                S_SCAN: begin if (in_bound) state <= S_REQ; else if (last_k) state <= S_WRITE; else begin advance_k(); state <= S_SCAN; end end
                S_REQ: if (mem_rd_ready) state <= S_WAIT;
                S_WAIT: if (mem_rd_valid) begin if ($signed(mem_rd_data) > max_val) max_val <= $signed(mem_rd_data); if (last_k) state <= S_WRITE; else begin advance_k(); state <= S_SCAN; end end
                S_WRITE: if (mem_wr_ready) begin if (last_output) state <= S_DONE; else begin advance_output(); state <= S_PREP; end end
                S_DONE: state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
