// upsample_ddr_core.v
// Shared DDR-style nearest upsample x2 core.
// Used by YOLOv5n layer11 and layer15.
//
// Flow:
//   input_base CHW tensor C x H x W
//   output_base CHW tensor C x (2H) x (2W)
//
// No MAC here. Later scheduler can reuse this core directly.

module upsample_ddr_core #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32,

    parameter IN_CH  = 128,
    parameter IN_H   = 1,
    parameter IN_W   = 1
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,

    input  wire [ADDR_W-1:0]       input_base,
    input  wire [ADDR_W-1:0]       output_base,

    output wire                    mem_rd_req,
    output wire [ADDR_W-1:0]       mem_rd_addr,
    input  wire                    mem_rd_ready,
    input  wire                    mem_rd_valid,
    input  wire [DATA_W-1:0]       mem_rd_data,

    output wire                    mem_wr_req,
    output wire [ADDR_W-1:0]       mem_wr_addr,
    output wire [DATA_W-1:0]       mem_wr_data,
    input  wire                    mem_wr_ready,

    output wire                    busy,
    output wire                    done,

    output wire                    out_valid,
    output wire signed [ACC_W-1:0] out_data,

    output wire                    final_wr_en,
    output wire [ADDR_W-1:0]       final_wr_addr,
    output wire [DATA_W-1:0]       final_wr_data
);

    localparam OUT_H = IN_H * 2;
    localparam OUT_W = IN_W * 2;
    localparam NOUT  = IN_CH * OUT_H * OUT_W;

    localparam S_IDLE  = 3'd0;
    localparam S_REQ   = 3'd1;
    localparam S_WAIT  = 3'd2;
    localparam S_WRITE = 3'd3;
    localparam S_DONE  = 3'd4;

    reg [2:0] state;
    integer idx;

    reg [DATA_W-1:0] data_hold;

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    wire [31:0] out_spatial;
    wire [31:0] ch;
    wire [31:0] rem;
    wire [31:0] oy;
    wire [31:0] ox;
    wire [31:0] iy;
    wire [31:0] ix;
    wire [31:0] in_offset;

    assign out_spatial = OUT_H * OUT_W;
    assign ch  = idx / out_spatial;
    assign rem = idx - (ch * out_spatial);
    assign oy  = rem / OUT_W;
    assign ox  = rem - (oy * OUT_W);
    assign iy  = oy >> 1;
    assign ix  = ox >> 1;

    assign in_offset = ch * (IN_H * IN_W) + iy * IN_W + ix;

    assign mem_rd_req  = (state == S_REQ);
    assign mem_rd_addr = input_base + in_offset;

    assign mem_wr_req  = (state == S_WRITE);
    assign mem_wr_addr = output_base + idx;
    assign mem_wr_data = data_hold;

    assign final_wr_en   = mem_wr_req && mem_wr_ready;
    assign final_wr_addr = mem_wr_addr;
    assign final_wr_data = mem_wr_data;

    assign out_valid = final_wr_en;
    assign out_data  = {{(ACC_W-DATA_W){1'b0}}, data_hold};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            idx <= 0;
            data_hold <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        idx <= 0;
                        state <= S_REQ;
                    end
                end

                S_REQ: begin
                    if (mem_rd_ready)
                        state <= S_WAIT;
                end

                S_WAIT: begin
                    if (mem_rd_valid) begin
                        data_hold <= mem_rd_data;
                        state <= S_WRITE;
                    end
                end

                S_WRITE: begin
                    if (mem_wr_ready) begin
                        if (idx == NOUT-1) begin
                            state <= S_DONE;
                        end else begin
                            idx <= idx + 1;
                            state <= S_REQ;
                        end
                    end
                end

                S_DONE: begin
                    state <= S_IDLE;
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
