// concat_ddr_core.v
// Shared DDR-style channel concat core.
// Used by YOLOv5n layer12, layer16, layer19, layer22.
//
// Flow:
//   input_a_base = first tensor A_CH x H x W
//   input_b_base = second tensor B_CH x H x W
//   output_base  = concat [A, B] = (A_CH+B_CH) x H x W
//
// No MAC here. Later scheduler can reuse this core directly.

module concat_ddr_core #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32,

    parameter A_CH   = 128,
    parameter B_CH   = 128,
    parameter H      = 2,
    parameter W      = 2
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,

    input  wire [ADDR_W-1:0]       input_a_base,
    input  wire [ADDR_W-1:0]       input_b_base,
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

    localparam A_N  = A_CH * H * W;
    localparam B_N  = B_CH * H * W;
    localparam OUT_N = A_N + B_N;

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

    wire use_a;
    wire [31:0] src_offset;

    assign use_a      = (idx < A_N);
    assign src_offset = use_a ? idx : (idx - A_N);

    assign mem_rd_req  = (state == S_REQ);
    assign mem_rd_addr = use_a ? (input_a_base + src_offset) :
                                 (input_b_base + src_offset);

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
                        if (idx == OUT_N-1) begin
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
