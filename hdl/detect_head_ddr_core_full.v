// detect_head_ddr_core_full.v
// Full 320x320 Detect layer24 raw head.
// small  : L17 64x40x40
// medium : L20 128x20x20
// large  : L23 256x10x10

module detect_head_ddr_core_full #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32,
    parameter DET_CH = 33
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,

    input  wire [ADDR_W-1:0]       small_base,
    input  wire [ADDR_W-1:0]       medium_base,
    input  wire [ADDR_W-1:0]       large_base,

    output wire                    mem_rd_req,
    output wire [ADDR_W-1:0]       mem_rd_addr,
    input  wire                    mem_rd_ready,
    input  wire                    mem_rd_valid,
    input  wire [DATA_W-1:0]       mem_rd_data,

    output wire                    busy,
    output wire                    done,

    output wire                    det_valid,
    output wire [1:0]              det_branch,
    output wire [31:0]             det_index,
    output wire signed [ACC_W-1:0] det_data
);

    `include "yolo_weight_addr_map.vh"


    localparam S_IDLE  = 3'd0;
    localparam S_SMALL = 3'd1;
    localparam S_MED   = 3'd2;
    localparam S_LARGE = 3'd3;
    localparam S_DONE  = 3'd4;

    reg [2:0] state;

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    reg start_s;
    reg start_m;
    reg start_l;

    wire done_s;
    wire done_m;
    wire done_l;

    wire s_rd_req;
    wire [ADDR_W-1:0] s_rd_addr;
    wire s_valid;
    wire [31:0] s_index;
    wire signed [ACC_W-1:0] s_data;

    wire m_rd_req;
    wire [ADDR_W-1:0] m_rd_addr;
    wire m_valid;
    wire [31:0] m_index;
    wire signed [ACC_W-1:0] m_data;

    wire l_rd_req;
    wire [ADDR_W-1:0] l_rd_addr;
    wire l_valid;
    wire [31:0] l_index;
    wire signed [ACC_W-1:0] l_data;

    wire sel_s = (state == S_SMALL);
    wire sel_m = (state == S_MED);
    wire sel_l = (state == S_LARGE);

    detect_raw_conv_ddr_core #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .ACC_W(ACC_W),
        .IN_CH(64),
        .DET_CH(DET_CH),
        .H(40),
        .W(40),
        .W_BASE(YWT_l24_s_w),
        .B_BASE(YWT_l24_s_b)
    ) u_small (
        .clk(clk), .rst_n(rst_n), .start(start_s),
        .input_base(small_base),
        .mem_rd_req(s_rd_req), .mem_rd_addr(s_rd_addr),
        .mem_rd_ready(sel_s ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_s ? mem_rd_valid : 1'b0),
        .mem_rd_data(mem_rd_data),
        .busy(), .done(done_s),
        .raw_valid(s_valid), .raw_index(s_index), .raw_data(s_data)
    );

    detect_raw_conv_ddr_core #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .ACC_W(ACC_W),
        .IN_CH(128),
        .DET_CH(DET_CH),
        .H(20),
        .W(20),
        .W_BASE(YWT_l24_m_w),
        .B_BASE(YWT_l24_m_b)
    ) u_medium (
        .clk(clk), .rst_n(rst_n), .start(start_m),
        .input_base(medium_base),
        .mem_rd_req(m_rd_req), .mem_rd_addr(m_rd_addr),
        .mem_rd_ready(sel_m ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_m ? mem_rd_valid : 1'b0),
        .mem_rd_data(mem_rd_data),
        .busy(), .done(done_m),
        .raw_valid(m_valid), .raw_index(m_index), .raw_data(m_data)
    );

    detect_raw_conv_ddr_core #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .ACC_W(ACC_W),
        .IN_CH(256),
        .DET_CH(DET_CH),
        .H(10),
        .W(10),
        .W_BASE(YWT_l24_l_w),
        .B_BASE(YWT_l24_l_b)
    ) u_large (
        .clk(clk), .rst_n(rst_n), .start(start_l),
        .input_base(large_base),
        .mem_rd_req(l_rd_req), .mem_rd_addr(l_rd_addr),
        .mem_rd_ready(sel_l ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_l ? mem_rd_valid : 1'b0),
        .mem_rd_data(mem_rd_data),
        .busy(), .done(done_l),
        .raw_valid(l_valid), .raw_index(l_index), .raw_data(l_data)
    );

    assign mem_rd_req  = sel_s ? s_rd_req  : sel_m ? m_rd_req  : sel_l ? l_rd_req  : 1'b0;
    assign mem_rd_addr = sel_s ? s_rd_addr : sel_m ? m_rd_addr : sel_l ? l_rd_addr : {ADDR_W{1'b0}};

    assign det_valid  = sel_s ? s_valid : sel_m ? m_valid : sel_l ? l_valid : 1'b0;
    assign det_branch = sel_s ? 2'd0    : sel_m ? 2'd1    : sel_l ? 2'd2   : 2'd0;
    assign det_index  = sel_s ? s_index : sel_m ? m_index : sel_l ? l_index : 32'd0;
    assign det_data   = sel_s ? s_data  : sel_m ? m_data  : sel_l ? l_data  : {ACC_W{1'b0}};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            start_s <= 1'b0;
            start_m <= 1'b0;
            start_l <= 1'b0;
        end else begin
            start_s <= 1'b0;
            start_m <= 1'b0;
            start_l <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        start_s <= 1'b1;
                        state <= S_SMALL;
                    end
                end

                S_SMALL: begin
                    if (done_s) begin
                        start_m <= 1'b1;
                        state <= S_MED;
                    end
                end

                S_MED: begin
                    if (done_m) begin
                        start_l <= 1'b1;
                        state <= S_LARGE;
                    end
                end

                S_LARGE: begin
                    if (done_l) begin
                        state <= S_DONE;
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
