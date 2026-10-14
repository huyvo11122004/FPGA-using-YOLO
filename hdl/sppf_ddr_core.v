// sppf_ddr_core.v
// Real DDR-style SPPF core for YOLOv5n.
// Supports both mini H=W=1 test and real 320-flow H=W=10.
//
// Flow:
//   cv1(input)          -> bank1
//   maxpool5x5(bank1)   -> bank2
//   maxpool5x5(bank2)   -> bank3
//   maxpool5x5(bank3)   -> bank4
//   concat [1,2,3,4]    -> bank5
//   cv2(bank5)          -> output/bank7

module sppf_ddr_core #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32,

    parameter IN_CH  = 256,
    parameter MID_CH = 128,
    parameter OUT_CH = 256,
    parameter H      = 1,
    parameter W      = 1,

    parameter CV1_W_BASE = 32'h0390_0000,
    parameter CV1_B_BASE = 32'h0390_0000,
    parameter CV2_W_BASE = 32'h0390_0000,
    parameter CV2_B_BASE = 32'h0390_0000,

    parameter MAC_LANES = 4
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,

    input  wire [ADDR_W-1:0]       input_base,
    input  wire [ADDR_W-1:0]       bank1_base,
    input  wire [ADDR_W-1:0]       bank2_base,
    input  wire [ADDR_W-1:0]       bank3_base,
    input  wire [ADDR_W-1:0]       bank4_base,
    input  wire [ADDR_W-1:0]       bank5_base,
    input  wire [ADDR_W-1:0]       bank6_base,   // unused/kept zero for standalone compare compatibility
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

    localparam S_IDLE  = 4'd0;
    localparam S_CV1   = 4'd1;
    localparam S_P1    = 4'd2;
    localparam S_P2    = 4'd3;
    localparam S_P3    = 4'd4;
    localparam S_CAT_RA = 4'd5;
    localparam S_CAT_WA = 4'd6;
    localparam S_CAT_WR = 4'd7;
    localparam S_CV2   = 4'd8;
    localparam S_DONE  = 4'd9;

    reg [3:0] state;

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    reg start_cv1;
    reg start_p1;
    reg start_p2;
    reg start_p3;
    reg start_cv2;

    wire cv1_done;
    wire p1_done;
    wire p2_done;
    wire p3_done;
    wire cv2_done;

    wire cv1_rd_req;
    wire [ADDR_W-1:0] cv1_rd_addr;
    wire cv1_wr_req;
    wire [ADDR_W-1:0] cv1_wr_addr;
    wire [DATA_W-1:0] cv1_wr_data;
    wire cv1_final_wr_en;
    wire [ADDR_W-1:0] cv1_final_wr_addr;
    wire [DATA_W-1:0] cv1_final_wr_data;
    wire cv1_out_valid;
    wire signed [ACC_W-1:0] cv1_out_data;

    wire p1_rd_req;
    wire [ADDR_W-1:0] p1_rd_addr;
    wire p1_wr_req;
    wire [ADDR_W-1:0] p1_wr_addr;
    wire [DATA_W-1:0] p1_wr_data;
    wire p1_final_wr_en;
    wire [ADDR_W-1:0] p1_final_wr_addr;
    wire [DATA_W-1:0] p1_final_wr_data;
    wire p1_out_valid;
    wire signed [ACC_W-1:0] p1_out_data;

    wire p2_rd_req;
    wire [ADDR_W-1:0] p2_rd_addr;
    wire p2_wr_req;
    wire [ADDR_W-1:0] p2_wr_addr;
    wire [DATA_W-1:0] p2_wr_data;
    wire p2_final_wr_en;
    wire [ADDR_W-1:0] p2_final_wr_addr;
    wire [DATA_W-1:0] p2_final_wr_data;
    wire p2_out_valid;
    wire signed [ACC_W-1:0] p2_out_data;

    wire p3_rd_req;
    wire [ADDR_W-1:0] p3_rd_addr;
    wire p3_wr_req;
    wire [ADDR_W-1:0] p3_wr_addr;
    wire [DATA_W-1:0] p3_wr_data;
    wire p3_final_wr_en;
    wire [ADDR_W-1:0] p3_final_wr_addr;
    wire [DATA_W-1:0] p3_final_wr_data;
    wire p3_out_valid;
    wire signed [ACC_W-1:0] p3_out_data;

    wire cv2_rd_req;
    wire [ADDR_W-1:0] cv2_rd_addr;
    wire cv2_wr_req;
    wire [ADDR_W-1:0] cv2_wr_addr;
    wire [DATA_W-1:0] cv2_wr_data;
    wire cv2_final_wr_en;
    wire [ADDR_W-1:0] cv2_final_wr_addr;
    wire [DATA_W-1:0] cv2_final_wr_data;
    wire cv2_out_valid;
    wire signed [ACC_W-1:0] cv2_out_data;

    wire sel_cv1 = (state == S_CV1);
    wire sel_p1  = (state == S_P1);
    wire sel_p2  = (state == S_P2);
    wire sel_p3  = (state == S_P3);
    wire sel_cv2 = (state == S_CV2);
    wire sel_cat = (state == S_CAT_RA) || (state == S_CAT_WA) || (state == S_CAT_WR);

    // ------------------------------------------------------------
    // Conv / Pool instances
    // ------------------------------------------------------------
    conv_layer_ddr #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
        .IN_CH(IN_CH), .OUT_CH(MID_CH), .IN_W(W), .IN_H(H), .OUT_W(W), .OUT_H(H),
        .K_SIZE(1), .STRIDE(1), .PAD(0),
        .W_NUM(MID_CH*IN_CH), .B_NUM(MID_CH),
        .W_BASE(CV1_W_BASE), .B_BASE(CV1_B_BASE),
        .MAC_LANES(MAC_LANES)
    ) u_cv1 (
        .clk(clk), .rst_n(rst_n), .start(start_cv1),
        .input_base(input_base), .output_base(bank1_base),
        .mem_rd_req(cv1_rd_req), .mem_rd_addr(cv1_rd_addr), .mem_rd_ready(sel_cv1 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_cv1 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
        .mem_wr_req(cv1_wr_req), .mem_wr_addr(cv1_wr_addr), .mem_wr_data(cv1_wr_data), .mem_wr_ready(sel_cv1 ? mem_wr_ready : 1'b0),
        .busy(), .done(cv1_done),
        .out_valid(cv1_out_valid), .out_data(cv1_out_data),
        .final_wr_en(cv1_final_wr_en), .final_wr_addr(cv1_final_wr_addr), .final_wr_data(cv1_final_wr_data)
    );

    maxpool5x5_ddr_core #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
        .CH(MID_CH), .H(H), .W(W)
    ) u_pool1 (
        .clk(clk), .rst_n(rst_n), .start(start_p1),
        .input_base(bank1_base), .output_base(bank2_base),
        .mem_rd_req(p1_rd_req), .mem_rd_addr(p1_rd_addr), .mem_rd_ready(sel_p1 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_p1 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
        .mem_wr_req(p1_wr_req), .mem_wr_addr(p1_wr_addr), .mem_wr_data(p1_wr_data), .mem_wr_ready(sel_p1 ? mem_wr_ready : 1'b0),
        .busy(), .done(p1_done),
        .out_valid(p1_out_valid), .out_data(p1_out_data),
        .final_wr_en(p1_final_wr_en), .final_wr_addr(p1_final_wr_addr), .final_wr_data(p1_final_wr_data)
    );

    maxpool5x5_ddr_core #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
        .CH(MID_CH), .H(H), .W(W)
    ) u_pool2 (
        .clk(clk), .rst_n(rst_n), .start(start_p2),
        .input_base(bank2_base), .output_base(bank3_base),
        .mem_rd_req(p2_rd_req), .mem_rd_addr(p2_rd_addr), .mem_rd_ready(sel_p2 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_p2 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
        .mem_wr_req(p2_wr_req), .mem_wr_addr(p2_wr_addr), .mem_wr_data(p2_wr_data), .mem_wr_ready(sel_p2 ? mem_wr_ready : 1'b0),
        .busy(), .done(p2_done),
        .out_valid(p2_out_valid), .out_data(p2_out_data),
        .final_wr_en(p2_final_wr_en), .final_wr_addr(p2_final_wr_addr), .final_wr_data(p2_final_wr_data)
    );

    maxpool5x5_ddr_core #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
        .CH(MID_CH), .H(H), .W(W)
    ) u_pool3 (
        .clk(clk), .rst_n(rst_n), .start(start_p3),
        .input_base(bank3_base), .output_base(bank4_base),
        .mem_rd_req(p3_rd_req), .mem_rd_addr(p3_rd_addr), .mem_rd_ready(sel_p3 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_p3 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
        .mem_wr_req(p3_wr_req), .mem_wr_addr(p3_wr_addr), .mem_wr_data(p3_wr_data), .mem_wr_ready(sel_p3 ? mem_wr_ready : 1'b0),
        .busy(), .done(p3_done),
        .out_valid(p3_out_valid), .out_data(p3_out_data),
        .final_wr_en(p3_final_wr_en), .final_wr_addr(p3_final_wr_addr), .final_wr_data(p3_final_wr_data)
    );

    conv_layer_ddr #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
        .IN_CH(4*MID_CH), .OUT_CH(OUT_CH), .IN_W(W), .IN_H(H), .OUT_W(W), .OUT_H(H),
        .K_SIZE(1), .STRIDE(1), .PAD(0),
        .W_NUM(OUT_CH*4*MID_CH), .B_NUM(OUT_CH),
        .W_BASE(CV2_W_BASE), .B_BASE(CV2_B_BASE),
        .MAC_LANES(MAC_LANES)
    ) u_cv2 (
        .clk(clk), .rst_n(rst_n), .start(start_cv2),
        .input_base(bank5_base), .output_base(output_base),
        .mem_rd_req(cv2_rd_req), .mem_rd_addr(cv2_rd_addr), .mem_rd_ready(sel_cv2 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_cv2 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
        .mem_wr_req(cv2_wr_req), .mem_wr_addr(cv2_wr_addr), .mem_wr_data(cv2_wr_data), .mem_wr_ready(sel_cv2 ? mem_wr_ready : 1'b0),
        .busy(), .done(cv2_done),
        .out_valid(cv2_out_valid), .out_data(cv2_out_data),
        .final_wr_en(cv2_final_wr_en), .final_wr_addr(cv2_final_wr_addr), .final_wr_data(cv2_final_wr_data)
    );

    // ------------------------------------------------------------
    // Concat [bank1, bank2, bank3, bank4] -> bank5
    // ------------------------------------------------------------
    localparam N_MID = MID_CH * H * W;
    localparam N_CAT = 4 * MID_CH * H * W;

    integer idx;
    reg [DATA_W-1:0] cat_hold;

    wire [31:0] cat_block = idx / N_MID;
    wire [31:0] cat_off   = idx - (cat_block * N_MID);

    reg [ADDR_W-1:0] cat_src_base;
    always @(*) begin
        case (cat_block)
            0: cat_src_base = bank1_base;
            1: cat_src_base = bank2_base;
            2: cat_src_base = bank3_base;
            default: cat_src_base = bank4_base;
        endcase
    end

    wire cat_rd_req = (state == S_CAT_RA);
    wire cat_wr_req = (state == S_CAT_WR);

    wire [ADDR_W-1:0] cat_rd_addr = cat_src_base + cat_off;
    wire [ADDR_W-1:0] cat_wr_addr = bank5_base + idx;

    // ------------------------------------------------------------
    // DDR mux
    // ------------------------------------------------------------
    assign mem_rd_req =
        sel_cv1 ? cv1_rd_req :
        sel_p1  ? p1_rd_req  :
        sel_p2  ? p2_rd_req  :
        sel_p3  ? p3_rd_req  :
        sel_cv2 ? cv2_rd_req :
        sel_cat ? cat_rd_req :
                  1'b0;

    assign mem_rd_addr =
        sel_cv1 ? cv1_rd_addr :
        sel_p1  ? p1_rd_addr  :
        sel_p2  ? p2_rd_addr  :
        sel_p3  ? p3_rd_addr  :
        sel_cv2 ? cv2_rd_addr :
        sel_cat ? cat_rd_addr :
                  {ADDR_W{1'b0}};

    assign mem_wr_req =
        sel_cv1 ? cv1_wr_req :
        sel_p1  ? p1_wr_req  :
        sel_p2  ? p2_wr_req  :
        sel_p3  ? p3_wr_req  :
        sel_cv2 ? cv2_wr_req :
        sel_cat ? cat_wr_req :
                  1'b0;

    assign mem_wr_addr =
        sel_cv1 ? cv1_wr_addr :
        sel_p1  ? p1_wr_addr  :
        sel_p2  ? p2_wr_addr  :
        sel_p3  ? p3_wr_addr  :
        sel_cv2 ? cv2_wr_addr :
        sel_cat ? cat_wr_addr :
                  {ADDR_W{1'b0}};

    assign mem_wr_data =
        sel_cv1 ? cv1_wr_data :
        sel_p1  ? p1_wr_data  :
        sel_p2  ? p2_wr_data  :
        sel_p3  ? p3_wr_data  :
        sel_cv2 ? cv2_wr_data :
        sel_cat ? cat_hold    :
                  {DATA_W{1'b0}};

    assign final_wr_en   = mem_wr_req && mem_wr_ready;
    assign final_wr_addr = mem_wr_addr;
    assign final_wr_data = mem_wr_data;

    assign out_valid =
        sel_cv1 ? cv1_out_valid :
        sel_p1  ? p1_out_valid  :
        sel_p2  ? p2_out_valid  :
        sel_p3  ? p3_out_valid  :
        sel_cv2 ? cv2_out_valid :
                  final_wr_en;

    assign out_data =
        sel_cv1 ? cv1_out_data :
        sel_p1  ? p1_out_data  :
        sel_p2  ? p2_out_data  :
        sel_p3  ? p3_out_data  :
        sel_cv2 ? cv2_out_data :
                  {ACC_W{1'b0}};

    // ------------------------------------------------------------
    // FSM
    // ------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;

            start_cv1 <= 1'b0;
            start_p1  <= 1'b0;
            start_p2  <= 1'b0;
            start_p3  <= 1'b0;
            start_cv2 <= 1'b0;

            idx <= 0;
            cat_hold <= 0;
        end else begin
            start_cv1 <= 1'b0;
            start_p1  <= 1'b0;
            start_p2  <= 1'b0;
            start_p3  <= 1'b0;
            start_cv2 <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        start_cv1 <= 1'b1;
                        state <= S_CV1;
                    end
                end

                S_CV1: begin
                    if (cv1_done) begin
                        start_p1 <= 1'b1;
                        state <= S_P1;
                    end
                end

                S_P1: begin
                    if (p1_done) begin
                        start_p2 <= 1'b1;
                        state <= S_P2;
                    end
                end

                S_P2: begin
                    if (p2_done) begin
                        start_p3 <= 1'b1;
                        state <= S_P3;
                    end
                end

                S_P3: begin
                    if (p3_done) begin
                        idx <= 0;
                        state <= S_CAT_RA;
                    end
                end

                S_CAT_RA: begin
                    if (mem_rd_ready)
                        state <= S_CAT_WA;
                end

                S_CAT_WA: begin
                    if (mem_rd_valid) begin
                        cat_hold <= mem_rd_data;
                        state <= S_CAT_WR;
                    end
                end

                S_CAT_WR: begin
                    if (mem_wr_ready) begin
                        if (idx == N_CAT-1) begin
                            start_cv2 <= 1'b1;
                            state <= S_CV2;
                        end else begin
                            idx <= idx + 1;
                            state <= S_CAT_RA;
                        end
                    end
                end

                S_CV2: begin
                    if (cv2_done)
                        state <= S_DONE;
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
