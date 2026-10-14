// c3_noadd_ddr_core.v
// Shared DDR-style C3 no-shortcut core for YOLOv5n.
// Used by layer13, layer17, layer20, layer23.
//
// Flow:
//   cv1(input)        -> bank1
//   cv2(input)        -> bank2
//   m0.cv1(bank1)    -> bank3
//   m0.cv2(bank3)    -> bank4
//   concat(bank4,bank2) -> bank5
//   cv3(bank5)       -> output/bank7
//
// All conv ops call conv_layer_ddr -> conv2d_ddr_core.
// Later, increasing MAC_LANES is done in conv2d_ddr_core, not here.

module c3_noadd_ddr_core #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32,

    parameter IN_CH  = 256,
    parameter MID_CH = 64,
    parameter OUT_CH = 128,
    parameter H      = 2,
    parameter W      = 2,

    parameter CV1_W_BASE = 32'h0390_0000,
    parameter CV1_B_BASE = 32'h0390_0000,
    parameter CV2_W_BASE = 32'h0390_0000,
    parameter CV2_B_BASE = 32'h0390_0000,
    parameter M0CV1_W_BASE = 32'h0390_0000,
    parameter M0CV1_B_BASE = 32'h0390_0000,
    parameter M0CV2_W_BASE = 32'h0390_0000,
    parameter M0CV2_B_BASE = 32'h0390_0000,
    parameter CV3_W_BASE = 32'h0390_0000,
    parameter CV3_B_BASE = 32'h0390_0000,

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

    localparam S_IDLE     = 4'd0;
    localparam S_CV1      = 4'd1;
    localparam S_CV2      = 4'd2;
    localparam S_M0CV1    = 4'd3;
    localparam S_M0CV2    = 4'd4;
    localparam S_CAT_REQ  = 4'd5;
    localparam S_CAT_WAIT = 4'd6;
    localparam S_CAT_WR   = 4'd7;
    localparam S_CV3      = 4'd8;
    localparam S_DONE     = 4'd9;

    reg [3:0] state;

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    reg start_cv1;
    reg start_cv2;
    reg start_m0cv1;
    reg start_m0cv2;
    reg start_cv3;

    // ------------------------------------------------------------
    // Conv wires
    // ------------------------------------------------------------
    wire cv1_done, cv2_done, m0cv1_done, m0cv2_done, cv3_done;

    wire cv1_rd_req, cv2_rd_req, m0cv1_rd_req, m0cv2_rd_req, cv3_rd_req;
    wire [ADDR_W-1:0] cv1_rd_addr, cv2_rd_addr, m0cv1_rd_addr, m0cv2_rd_addr, cv3_rd_addr;

    wire cv1_wr_req, cv2_wr_req, m0cv1_wr_req, m0cv2_wr_req, cv3_wr_req;
    wire [ADDR_W-1:0] cv1_wr_addr, cv2_wr_addr, m0cv1_wr_addr, m0cv2_wr_addr, cv3_wr_addr;
    wire [DATA_W-1:0] cv1_wr_data, cv2_wr_data, m0cv1_wr_data, m0cv2_wr_data, cv3_wr_data;

    wire cv1_final_wr_en, cv2_final_wr_en, m0cv1_final_wr_en, m0cv2_final_wr_en, cv3_final_wr_en;
    wire [ADDR_W-1:0] cv1_final_wr_addr, cv2_final_wr_addr, m0cv1_final_wr_addr, m0cv2_final_wr_addr, cv3_final_wr_addr;
    wire [DATA_W-1:0] cv1_final_wr_data, cv2_final_wr_data, m0cv1_final_wr_data, m0cv2_final_wr_data, cv3_final_wr_data;

    wire cv1_out_valid, cv2_out_valid, m0cv1_out_valid, m0cv2_out_valid, cv3_out_valid;
    wire signed [ACC_W-1:0] cv1_out_data, cv2_out_data, m0cv1_out_data, m0cv2_out_data, cv3_out_data;

    wire sel_cv1   = (state == S_CV1);
    wire sel_cv2   = (state == S_CV2);
    wire sel_m0cv1 = (state == S_M0CV1);
    wire sel_m0cv2 = (state == S_M0CV2);
    wire sel_cv3   = (state == S_CV3);
    wire sel_cat   = (state == S_CAT_REQ) || (state == S_CAT_WAIT) || (state == S_CAT_WR);

    // ------------------------------------------------------------
    // Conv instances
    // ------------------------------------------------------------
    conv_layer_ddr #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .ACC_W(ACC_W),
        .IN_CH(IN_CH),
        .OUT_CH(MID_CH),
        .IN_W(W),
        .IN_H(H),
        .OUT_W(W),
        .OUT_H(H),
        .K_SIZE(1),
        .STRIDE(1),
        .PAD(0),
        .W_NUM(MID_CH*IN_CH),
        .B_NUM(MID_CH),
        .W_BASE(CV1_W_BASE),
        .B_BASE(CV1_B_BASE),
        .MAC_LANES(MAC_LANES)
    ) u_cv1 (
        .clk(clk), .rst_n(rst_n), .start(start_cv1),
        .input_base(input_base), .output_base(bank1_base),

        .mem_rd_req(cv1_rd_req), .mem_rd_addr(cv1_rd_addr),
        .mem_rd_ready(sel_cv1 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_cv1 ? mem_rd_valid : 1'b0),
        .mem_rd_data(mem_rd_data),

        .mem_wr_req(cv1_wr_req), .mem_wr_addr(cv1_wr_addr), .mem_wr_data(cv1_wr_data),
        .mem_wr_ready(sel_cv1 ? mem_wr_ready : 1'b0),

        .busy(), .done(cv1_done),
        .out_valid(cv1_out_valid), .out_data(cv1_out_data),
        .final_wr_en(cv1_final_wr_en), .final_wr_addr(cv1_final_wr_addr), .final_wr_data(cv1_final_wr_data)
    );

    conv_layer_ddr #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .ACC_W(ACC_W),
        .IN_CH(IN_CH),
        .OUT_CH(MID_CH),
        .IN_W(W),
        .IN_H(H),
        .OUT_W(W),
        .OUT_H(H),
        .K_SIZE(1),
        .STRIDE(1),
        .PAD(0),
        .W_NUM(MID_CH*IN_CH),
        .B_NUM(MID_CH),
        .W_BASE(CV2_W_BASE),
        .B_BASE(CV2_B_BASE),
        .MAC_LANES(MAC_LANES)
    ) u_cv2 (
        .clk(clk), .rst_n(rst_n), .start(start_cv2),
        .input_base(input_base), .output_base(bank2_base),

        .mem_rd_req(cv2_rd_req), .mem_rd_addr(cv2_rd_addr),
        .mem_rd_ready(sel_cv2 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_cv2 ? mem_rd_valid : 1'b0),
        .mem_rd_data(mem_rd_data),

        .mem_wr_req(cv2_wr_req), .mem_wr_addr(cv2_wr_addr), .mem_wr_data(cv2_wr_data),
        .mem_wr_ready(sel_cv2 ? mem_wr_ready : 1'b0),

        .busy(), .done(cv2_done),
        .out_valid(cv2_out_valid), .out_data(cv2_out_data),
        .final_wr_en(cv2_final_wr_en), .final_wr_addr(cv2_final_wr_addr), .final_wr_data(cv2_final_wr_data)
    );

    conv_layer_ddr #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .ACC_W(ACC_W),
        .IN_CH(MID_CH),
        .OUT_CH(MID_CH),
        .IN_W(W),
        .IN_H(H),
        .OUT_W(W),
        .OUT_H(H),
        .K_SIZE(1),
        .STRIDE(1),
        .PAD(0),
        .W_NUM(MID_CH*MID_CH),
        .B_NUM(MID_CH),
        .W_BASE(M0CV1_W_BASE),
        .B_BASE(M0CV1_B_BASE),
        .MAC_LANES(MAC_LANES)
    ) u_m0cv1 (
        .clk(clk), .rst_n(rst_n), .start(start_m0cv1),
        .input_base(bank1_base), .output_base(bank3_base),

        .mem_rd_req(m0cv1_rd_req), .mem_rd_addr(m0cv1_rd_addr),
        .mem_rd_ready(sel_m0cv1 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_m0cv1 ? mem_rd_valid : 1'b0),
        .mem_rd_data(mem_rd_data),

        .mem_wr_req(m0cv1_wr_req), .mem_wr_addr(m0cv1_wr_addr), .mem_wr_data(m0cv1_wr_data),
        .mem_wr_ready(sel_m0cv1 ? mem_wr_ready : 1'b0),

        .busy(), .done(m0cv1_done),
        .out_valid(m0cv1_out_valid), .out_data(m0cv1_out_data),
        .final_wr_en(m0cv1_final_wr_en), .final_wr_addr(m0cv1_final_wr_addr), .final_wr_data(m0cv1_final_wr_data)
    );

    conv_layer_ddr #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .ACC_W(ACC_W),
        .IN_CH(MID_CH),
        .OUT_CH(MID_CH),
        .IN_W(W),
        .IN_H(H),
        .OUT_W(W),
        .OUT_H(H),
        .K_SIZE(3),
        .STRIDE(1),
        .PAD(1),
        .W_NUM(MID_CH*MID_CH*3*3),
        .B_NUM(MID_CH),
        .W_BASE(M0CV2_W_BASE),
        .B_BASE(M0CV2_B_BASE),
        .MAC_LANES(MAC_LANES)
    ) u_m0cv2 (
        .clk(clk), .rst_n(rst_n), .start(start_m0cv2),
        .input_base(bank3_base), .output_base(bank4_base),

        .mem_rd_req(m0cv2_rd_req), .mem_rd_addr(m0cv2_rd_addr),
        .mem_rd_ready(sel_m0cv2 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_m0cv2 ? mem_rd_valid : 1'b0),
        .mem_rd_data(mem_rd_data),

        .mem_wr_req(m0cv2_wr_req), .mem_wr_addr(m0cv2_wr_addr), .mem_wr_data(m0cv2_wr_data),
        .mem_wr_ready(sel_m0cv2 ? mem_wr_ready : 1'b0),

        .busy(), .done(m0cv2_done),
        .out_valid(m0cv2_out_valid), .out_data(m0cv2_out_data),
        .final_wr_en(m0cv2_final_wr_en), .final_wr_addr(m0cv2_final_wr_addr), .final_wr_data(m0cv2_final_wr_data)
    );

    conv_layer_ddr #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .ACC_W(ACC_W),
        .IN_CH(2*MID_CH),
        .OUT_CH(OUT_CH),
        .IN_W(W),
        .IN_H(H),
        .OUT_W(W),
        .OUT_H(H),
        .K_SIZE(1),
        .STRIDE(1),
        .PAD(0),
        .W_NUM(OUT_CH*2*MID_CH),
        .B_NUM(OUT_CH),
        .W_BASE(CV3_W_BASE),
        .B_BASE(CV3_B_BASE),
        .MAC_LANES(MAC_LANES)
    ) u_cv3 (
        .clk(clk), .rst_n(rst_n), .start(start_cv3),
        .input_base(bank5_base), .output_base(output_base),

        .mem_rd_req(cv3_rd_req), .mem_rd_addr(cv3_rd_addr),
        .mem_rd_ready(sel_cv3 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_cv3 ? mem_rd_valid : 1'b0),
        .mem_rd_data(mem_rd_data),

        .mem_wr_req(cv3_wr_req), .mem_wr_addr(cv3_wr_addr), .mem_wr_data(cv3_wr_data),
        .mem_wr_ready(sel_cv3 ? mem_wr_ready : 1'b0),

        .busy(), .done(cv3_done),
        .out_valid(cv3_out_valid), .out_data(cv3_out_data),
        .final_wr_en(cv3_final_wr_en), .final_wr_addr(cv3_final_wr_addr), .final_wr_data(cv3_final_wr_data)
    );

    // ------------------------------------------------------------
    // Concat [bank4, bank2] -> bank5
    // ------------------------------------------------------------
    localparam N_MID = MID_CH * H * W;
    localparam N_CAT = 2 * MID_CH * H * W;

    integer cat_idx;
    reg [DATA_W-1:0] cat_hold;

    wire [31:0] cat_src_offset;
    assign cat_src_offset = cat_idx - N_MID;

    wire [ADDR_W-1:0] cat_rd_addr;
    wire [ADDR_W-1:0] cat_wr_addr;

    assign cat_rd_addr = (cat_idx < N_MID) ?
                         (bank4_base + cat_idx) :
                         (bank2_base + cat_src_offset);

    assign cat_wr_addr = bank5_base + cat_idx;

    wire cat_rd_req = (state == S_CAT_REQ);
    wire cat_wr_req = (state == S_CAT_WR);

    // ------------------------------------------------------------
    // DDR mux
    // ------------------------------------------------------------
    assign mem_rd_req =
        sel_cv1   ? cv1_rd_req   :
        sel_cv2   ? cv2_rd_req   :
        sel_m0cv1 ? m0cv1_rd_req :
        sel_m0cv2 ? m0cv2_rd_req :
        sel_cv3   ? cv3_rd_req   :
        sel_cat   ? cat_rd_req   :
                    1'b0;

    assign mem_rd_addr =
        sel_cv1   ? cv1_rd_addr   :
        sel_cv2   ? cv2_rd_addr   :
        sel_m0cv1 ? m0cv1_rd_addr :
        sel_m0cv2 ? m0cv2_rd_addr :
        sel_cv3   ? cv3_rd_addr   :
        sel_cat   ? cat_rd_addr   :
                    {ADDR_W{1'b0}};

    assign mem_wr_req =
        sel_cv1   ? cv1_wr_req   :
        sel_cv2   ? cv2_wr_req   :
        sel_m0cv1 ? m0cv1_wr_req :
        sel_m0cv2 ? m0cv2_wr_req :
        sel_cv3   ? cv3_wr_req   :
        sel_cat   ? cat_wr_req   :
                    1'b0;

    assign mem_wr_addr =
        sel_cv1   ? cv1_wr_addr   :
        sel_cv2   ? cv2_wr_addr   :
        sel_m0cv1 ? m0cv1_wr_addr :
        sel_m0cv2 ? m0cv2_wr_addr :
        sel_cv3   ? cv3_wr_addr   :
        sel_cat   ? cat_wr_addr   :
                    {ADDR_W{1'b0}};

    assign mem_wr_data =
        sel_cv1   ? cv1_wr_data   :
        sel_cv2   ? cv2_wr_data   :
        sel_m0cv1 ? m0cv1_wr_data :
        sel_m0cv2 ? m0cv2_wr_data :
        sel_cv3   ? cv3_wr_data   :
        sel_cat   ? cat_hold      :
                    {DATA_W{1'b0}};

    assign final_wr_en =
        sel_cv1   ? cv1_final_wr_en   :
        sel_cv2   ? cv2_final_wr_en   :
        sel_m0cv1 ? m0cv1_final_wr_en :
        sel_m0cv2 ? m0cv2_final_wr_en :
        sel_cv3   ? cv3_final_wr_en   :
        sel_cat   ? (cat_wr_req && mem_wr_ready) :
                    1'b0;

    assign final_wr_addr =
        sel_cv1   ? cv1_final_wr_addr   :
        sel_cv2   ? cv2_final_wr_addr   :
        sel_m0cv1 ? m0cv1_final_wr_addr :
        sel_m0cv2 ? m0cv2_final_wr_addr :
        sel_cv3   ? cv3_final_wr_addr   :
        sel_cat   ? cat_wr_addr         :
                    {ADDR_W{1'b0}};

    assign final_wr_data =
        sel_cv1   ? cv1_final_wr_data   :
        sel_cv2   ? cv2_final_wr_data   :
        sel_m0cv1 ? m0cv1_final_wr_data :
        sel_m0cv2 ? m0cv2_final_wr_data :
        sel_cv3   ? cv3_final_wr_data   :
        sel_cat   ? cat_hold            :
                    {DATA_W{1'b0}};

    assign out_valid =
        sel_cv1   ? cv1_out_valid   :
        sel_cv2   ? cv2_out_valid   :
        sel_m0cv1 ? m0cv1_out_valid :
        sel_m0cv2 ? m0cv2_out_valid :
        sel_cv3   ? cv3_out_valid   :
                    final_wr_en;

    assign out_data =
        sel_cv1   ? cv1_out_data   :
        sel_cv2   ? cv2_out_data   :
        sel_m0cv1 ? m0cv1_out_data :
        sel_m0cv2 ? m0cv2_out_data :
        sel_cv3   ? cv3_out_data   :
                    {ACC_W{1'b0}};

    // ------------------------------------------------------------
    // FSM
    // ------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;

            start_cv1   <= 1'b0;
            start_cv2   <= 1'b0;
            start_m0cv1 <= 1'b0;
            start_m0cv2 <= 1'b0;
            start_cv3   <= 1'b0;

            cat_idx <= 0;
            cat_hold <= 0;
        end else begin
            start_cv1   <= 1'b0;
            start_cv2   <= 1'b0;
            start_m0cv1 <= 1'b0;
            start_m0cv2 <= 1'b0;
            start_cv3   <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        start_cv1 <= 1'b1;
                        state <= S_CV1;
                    end
                end

                S_CV1: begin
                    if (cv1_done) begin
                        start_cv2 <= 1'b1;
                        state <= S_CV2;
                    end
                end

                S_CV2: begin
                    if (cv2_done) begin
                        start_m0cv1 <= 1'b1;
                        state <= S_M0CV1;
                    end
                end

                S_M0CV1: begin
                    if (m0cv1_done) begin
                        start_m0cv2 <= 1'b1;
                        state <= S_M0CV2;
                    end
                end

                S_M0CV2: begin
                    if (m0cv2_done) begin
                        cat_idx <= 0;
                        state <= S_CAT_REQ;
                    end
                end

                S_CAT_REQ: begin
                    if (mem_rd_ready) begin
                        state <= S_CAT_WAIT;
                    end
                end

                S_CAT_WAIT: begin
                    if (mem_rd_valid) begin
                        cat_hold <= mem_rd_data;
                        state <= S_CAT_WR;
                    end
                end

                S_CAT_WR: begin
                    if (mem_wr_ready) begin
                        if (cat_idx == N_CAT-1) begin
                            start_cv3 <= 1'b1;
                            state <= S_CV3;
                        end else begin
                            cat_idx <= cat_idx + 1;
                            state <= S_CAT_REQ;
                        end
                    end
                end

                S_CV3: begin
                    if (cv3_done) begin
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
