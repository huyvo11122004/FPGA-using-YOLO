// c3_shortcut_ddr_core.v
// Shared DDR-style C3 shortcut core for YOLOv5n backbone C3 layers.
// Used by layer2, layer4, layer6, layer8.
//
// Flow:
//   cv1(input) -> bank1
//   cv2(input) -> bank2
//
//   m0.cv1(residual) -> bank3
//   m0.cv2(bank3)    -> bank4
//   add(residual + bank4) -> bank5
//
//   if NUM_M >= 2:
//     m1.cv1(bank5) -> bank3
//     m1.cv2(bank3) -> bank4
//     add(bank5 + bank4) -> bank1
//
//   if NUM_M >= 3:
//     m2.cv1(bank1) -> bank3
//     m2.cv2(bank3) -> bank4
//     add(bank1 + bank4) -> bank5
//
//   concat(final_residual, bank2) -> bank6
//   cv3(bank6) -> output/bank7
//
// All conv ops call conv_layer_ddr -> conv2d_ddr_core.

module c3_shortcut_ddr_core #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32,

    parameter IN_CH  = 32,
    parameter MID_CH = 16,
    parameter OUT_CH = 32,
    parameter H      = 8,
    parameter W      = 8,
    parameter NUM_M  = 1,

    parameter CV1_W_BASE = 32'h0390_0000,
    parameter CV1_B_BASE = 32'h0390_0000,
    parameter CV2_W_BASE = 32'h0390_0000,
    parameter CV2_B_BASE = 32'h0390_0000,

    parameter M0CV1_W_BASE = 32'h0390_0000,
    parameter M0CV1_B_BASE = 32'h0390_0000,
    parameter M0CV2_W_BASE = 32'h0390_0000,
    parameter M0CV2_B_BASE = 32'h0390_0000,

    parameter M1CV1_W_BASE = 32'h0390_0000,
    parameter M1CV1_B_BASE = 32'h0390_0000,
    parameter M1CV2_W_BASE = 32'h0390_0000,
    parameter M1CV2_B_BASE = 32'h0390_0000,

    parameter M2CV1_W_BASE = 32'h0390_0000,
    parameter M2CV1_B_BASE = 32'h0390_0000,
    parameter M2CV2_W_BASE = 32'h0390_0000,
    parameter M2CV2_B_BASE = 32'h0390_0000,

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
    input  wire [ADDR_W-1:0]       bank6_base,
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

    localparam S_IDLE       = 5'd0;
    localparam S_CV1        = 5'd1;
    localparam S_CV2        = 5'd2;
    localparam S_M0CV1      = 5'd3;
    localparam S_M0CV2      = 5'd4;
    localparam S_ADD0_RA    = 5'd5;
    localparam S_ADD0_WA    = 5'd6;
    localparam S_ADD0_RB    = 5'd7;
    localparam S_ADD0_WB    = 5'd8;
    localparam S_ADD0_WR    = 5'd9;
    localparam S_M1CV1      = 5'd10;
    localparam S_M1CV2      = 5'd11;
    localparam S_ADD1_RA    = 5'd12;
    localparam S_ADD1_WA    = 5'd13;
    localparam S_ADD1_RB    = 5'd14;
    localparam S_ADD1_WB    = 5'd15;
    localparam S_ADD1_WR    = 5'd16;
    localparam S_M2CV1      = 5'd17;
    localparam S_M2CV2      = 5'd18;
    localparam S_ADD2_RA    = 5'd19;
    localparam S_ADD2_WA    = 5'd20;
    localparam S_ADD2_RB    = 5'd21;
    localparam S_ADD2_WB    = 5'd22;
    localparam S_ADD2_WR    = 5'd23;
    localparam S_CAT_RA     = 5'd24;
    localparam S_CAT_WA     = 5'd25;
    localparam S_CAT_WR     = 5'd26;
    localparam S_CV3        = 5'd27;
    localparam S_DONE       = 5'd28;

    reg [4:0] state;

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    // ------------------------------------------------------------
    // Start pulses
    // ------------------------------------------------------------
    reg start_cv1, start_cv2;
    reg start_m0cv1, start_m0cv2;
    reg start_m1cv1, start_m1cv2;
    reg start_m2cv1, start_m2cv2;
    reg start_cv3;

    // ------------------------------------------------------------
    // Conv wires
    // ------------------------------------------------------------
    wire cv1_done, cv2_done, m0cv1_done, m0cv2_done, m1cv1_done, m1cv2_done, m2cv1_done, m2cv2_done, cv3_done;

    wire cv1_rd_req, cv2_rd_req, m0cv1_rd_req, m0cv2_rd_req, m1cv1_rd_req, m1cv2_rd_req, m2cv1_rd_req, m2cv2_rd_req, cv3_rd_req;
    wire [ADDR_W-1:0] cv1_rd_addr, cv2_rd_addr, m0cv1_rd_addr, m0cv2_rd_addr, m1cv1_rd_addr, m1cv2_rd_addr, m2cv1_rd_addr, m2cv2_rd_addr, cv3_rd_addr;

    wire cv1_wr_req, cv2_wr_req, m0cv1_wr_req, m0cv2_wr_req, m1cv1_wr_req, m1cv2_wr_req, m2cv1_wr_req, m2cv2_wr_req, cv3_wr_req;
    wire [ADDR_W-1:0] cv1_wr_addr, cv2_wr_addr, m0cv1_wr_addr, m0cv2_wr_addr, m1cv1_wr_addr, m1cv2_wr_addr, m2cv1_wr_addr, m2cv2_wr_addr, cv3_wr_addr;
    wire [DATA_W-1:0] cv1_wr_data, cv2_wr_data, m0cv1_wr_data, m0cv2_wr_data, m1cv1_wr_data, m1cv2_wr_data, m2cv1_wr_data, m2cv2_wr_data, cv3_wr_data;

    wire cv1_final_wr_en, cv2_final_wr_en, m0cv1_final_wr_en, m0cv2_final_wr_en, m1cv1_final_wr_en, m1cv2_final_wr_en, m2cv1_final_wr_en, m2cv2_final_wr_en, cv3_final_wr_en;
    wire [ADDR_W-1:0] cv1_final_wr_addr, cv2_final_wr_addr, m0cv1_final_wr_addr, m0cv2_final_wr_addr, m1cv1_final_wr_addr, m1cv2_final_wr_addr, m2cv1_final_wr_addr, m2cv2_final_wr_addr, cv3_final_wr_addr;
    wire [DATA_W-1:0] cv1_final_wr_data, cv2_final_wr_data, m0cv1_final_wr_data, m0cv2_final_wr_data, m1cv1_final_wr_data, m1cv2_final_wr_data, m2cv1_final_wr_data, m2cv2_final_wr_data, cv3_final_wr_data;

    wire cv1_out_valid, cv2_out_valid, m0cv1_out_valid, m0cv2_out_valid, m1cv1_out_valid, m1cv2_out_valid, m2cv1_out_valid, m2cv2_out_valid, cv3_out_valid;
    wire signed [ACC_W-1:0] cv1_out_data, cv2_out_data, m0cv1_out_data, m0cv2_out_data, m1cv1_out_data, m1cv2_out_data, m2cv1_out_data, m2cv2_out_data, cv3_out_data;

    wire sel_cv1   = (state == S_CV1);
    wire sel_cv2   = (state == S_CV2);
    wire sel_m0cv1 = (state == S_M0CV1);
    wire sel_m0cv2 = (state == S_M0CV2);
    wire sel_m1cv1 = (state == S_M1CV1);
    wire sel_m1cv2 = (state == S_M1CV2);
    wire sel_m2cv1 = (state == S_M2CV1);
    wire sel_m2cv2 = (state == S_M2CV2);
    wire sel_cv3   = (state == S_CV3);

    wire sel_add = (state == S_ADD0_RA) || (state == S_ADD0_WA) || (state == S_ADD0_RB) || (state == S_ADD0_WB) || (state == S_ADD0_WR) ||
                   (state == S_ADD1_RA) || (state == S_ADD1_WA) || (state == S_ADD1_RB) || (state == S_ADD1_WB) || (state == S_ADD1_WR) ||
                   (state == S_ADD2_RA) || (state == S_ADD2_WA) || (state == S_ADD2_RB) || (state == S_ADD2_WB) || (state == S_ADD2_WR);

    wire sel_cat = (state == S_CAT_RA) || (state == S_CAT_WA) || (state == S_CAT_WR);

    // ------------------------------------------------------------
    // Conv instances
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
        .busy(), .done(cv1_done), .out_valid(cv1_out_valid), .out_data(cv1_out_data),
        .final_wr_en(cv1_final_wr_en), .final_wr_addr(cv1_final_wr_addr), .final_wr_data(cv1_final_wr_data)
    );

    conv_layer_ddr #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
        .IN_CH(IN_CH), .OUT_CH(MID_CH), .IN_W(W), .IN_H(H), .OUT_W(W), .OUT_H(H),
        .K_SIZE(1), .STRIDE(1), .PAD(0),
        .W_NUM(MID_CH*IN_CH), .B_NUM(MID_CH),
        .W_BASE(CV2_W_BASE), .B_BASE(CV2_B_BASE),
        .MAC_LANES(MAC_LANES)
    ) u_cv2 (
        .clk(clk), .rst_n(rst_n), .start(start_cv2),
        .input_base(input_base), .output_base(bank2_base),
        .mem_rd_req(cv2_rd_req), .mem_rd_addr(cv2_rd_addr), .mem_rd_ready(sel_cv2 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_cv2 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
        .mem_wr_req(cv2_wr_req), .mem_wr_addr(cv2_wr_addr), .mem_wr_data(cv2_wr_data), .mem_wr_ready(sel_cv2 ? mem_wr_ready : 1'b0),
        .busy(), .done(cv2_done), .out_valid(cv2_out_valid), .out_data(cv2_out_data),
        .final_wr_en(cv2_final_wr_en), .final_wr_addr(cv2_final_wr_addr), .final_wr_data(cv2_final_wr_data)
    );

    conv_layer_ddr #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
        .IN_CH(MID_CH), .OUT_CH(MID_CH), .IN_W(W), .IN_H(H), .OUT_W(W), .OUT_H(H),
        .K_SIZE(1), .STRIDE(1), .PAD(0),
        .W_NUM(MID_CH*MID_CH), .B_NUM(MID_CH),
        .W_BASE(M0CV1_W_BASE), .B_BASE(M0CV1_B_BASE),
        .MAC_LANES(MAC_LANES)
    ) u_m0cv1 (
        .clk(clk), .rst_n(rst_n), .start(start_m0cv1),
        .input_base(bank1_base), .output_base(bank3_base),
        .mem_rd_req(m0cv1_rd_req), .mem_rd_addr(m0cv1_rd_addr), .mem_rd_ready(sel_m0cv1 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_m0cv1 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
        .mem_wr_req(m0cv1_wr_req), .mem_wr_addr(m0cv1_wr_addr), .mem_wr_data(m0cv1_wr_data), .mem_wr_ready(sel_m0cv1 ? mem_wr_ready : 1'b0),
        .busy(), .done(m0cv1_done), .out_valid(m0cv1_out_valid), .out_data(m0cv1_out_data),
        .final_wr_en(m0cv1_final_wr_en), .final_wr_addr(m0cv1_final_wr_addr), .final_wr_data(m0cv1_final_wr_data)
    );

    conv_layer_ddr #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
        .IN_CH(MID_CH), .OUT_CH(MID_CH), .IN_W(W), .IN_H(H), .OUT_W(W), .OUT_H(H),
        .K_SIZE(3), .STRIDE(1), .PAD(1),
        .W_NUM(MID_CH*MID_CH*3*3), .B_NUM(MID_CH),
        .W_BASE(M0CV2_W_BASE), .B_BASE(M0CV2_B_BASE),
        .MAC_LANES(MAC_LANES)
    ) u_m0cv2 (
        .clk(clk), .rst_n(rst_n), .start(start_m0cv2),
        .input_base(bank3_base), .output_base(bank4_base),
        .mem_rd_req(m0cv2_rd_req), .mem_rd_addr(m0cv2_rd_addr), .mem_rd_ready(sel_m0cv2 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_m0cv2 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
        .mem_wr_req(m0cv2_wr_req), .mem_wr_addr(m0cv2_wr_addr), .mem_wr_data(m0cv2_wr_data), .mem_wr_ready(sel_m0cv2 ? mem_wr_ready : 1'b0),
        .busy(), .done(m0cv2_done), .out_valid(m0cv2_out_valid), .out_data(m0cv2_out_data),
        .final_wr_en(m0cv2_final_wr_en), .final_wr_addr(m0cv2_final_wr_addr), .final_wr_data(m0cv2_final_wr_data)
    );

    // Optional m1/m2 bottleneck blocks.
    // NUM_M is a static parameter per top-level C3 instance, so synthesis can remove
    // unused conv cores for NUM_M=1 or NUM_M=2. This saves LUTs/DSPs.
    generate
        if (NUM_M >= 2) begin : GEN_M1
            conv_layer_ddr #(
                .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
                .IN_CH(MID_CH), .OUT_CH(MID_CH), .IN_W(W), .IN_H(H), .OUT_W(W), .OUT_H(H),
                .K_SIZE(1), .STRIDE(1), .PAD(0),
                .W_NUM(MID_CH*MID_CH), .B_NUM(MID_CH),
                .W_BASE(M1CV1_W_BASE), .B_BASE(M1CV1_B_BASE),
                .MAC_LANES(MAC_LANES)
            ) u_m1cv1 (
                .clk(clk), .rst_n(rst_n), .start(start_m1cv1),
                .input_base(bank5_base), .output_base(bank3_base),
                .mem_rd_req(m1cv1_rd_req), .mem_rd_addr(m1cv1_rd_addr), .mem_rd_ready(sel_m1cv1 ? mem_rd_ready : 1'b0),
                .mem_rd_valid(sel_m1cv1 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
                .mem_wr_req(m1cv1_wr_req), .mem_wr_addr(m1cv1_wr_addr), .mem_wr_data(m1cv1_wr_data), .mem_wr_ready(sel_m1cv1 ? mem_wr_ready : 1'b0),
                .busy(), .done(m1cv1_done), .out_valid(m1cv1_out_valid), .out_data(m1cv1_out_data),
                .final_wr_en(m1cv1_final_wr_en), .final_wr_addr(m1cv1_final_wr_addr), .final_wr_data(m1cv1_final_wr_data)
            );

            conv_layer_ddr #(
                .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
                .IN_CH(MID_CH), .OUT_CH(MID_CH), .IN_W(W), .IN_H(H), .OUT_W(W), .OUT_H(H),
                .K_SIZE(3), .STRIDE(1), .PAD(1),
                .W_NUM(MID_CH*MID_CH*3*3), .B_NUM(MID_CH),
                .W_BASE(M1CV2_W_BASE), .B_BASE(M1CV2_B_BASE),
                .MAC_LANES(MAC_LANES)
            ) u_m1cv2 (
                .clk(clk), .rst_n(rst_n), .start(start_m1cv2),
                .input_base(bank3_base), .output_base(bank4_base),
                .mem_rd_req(m1cv2_rd_req), .mem_rd_addr(m1cv2_rd_addr), .mem_rd_ready(sel_m1cv2 ? mem_rd_ready : 1'b0),
                .mem_rd_valid(sel_m1cv2 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
                .mem_wr_req(m1cv2_wr_req), .mem_wr_addr(m1cv2_wr_addr), .mem_wr_data(m1cv2_wr_data), .mem_wr_ready(sel_m1cv2 ? mem_wr_ready : 1'b0),
                .busy(), .done(m1cv2_done), .out_valid(m1cv2_out_valid), .out_data(m1cv2_out_data),
                .final_wr_en(m1cv2_final_wr_en), .final_wr_addr(m1cv2_final_wr_addr), .final_wr_data(m1cv2_final_wr_data)
            );
        end else begin : NO_M1
            assign m1cv1_done = 1'b1;
            assign m1cv1_rd_req = 1'b0;
            assign m1cv1_rd_addr = {ADDR_W{1'b0}};
            assign m1cv1_wr_req = 1'b0;
            assign m1cv1_wr_addr = {ADDR_W{1'b0}};
            assign m1cv1_wr_data = {DATA_W{1'b0}};
            assign m1cv1_final_wr_en = 1'b0;
            assign m1cv1_final_wr_addr = {ADDR_W{1'b0}};
            assign m1cv1_final_wr_data = {DATA_W{1'b0}};
            assign m1cv1_out_valid = 1'b0;
            assign m1cv1_out_data = {ACC_W{1'b0}};

            assign m1cv2_done = 1'b1;
            assign m1cv2_rd_req = 1'b0;
            assign m1cv2_rd_addr = {ADDR_W{1'b0}};
            assign m1cv2_wr_req = 1'b0;
            assign m1cv2_wr_addr = {ADDR_W{1'b0}};
            assign m1cv2_wr_data = {DATA_W{1'b0}};
            assign m1cv2_final_wr_en = 1'b0;
            assign m1cv2_final_wr_addr = {ADDR_W{1'b0}};
            assign m1cv2_final_wr_data = {DATA_W{1'b0}};
            assign m1cv2_out_valid = 1'b0;
            assign m1cv2_out_data = {ACC_W{1'b0}};
        end

        if (NUM_M >= 3) begin : GEN_M2
            conv_layer_ddr #(
                .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
                .IN_CH(MID_CH), .OUT_CH(MID_CH), .IN_W(W), .IN_H(H), .OUT_W(W), .OUT_H(H),
                .K_SIZE(1), .STRIDE(1), .PAD(0),
                .W_NUM(MID_CH*MID_CH), .B_NUM(MID_CH),
                .W_BASE(M2CV1_W_BASE), .B_BASE(M2CV1_B_BASE),
                .MAC_LANES(MAC_LANES)
            ) u_m2cv1 (
                .clk(clk), .rst_n(rst_n), .start(start_m2cv1),
                .input_base(bank1_base), .output_base(bank3_base),
                .mem_rd_req(m2cv1_rd_req), .mem_rd_addr(m2cv1_rd_addr), .mem_rd_ready(sel_m2cv1 ? mem_rd_ready : 1'b0),
                .mem_rd_valid(sel_m2cv1 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
                .mem_wr_req(m2cv1_wr_req), .mem_wr_addr(m2cv1_wr_addr), .mem_wr_data(m2cv1_wr_data), .mem_wr_ready(sel_m2cv1 ? mem_wr_ready : 1'b0),
                .busy(), .done(m2cv1_done), .out_valid(m2cv1_out_valid), .out_data(m2cv1_out_data),
                .final_wr_en(m2cv1_final_wr_en), .final_wr_addr(m2cv1_final_wr_addr), .final_wr_data(m2cv1_final_wr_data)
            );

            conv_layer_ddr #(
                .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
                .IN_CH(MID_CH), .OUT_CH(MID_CH), .IN_W(W), .IN_H(H), .OUT_W(W), .OUT_H(H),
                .K_SIZE(3), .STRIDE(1), .PAD(1),
                .W_NUM(MID_CH*MID_CH*3*3), .B_NUM(MID_CH),
                .W_BASE(M2CV2_W_BASE), .B_BASE(M2CV2_B_BASE),
                .MAC_LANES(MAC_LANES)
            ) u_m2cv2 (
                .clk(clk), .rst_n(rst_n), .start(start_m2cv2),
                .input_base(bank3_base), .output_base(bank4_base),
                .mem_rd_req(m2cv2_rd_req), .mem_rd_addr(m2cv2_rd_addr), .mem_rd_ready(sel_m2cv2 ? mem_rd_ready : 1'b0),
                .mem_rd_valid(sel_m2cv2 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
                .mem_wr_req(m2cv2_wr_req), .mem_wr_addr(m2cv2_wr_addr), .mem_wr_data(m2cv2_wr_data), .mem_wr_ready(sel_m2cv2 ? mem_wr_ready : 1'b0),
                .busy(), .done(m2cv2_done), .out_valid(m2cv2_out_valid), .out_data(m2cv2_out_data),
                .final_wr_en(m2cv2_final_wr_en), .final_wr_addr(m2cv2_final_wr_addr), .final_wr_data(m2cv2_final_wr_data)
            );
        end else begin : NO_M2
            assign m2cv1_done = 1'b1;
            assign m2cv1_rd_req = 1'b0;
            assign m2cv1_rd_addr = {ADDR_W{1'b0}};
            assign m2cv1_wr_req = 1'b0;
            assign m2cv1_wr_addr = {ADDR_W{1'b0}};
            assign m2cv1_wr_data = {DATA_W{1'b0}};
            assign m2cv1_final_wr_en = 1'b0;
            assign m2cv1_final_wr_addr = {ADDR_W{1'b0}};
            assign m2cv1_final_wr_data = {DATA_W{1'b0}};
            assign m2cv1_out_valid = 1'b0;
            assign m2cv1_out_data = {ACC_W{1'b0}};

            assign m2cv2_done = 1'b1;
            assign m2cv2_rd_req = 1'b0;
            assign m2cv2_rd_addr = {ADDR_W{1'b0}};
            assign m2cv2_wr_req = 1'b0;
            assign m2cv2_wr_addr = {ADDR_W{1'b0}};
            assign m2cv2_wr_data = {DATA_W{1'b0}};
            assign m2cv2_final_wr_en = 1'b0;
            assign m2cv2_final_wr_addr = {ADDR_W{1'b0}};
            assign m2cv2_final_wr_data = {DATA_W{1'b0}};
            assign m2cv2_out_valid = 1'b0;
            assign m2cv2_out_data = {ACC_W{1'b0}};
        end
    endgenerate

    conv_layer_ddr #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W),
        .IN_CH(2*MID_CH), .OUT_CH(OUT_CH), .IN_W(W), .IN_H(H), .OUT_W(W), .OUT_H(H),
        .K_SIZE(1), .STRIDE(1), .PAD(0),
        .W_NUM(OUT_CH*2*MID_CH), .B_NUM(OUT_CH),
        .W_BASE(CV3_W_BASE), .B_BASE(CV3_B_BASE),
        .MAC_LANES(MAC_LANES)
    ) u_cv3 (
        .clk(clk), .rst_n(rst_n), .start(start_cv3),
        .input_base(bank6_base), .output_base(output_base),
        .mem_rd_req(cv3_rd_req), .mem_rd_addr(cv3_rd_addr), .mem_rd_ready(sel_cv3 ? mem_rd_ready : 1'b0),
        .mem_rd_valid(sel_cv3 ? mem_rd_valid : 1'b0), .mem_rd_data(mem_rd_data),
        .mem_wr_req(cv3_wr_req), .mem_wr_addr(cv3_wr_addr), .mem_wr_data(cv3_wr_data), .mem_wr_ready(sel_cv3 ? mem_wr_ready : 1'b0),
        .busy(), .done(cv3_done), .out_valid(cv3_out_valid), .out_data(cv3_out_data),
        .final_wr_en(cv3_final_wr_en), .final_wr_addr(cv3_final_wr_addr), .final_wr_data(cv3_final_wr_data)
    );

    // ------------------------------------------------------------
    // Add / concat datapath
    // ------------------------------------------------------------
    localparam N_MID = MID_CH * H * W;
    localparam N_CAT = 2 * MID_CH * H * W;

    integer idx;
    reg [DATA_W-1:0] add_a_hold;
    reg [DATA_W-1:0] add_b_hold;
    reg [DATA_W-1:0] cat_hold;

    wire signed [7:0] add_a_s = add_a_hold;
    wire signed [7:0] add_b_s = add_b_hold;
    wire signed [15:0] add_sum = add_a_s + add_b_s;

    function [DATA_W-1:0] clip_add_u8;
        input signed [15:0] din;
        begin
            if (din < 0)
                clip_add_u8 = 8'd0;
            else if (din > 255)
                clip_add_u8 = 8'd255;
            else
                clip_add_u8 = din[7:0];
        end
    endfunction

    wire [ADDR_W-1:0] add_a_addr;
    wire [ADDR_W-1:0] add_b_addr;
    wire [ADDR_W-1:0] add_wr_addr;

    assign add_a_addr =
        ((state == S_ADD0_RA) || (state == S_ADD0_WA) || (state == S_ADD0_RB) || (state == S_ADD0_WB) || (state == S_ADD0_WR)) ? (bank1_base + idx) :
        ((state == S_ADD1_RA) || (state == S_ADD1_WA) || (state == S_ADD1_RB) || (state == S_ADD1_WB) || (state == S_ADD1_WR)) ? (bank5_base + idx) :
                                                                                                                                        (bank1_base + idx);

    assign add_b_addr = bank4_base + idx;

    assign add_wr_addr =
        ((state == S_ADD1_RA) || (state == S_ADD1_WA) || (state == S_ADD1_RB) || (state == S_ADD1_WB) || (state == S_ADD1_WR)) ? (bank1_base + idx) :
                                                                                                                                    (bank5_base + idx);

    wire [ADDR_W-1:0] final_res_base;
    assign final_res_base = (NUM_M == 1) ? bank5_base :
                            (NUM_M == 2) ? bank1_base :
                                           bank5_base;

    wire [31:0] cat_src_offset = idx - N_MID;

    wire [ADDR_W-1:0] cat_rd_addr = (idx < N_MID) ? (final_res_base + idx) : (bank2_base + cat_src_offset);
    wire [ADDR_W-1:0] cat_wr_addr = bank6_base + idx;

    wire add_rd_a_state = (state == S_ADD0_RA) || (state == S_ADD1_RA) || (state == S_ADD2_RA);
    wire add_rd_b_state = (state == S_ADD0_RB) || (state == S_ADD1_RB) || (state == S_ADD2_RB);
    wire add_wr_state   = (state == S_ADD0_WR) || (state == S_ADD1_WR) || (state == S_ADD2_WR);

    wire add_rd_req = add_rd_a_state || add_rd_b_state;
    wire [ADDR_W-1:0] add_rd_addr = add_rd_a_state ? add_a_addr : add_b_addr;

    wire cat_rd_req = (state == S_CAT_RA);
    wire cat_wr_req = (state == S_CAT_WR);

    // ------------------------------------------------------------
    // DDR mux
    // ------------------------------------------------------------
    assign mem_rd_req =
        sel_cv1   ? cv1_rd_req   :
        sel_cv2   ? cv2_rd_req   :
        sel_m0cv1 ? m0cv1_rd_req :
        sel_m0cv2 ? m0cv2_rd_req :
        sel_m1cv1 ? m1cv1_rd_req :
        sel_m1cv2 ? m1cv2_rd_req :
        sel_m2cv1 ? m2cv1_rd_req :
        sel_m2cv2 ? m2cv2_rd_req :
        sel_cv3   ? cv3_rd_req   :
        sel_add   ? add_rd_req   :
        sel_cat   ? cat_rd_req   :
                    1'b0;

    assign mem_rd_addr =
        sel_cv1   ? cv1_rd_addr   :
        sel_cv2   ? cv2_rd_addr   :
        sel_m0cv1 ? m0cv1_rd_addr :
        sel_m0cv2 ? m0cv2_rd_addr :
        sel_m1cv1 ? m1cv1_rd_addr :
        sel_m1cv2 ? m1cv2_rd_addr :
        sel_m2cv1 ? m2cv1_rd_addr :
        sel_m2cv2 ? m2cv2_rd_addr :
        sel_cv3   ? cv3_rd_addr   :
        sel_add   ? add_rd_addr   :
        sel_cat   ? cat_rd_addr   :
                    {ADDR_W{1'b0}};

    assign mem_wr_req =
        sel_cv1   ? cv1_wr_req   :
        sel_cv2   ? cv2_wr_req   :
        sel_m0cv1 ? m0cv1_wr_req :
        sel_m0cv2 ? m0cv2_wr_req :
        sel_m1cv1 ? m1cv1_wr_req :
        sel_m1cv2 ? m1cv2_wr_req :
        sel_m2cv1 ? m2cv1_wr_req :
        sel_m2cv2 ? m2cv2_wr_req :
        sel_cv3   ? cv3_wr_req   :
        sel_add   ? add_wr_state :
        sel_cat   ? cat_wr_req   :
                    1'b0;

    assign mem_wr_addr =
        sel_cv1   ? cv1_wr_addr   :
        sel_cv2   ? cv2_wr_addr   :
        sel_m0cv1 ? m0cv1_wr_addr :
        sel_m0cv2 ? m0cv2_wr_addr :
        sel_m1cv1 ? m1cv1_wr_addr :
        sel_m1cv2 ? m1cv2_wr_addr :
        sel_m2cv1 ? m2cv1_wr_addr :
        sel_m2cv2 ? m2cv2_wr_addr :
        sel_cv3   ? cv3_wr_addr   :
        sel_add   ? add_wr_addr   :
        sel_cat   ? cat_wr_addr   :
                    {ADDR_W{1'b0}};

    assign mem_wr_data =
        sel_cv1   ? cv1_wr_data   :
        sel_cv2   ? cv2_wr_data   :
        sel_m0cv1 ? m0cv1_wr_data :
        sel_m0cv2 ? m0cv2_wr_data :
        sel_m1cv1 ? m1cv1_wr_data :
        sel_m1cv2 ? m1cv2_wr_data :
        sel_m2cv1 ? m2cv1_wr_data :
        sel_m2cv2 ? m2cv2_wr_data :
        sel_cv3   ? cv3_wr_data   :
        sel_add   ? clip_add_u8(add_sum) :
        sel_cat   ? cat_hold      :
                    {DATA_W{1'b0}};

    assign final_wr_en = mem_wr_req && mem_wr_ready;
    assign final_wr_addr = mem_wr_addr;
    assign final_wr_data = mem_wr_data;

    assign out_valid =
        sel_cv1   ? cv1_out_valid   :
        sel_cv2   ? cv2_out_valid   :
        sel_m0cv1 ? m0cv1_out_valid :
        sel_m0cv2 ? m0cv2_out_valid :
        sel_m1cv1 ? m1cv1_out_valid :
        sel_m1cv2 ? m1cv2_out_valid :
        sel_m2cv1 ? m2cv1_out_valid :
        sel_m2cv2 ? m2cv2_out_valid :
        sel_cv3   ? cv3_out_valid   :
                    final_wr_en;

    assign out_data =
        sel_cv1   ? cv1_out_data   :
        sel_cv2   ? cv2_out_data   :
        sel_m0cv1 ? m0cv1_out_data :
        sel_m0cv2 ? m0cv2_out_data :
        sel_m1cv1 ? m1cv1_out_data :
        sel_m1cv2 ? m1cv2_out_data :
        sel_m2cv1 ? m2cv1_out_data :
        sel_m2cv2 ? m2cv2_out_data :
        sel_cv3   ? cv3_out_data   :
                    {ACC_W{1'b0}};

    // ------------------------------------------------------------
    // FSM
    // ------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;

            start_cv1 <= 1'b0;
            start_cv2 <= 1'b0;
            start_m0cv1 <= 1'b0;
            start_m0cv2 <= 1'b0;
            start_m1cv1 <= 1'b0;
            start_m1cv2 <= 1'b0;
            start_m2cv1 <= 1'b0;
            start_m2cv2 <= 1'b0;
            start_cv3 <= 1'b0;

            idx <= 0;
            add_a_hold <= 0;
            add_b_hold <= 0;
            cat_hold <= 0;
        end else begin
            start_cv1 <= 1'b0;
            start_cv2 <= 1'b0;
            start_m0cv1 <= 1'b0;
            start_m0cv2 <= 1'b0;
            start_m1cv1 <= 1'b0;
            start_m1cv2 <= 1'b0;
            start_m2cv1 <= 1'b0;
            start_m2cv2 <= 1'b0;
            start_cv3 <= 1'b0;

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
                        idx <= 0;
                        state <= S_ADD0_RA;
                    end
                end

                S_ADD0_RA: if (mem_rd_ready) state <= S_ADD0_WA;
                S_ADD0_WA: if (mem_rd_valid) begin add_a_hold <= mem_rd_data; state <= S_ADD0_RB; end
                S_ADD0_RB: if (mem_rd_ready) state <= S_ADD0_WB;
                S_ADD0_WB: if (mem_rd_valid) begin add_b_hold <= mem_rd_data; state <= S_ADD0_WR; end
                S_ADD0_WR: begin
                    if (mem_wr_ready) begin
                        if (idx == N_MID-1) begin
                            if (NUM_M >= 2) begin
                                start_m1cv1 <= 1'b1;
                                state <= S_M1CV1;
                            end else begin
                                idx <= 0;
                                state <= S_CAT_RA;
                            end
                        end else begin
                            idx <= idx + 1;
                            state <= S_ADD0_RA;
                        end
                    end
                end

                S_M1CV1: begin
                    if (m1cv1_done) begin
                        start_m1cv2 <= 1'b1;
                        state <= S_M1CV2;
                    end
                end

                S_M1CV2: begin
                    if (m1cv2_done) begin
                        idx <= 0;
                        state <= S_ADD1_RA;
                    end
                end

                S_ADD1_RA: if (mem_rd_ready) state <= S_ADD1_WA;
                S_ADD1_WA: if (mem_rd_valid) begin add_a_hold <= mem_rd_data; state <= S_ADD1_RB; end
                S_ADD1_RB: if (mem_rd_ready) state <= S_ADD1_WB;
                S_ADD1_WB: if (mem_rd_valid) begin add_b_hold <= mem_rd_data; state <= S_ADD1_WR; end
                S_ADD1_WR: begin
                    if (mem_wr_ready) begin
                        if (idx == N_MID-1) begin
                            if (NUM_M >= 3) begin
                                start_m2cv1 <= 1'b1;
                                state <= S_M2CV1;
                            end else begin
                                idx <= 0;
                                state <= S_CAT_RA;
                            end
                        end else begin
                            idx <= idx + 1;
                            state <= S_ADD1_RA;
                        end
                    end
                end

                S_M2CV1: begin
                    if (m2cv1_done) begin
                        start_m2cv2 <= 1'b1;
                        state <= S_M2CV2;
                    end
                end

                S_M2CV2: begin
                    if (m2cv2_done) begin
                        idx <= 0;
                        state <= S_ADD2_RA;
                    end
                end

                S_ADD2_RA: if (mem_rd_ready) state <= S_ADD2_WA;
                S_ADD2_WA: if (mem_rd_valid) begin add_a_hold <= mem_rd_data; state <= S_ADD2_RB; end
                S_ADD2_RB: if (mem_rd_ready) state <= S_ADD2_WB;
                S_ADD2_WB: if (mem_rd_valid) begin add_b_hold <= mem_rd_data; state <= S_ADD2_WR; end
                S_ADD2_WR: begin
                    if (mem_wr_ready) begin
                        if (idx == N_MID-1) begin
                            idx <= 0;
                            state <= S_CAT_RA;
                        end else begin
                            idx <= idx + 1;
                            state <= S_ADD2_RA;
                        end
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
                            start_cv3 <= 1'b1;
                            state <= S_CV3;
                        end else begin
                            idx <= idx + 1;
                            state <= S_CAT_RA;
                        end
                    end
                end

                S_CV3: begin
                    if (cv3_done)
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
