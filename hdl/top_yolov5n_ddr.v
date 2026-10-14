// FIX222B_CFG_LATCH_NEXTOP_FIX: next op must relatch cfg before start.
// FIX222_CFG_LATCH_PIPE_75: latch cfg before engine launch to target 75MHz.
// FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE: one u_yolo, runtime op range inputs.
// FIX215A_PARAMETERIZE_RANGE: RUN_START_OP/RUN_END_OP are parameters for future A/B engines.
// FIX214_RANGE_RUN_SMOKE: compile-time op range scheduler smoke test.
// FIX211_SKIP_SLOT1_SMOKE: static remap Y_BASE_L4/Y_BASE_L6 to slot1 buffers for smoke test.
// FIX210_STAGEB_TMP_REMAP_SMOKE: op34..op84 use Y_B_TMP0..Y_B_TMP5 scratch banks.
// top_yolov5n_ddr.v
// FULL shared-engine YOLOv5n scheduler (weight-in-DDR).
// Replaces structural per-layer cores with one shared conv + shared memory ops.
// NOTE: This is a major architecture rewrite; verify with synthesis/runtime before final demo.
module top_yolov5n_ddr #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32,
    parameter DET_CH = 33,
    parameter MAC_LANES = 64, // FIX180_FULL_MAC64_GROUP64 // FIX172_FULL_MAC32_REAL
    // Temporary shared shift for all conv feature-map writes.
    // Tune this first (try 9/10/11) until early feature dumps stop saturating.
    parameter CONV_OUT_SHIFT = 8
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    input  wire [7:0]              run_start_op,
    input  wire [7:0]              run_end_op,

    output wire                    mem_rd_req,
    output wire [ADDR_W-1:0]       mem_rd_addr,
    input  wire                    mem_rd_ready,
    input  wire                    mem_rd_valid,
    input  wire [DATA_W-1:0]       mem_rd_data,

    // FIX164_MAC4_WEIGHT_PACK32: 32-bit read sideband for packed MAC4 weights.
    output wire                    mem_rd32_req,
    output wire [ADDR_W-1:0]       mem_rd32_addr,
    input  wire                    mem_rd32_ready,
    input  wire                    mem_rd32_valid,
    input  wire [31:0]             mem_rd32_data,

    // FIX164_MAC8_VECTOR_PACK64: 64-bit read sideband for packed MAC8 weights.
    output wire                    mem_rd64_req,
    output wire [ADDR_W-1:0]       mem_rd64_addr,
    input  wire                    mem_rd64_ready,
    input  wire                    mem_rd64_valid,
    input  wire [63:0]             mem_rd64_data,

    output wire                    mem_wr_req,
    output wire [ADDR_W-1:0]       mem_wr_addr,
    output wire [DATA_W-1:0]       mem_wr_data,
    input  wire                    mem_wr_ready,
    output wire                    mem_wr64_req,   // FIX202_CAT2_RD64_WR64
    output wire [ADDR_W-1:0]       mem_wr64_addr,  // FIX202_CAT2_RD64_WR64
    output wire [63:0]             mem_wr64_data,  // FIX202_CAT2_RD64_WR64
    input  wire                    mem_wr64_ready, // FIX202_CAT2_RD64_WR64

    output wire                    busy,
    output wire                    done,

    output wire                    det_valid,
    output wire [1:0]              det_branch,
    output wire [31:0]             det_index,
    output wire signed [ACC_W-1:0] det_data,
    input  wire                    det_ready,

    output wire [5:0]              debug_op_id,

    // FIX164_MAC16_OP_PROFILER: raw scheduler op_id for per-op profiler.
    output wire [7:0]              prof_op_id,

    // FIX164_MAC16_DETAILED_PROFILER: conv FSM state for APB profiler.
    output wire [4:0]              conv_prof_state,

    // FIX186_DET_PROFILER: raw DET FSM state for APB profiler.
    output wire [3:0]              det_prof_state
);
    `include "yolo_weight_addr_map.vh"
    `include "yolo_ddr_addr_map.vh"

    localparam [7:0] OP_COUNT = 8'd85; // full YOLOv5n shared schedule: ops 0..84
    // FIX214_RANGE_RUN_SMOKE:
    // Compile-time op range for smoke testing scheduler split.
    // FULL:    RUN_START_OP=0,  RUN_END_OP=84
    // Stage A: RUN_START_OP=0,  RUN_END_OP=33
    // Stage B: RUN_START_OP=34, RUN_END_OP=84
    parameter [7:0] RUN_START_OP = 8'd0;  // FIX215A_PARAMETERIZE_RANGE default FULL
    parameter [7:0] RUN_END_OP   = 8'd84; // FIX215A_PARAMETERIZE_RANGE default FULL
    localparam [3:0] OP_NONE = 4'd0, OP_CONV = 4'd1, OP_UP = 4'd2, OP_CAT2 = 4'd3, OP_CAT4 = 4'd4, OP_ADD = 4'd5, OP_MAXP = 4'd6, OP_DET = 4'd7;
    // FIX164_QUAD_SCOPE_DIAG: OP_CONV with op_id <= QUAD_OP_MAX use quad-interleaved weights.
    localparam [7:0] QUAD_OP_MAX = 8'd20;
    localparam S_IDLE=3'd0, S_LATCH=3'd1, S_START=3'd2, S_WAIT=3'd3, S_DONE=3'd4; // FIX222_CFG_LATCH_PIPE_75
    reg [2:0] state; // FIX222_CFG_LATCH_PIPE_75
    reg [7:0] op_id;
    reg [7:0] run_start_lat; // FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE
    reg [7:0] run_end_lat;   // FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE
    reg [3:0] active_type;
    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);
    assign debug_op_id = (op_id == 8'd8) ? {1'b1, cfg_conv_shift} : op_id[5:0];
    assign prof_op_id = op_id; // FIX164_MAC16_OP_PROFILER

    reg [3:0] cfg_type;
    reg [ADDR_W-1:0] cfg_in0, cfg_in1, cfg_in2, cfg_in3, cfg_out, cfg_wbase, cfg_bbase;
    reg [15:0] cfg_in_ch, cfg_out_ch, cfg_in_h, cfg_in_w, cfg_out_h, cfg_out_w;
    reg [3:0] cfg_k, cfg_stride, cfg_pad;
    reg [15:0] cfg_a_ch, cfg_b_ch, cfg_ch;
    reg [1:0] cfg_branch;
    reg [4:0] cfg_conv_shift;
    reg [31:0] cfg_conv_mult;
    reg [4:0] cfg_conv_mshift;
    reg [4:0] cfg_det_shift;
    reg [31:0] cfg_det_mult;
    reg [4:0] cfg_det_mshift;
    reg [7:0]  cfg_silu_lut_id;
    reg [31:0] cfg_silu_in_mult;
    reg [4:0]  cfg_silu_in_shift;
    reg [31:0] cfg_silu_out_mult;
    reg [4:0]  cfg_silu_out_shift;
    reg [31:0] cfg_add_a_mult;
    reg [31:0] cfg_add_b_mult;
    reg [4:0]  cfg_add_mshift;
    reg [31:0] cfg_cat0_mult;
    reg [31:0] cfg_cat1_mult;
    reg [31:0] cfg_cat2_mult;
    reg [31:0] cfg_cat3_mult;
    reg [4:0]  cfg_cat_mshift;
    reg                    cfg_act_silu;

// FIX222_CFG_LATCH_PIPE_75:
// Latch cfg_* from op_id before launching each engine.
// This cuts op_id -> engine/read/write critical paths for 75MHz.
reg [ADDR_W-1:0] active_in0, active_in1, active_in2, active_in3, active_out, active_wbase, active_bbase; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [15:0] active_in_ch, active_out_ch, active_in_h, active_in_w, active_out_h, active_out_w; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [3:0] active_k, active_stride, active_pad; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [15:0] active_a_ch, active_b_ch, active_ch; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [1:0] active_branch; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [4:0] active_conv_shift; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [31:0] active_conv_mult; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [4:0] active_conv_mshift; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [4:0] active_det_shift; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [31:0] active_det_mult; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [4:0] active_det_mshift; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [7:0]  active_silu_lut_id; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [31:0] active_silu_in_mult; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [4:0]  active_silu_in_shift; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [31:0] active_silu_out_mult; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [4:0]  active_silu_out_shift; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [31:0] active_add_a_mult; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [31:0] active_add_b_mult; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [4:0]  active_add_mshift; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [31:0] active_cat0_mult; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [31:0] active_cat1_mult; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [31:0] active_cat2_mult; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [31:0] active_cat3_mult; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg [4:0]  active_cat_mshift; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
reg        active_act_silu; // FIX222C_REMOVE_ACTIVE_PRESERVE_TO_FIT_LUT
    always @(*) begin
        cfg_type=OP_NONE; cfg_in0=0; cfg_in1=0; cfg_in2=0; cfg_in3=0; cfg_out=0; cfg_wbase=0; cfg_bbase=0;
        cfg_in_ch=0; cfg_out_ch=0; cfg_in_h=0; cfg_in_w=0; cfg_out_h=0; cfg_out_w=0;
        cfg_k=0; cfg_stride=0; cfg_pad=0; cfg_a_ch=0; cfg_b_ch=0; cfg_ch=0; cfg_branch=0; cfg_conv_shift=CONV_OUT_SHIFT; cfg_conv_mult = 32'd0; cfg_conv_mshift=5'd20; cfg_det_shift=5'd0; cfg_det_mult = 32'd0; cfg_det_mshift=5'd20; cfg_silu_in_mult=32'd1048576; cfg_silu_in_shift=5'd20; cfg_silu_out_mult=32'd1048576; cfg_silu_out_shift=5'd20; cfg_silu_lut_id=8'd255; cfg_add_a_mult=32'd1048576; cfg_add_b_mult=32'd1048576; cfg_add_mshift=5'd20; cfg_cat0_mult=32'd1048576; cfg_cat1_mult=32'd1048576; cfg_cat2_mult=32'd1048576; cfg_cat3_mult=32'd1048576; cfg_cat_mshift=5'd20; cfg_act_silu=1'b0;
        case (op_id)
            8'd0: begin // L0
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd3476;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd0;
                cfg_silu_in_mult = 32'd3852540;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd363410;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_INPUT[ADDR_W-1:0];
                cfg_out = Y_BASE_L0[ADDR_W-1:0];
                cfg_wbase = YWT_l0_w[ADDR_W-1:0];
                cfg_bbase = YWT_l0_b[ADDR_W-1:0];
                cfg_in_ch = 3;
                cfg_out_ch = 16;
                cfg_in_h = 256;
                cfg_in_w = 256;
                cfg_out_h = 128;
                cfg_out_w = 128;
                cfg_k = 6;
                cfg_stride = 2;
                cfg_pad = 2;
            end
            8'd1: begin // L1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd4981;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd1;
                cfg_silu_in_mult = 32'd10684755;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd102905;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L0[ADDR_W-1:0];
                cfg_out = Y_BASE_L1[ADDR_W-1:0];
                cfg_wbase = YWT_l1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l1_b[ADDR_W-1:0];
                cfg_in_ch = 16;
                cfg_out_ch = 32;
                cfg_in_h = 128;
                cfg_in_w = 128;
                cfg_out_h = 64;
                cfg_out_w = 64;
                cfg_k = 3;
                cfg_stride = 2;
                cfg_pad = 1;
            end
            8'd2: begin // l2_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd6;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd11011;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd2;
                cfg_silu_in_mult = 32'd5201990;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd446421;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L1[ADDR_W-1:0];
                cfg_out = Y_TMP0[ADDR_W-1:0];
                cfg_wbase = YWT_l2_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l2_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 32;
                cfg_out_ch = 16;
                cfg_in_h = 64;
                cfg_in_w = 64;
                cfg_out_h = 64;
                cfg_out_w = 64;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd3: begin // l2_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd7488;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd3;
                cfg_silu_in_mult = 32'd10415134;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd105569;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L1[ADDR_W-1:0];
                cfg_out = Y_TMP1[ADDR_W-1:0];
                cfg_wbase = YWT_l2_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l2_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 32;
                cfg_out_ch = 16;
                cfg_in_h = 64;
                cfg_in_w = 64;
                cfg_out_h = 64;
                cfg_out_w = 64;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd4: begin // l2_m0_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd6;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd15162;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd4;
                cfg_silu_in_mult = 32'd3373313;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd526274;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP0[ADDR_W-1:0];
                cfg_out = Y_TMP2[ADDR_W-1:0];
                cfg_wbase = YWT_l2_m0_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l2_m0_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 16;
                cfg_out_ch = 16;
                cfg_in_h = 64;
                cfg_in_w = 64;
                cfg_out_h = 64;
                cfg_out_w = 64;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd5: begin // l2_m0_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd4661;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd5;
                cfg_silu_in_mult = 32'd3214815;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd342014;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP2[ADDR_W-1:0];
                cfg_out = Y_TMP3[ADDR_W-1:0];
                cfg_wbase = YWT_l2_m0_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l2_m0_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 16;
                cfg_out_ch = 16;
                cfg_in_h = 64;
                cfg_in_w = 64;
                cfg_out_h = 64;
                cfg_out_w = 64;
                cfg_k = 3;
                cfg_stride = 1;
                cfg_pad = 1;
            end
            8'd6: begin // l2_add0
                cfg_type = OP_ADD;
                cfg_add_a_mult = 32'd948930; // s_a=0.1468030524 -> s_out=0.1622186946
                cfg_add_b_mult = 32'd1238611; // s_b=0.1916178906 -> s_out=0.1622186946
                cfg_add_mshift = 5'd20;
                cfg_in0 = Y_TMP0[ADDR_W-1:0];
                cfg_in1 = Y_TMP3[ADDR_W-1:0];
                cfg_out = Y_TMP4[ADDR_W-1:0];
                cfg_in_h = 64;
                cfg_in_w = 64;
                cfg_ch = 16;
            end
            8'd7: begin // l2_cat
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd1048576; // s_a=0.1622186946 -> s_out=0.1622186946
                cfg_cat1_mult = 32'd4012767; // s_b=0.6207903764 -> s_out=0.1622186946
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_TMP4[ADDR_W-1:0];
                cfg_in1 = Y_TMP1[ADDR_W-1:0];
                cfg_out = Y_TMP5[ADDR_W-1:0];
                cfg_in_h = 64;
                cfg_in_w = 64;
                cfg_a_ch = 16;
                cfg_b_ch = 16;
            end
            8'd8: begin // l2_cv3
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd6;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd13657;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd6;
                cfg_silu_in_mult = 32'd1461666;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd809783;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP5[ADDR_W-1:0];
                cfg_out = Y_BASE_L2[ADDR_W-1:0];
                cfg_wbase = YWT_l2_cv3_w[ADDR_W-1:0];
                cfg_bbase = YWT_l2_cv3_b[ADDR_W-1:0];
                cfg_in_ch = 32;
                cfg_out_ch = 32;
                cfg_in_h = 64;
                cfg_in_w = 64;
                cfg_out_h = 64;
                cfg_out_w = 64;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd9: begin // L3
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd4858;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd7;
                cfg_silu_in_mult = 32'd1049475;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1313058;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L2[ADDR_W-1:0];
                cfg_out = Y_BASE_L3[ADDR_W-1:0];
                cfg_wbase = YWT_l3_w[ADDR_W-1:0];
                cfg_bbase = YWT_l3_b[ADDR_W-1:0];
                cfg_in_ch = 32;
                cfg_out_ch = 64;
                cfg_in_h = 64;
                cfg_in_w = 64;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 3;
                cfg_stride = 2;
                cfg_pad = 1;
            end
            8'd10: begin // l4_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd5292;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd8;
                cfg_silu_in_mult = 32'd659319;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd2949086;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L3[ADDR_W-1:0];
                cfg_out = Y_TMP0[ADDR_W-1:0];
                cfg_wbase = YWT_l4_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l4_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 32;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd11: begin // l4_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd6;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd11762;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd9;
                cfg_silu_in_mult = 32'd854433;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1286833;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L3[ADDR_W-1:0];
                cfg_out = Y_TMP1[ADDR_W-1:0];
                cfg_wbase = YWT_l4_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l4_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 32;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd12: begin // l4_m0_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd9398;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd10;
                cfg_silu_in_mult = 32'd633443;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1750128;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP0[ADDR_W-1:0];
                cfg_out = Y_TMP2[ADDR_W-1:0];
                cfg_wbase = YWT_l4_m0_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l4_m0_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 32;
                cfg_out_ch = 32;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd13: begin // l4_m0_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd2839;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd11;
                cfg_silu_in_mult = 32'd737215;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1491440;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP2[ADDR_W-1:0];
                cfg_out = Y_TMP3[ADDR_W-1:0];
                cfg_wbase = YWT_l4_m0_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l4_m0_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 32;
                cfg_out_ch = 32;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 3;
                cfg_stride = 1;
                cfg_pad = 1;
            end
            8'd14: begin // l4_add0
                cfg_type = OP_ADD;
                cfg_add_a_mult = 32'd545015; // s_a=0.02222248137 -> s_out=0.04275471394
                cfg_add_b_mult = 32'd1077680; // s_b=0.04394142271 -> s_out=0.04275471394
                cfg_add_mshift = 5'd20;
                cfg_in0 = Y_TMP0[ADDR_W-1:0];
                cfg_in1 = Y_TMP3[ADDR_W-1:0];
                cfg_out = Y_TMP4[ADDR_W-1:0];
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_ch = 32;
            end
            8'd15: begin // l4_m1_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd8998;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd12;
                cfg_silu_in_mult = 32'd781847;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1837445;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP4[ADDR_W-1:0];
                cfg_out = Y_TMP2[ADDR_W-1:0];
                cfg_wbase = YWT_l4_m1_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l4_m1_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 32;
                cfg_out_ch = 32;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd16: begin // l4_m1_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd2155;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd13;
                cfg_silu_in_mult = 32'd1557579;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd705911;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP2[ADDR_W-1:0];
                cfg_out = Y_TMP3[ADDR_W-1:0];
                cfg_wbase = YWT_l4_m1_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l4_m1_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 32;
                cfg_out_ch = 32;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 3;
                cfg_stride = 1;
                cfg_pad = 1;
            end
            8'd17: begin // l4_add1
                cfg_type = OP_ADD;
                cfg_add_a_mult = 32'd597117; // s_a=0.04275471394 -> s_out=0.0750800756
                cfg_add_b_mult = 32'd1296598; // s_b=0.09283894817 -> s_out=0.0750800756
                cfg_add_mshift = 5'd20;
                cfg_in0 = Y_TMP4[ADDR_W-1:0];
                cfg_in1 = Y_TMP3[ADDR_W-1:0];
                cfg_out = Y_TMP0[ADDR_W-1:0];
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_ch = 32;
            end
            8'd18: begin // l4_cat
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd1048576; // s_a=0.0750800756 -> s_out=0.0750800756
                cfg_cat1_mult = 32'd711268; // s_b=0.05092814588 -> s_out=0.0750800756
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_TMP0[ADDR_W-1:0];
                cfg_in1 = Y_TMP1[ADDR_W-1:0];
                cfg_out = Y_TMP5[ADDR_W-1:0];
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_a_ch = 32;
                cfg_b_ch = 32;
            end
            8'd19: begin // l4_cv3
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd8564;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd14;
                cfg_silu_in_mult = 32'd834308;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1378081;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP5[ADDR_W-1:0];
                cfg_out = Y_L4_SLOT1[ADDR_W-1:0];
                cfg_wbase = YWT_l4_cv3_w[ADDR_W-1:0];
                cfg_bbase = YWT_l4_cv3_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd20: begin // L5
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd4470;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd15;
                cfg_silu_in_mult = 32'd904675;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1622147;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_L4_SLOT1[ADDR_W-1:0];
                cfg_out = Y_BASE_L5[ADDR_W-1:0];
                cfg_wbase = YWT_l5_w[ADDR_W-1:0];
                cfg_bbase = YWT_l5_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 128;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 3;
                cfg_stride = 2;
                cfg_pad = 1;
            end
            8'd21: begin // l6_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd5791;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd16;
                cfg_silu_in_mult = 32'd698124;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd3050462;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L5[ADDR_W-1:0];
                cfg_out = Y_TMP0[ADDR_W-1:0];
                cfg_wbase = YWT_l6_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l6_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd22: begin // l6_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd7273;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd17;
                cfg_silu_in_mult = 32'd976247;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1126264;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L5[ADDR_W-1:0];
                cfg_out = Y_TMP1[ADDR_W-1:0];
                cfg_wbase = YWT_l6_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l6_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd23: begin // l6_m0_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd5988;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd18;
                cfg_silu_in_mult = 32'd1033662;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1250253;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP0[ADDR_W-1:0];
                cfg_out = Y_TMP2[ADDR_W-1:0];
                cfg_wbase = YWT_l6_m0_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l6_m0_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd24: begin // l6_m0_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd3268;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd19;
                cfg_silu_in_mult = 32'd670392;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1640103;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP2[ADDR_W-1:0];
                cfg_out = Y_TMP3[ADDR_W-1:0];
                cfg_wbase = YWT_l6_m0_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l6_m0_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 3;
                cfg_stride = 1;
                cfg_pad = 1;
            end
            8'd25: begin // l6_add0
                cfg_type = OP_ADD;
                cfg_add_a_mult = 32'd938141; // s_a=0.02148395636 -> s_out=0.02401297111
                cfg_add_b_mult = 32'd1744868; // s_b=0.03995845449 -> s_out=0.02401297111
                cfg_add_mshift = 5'd20;
                cfg_in0 = Y_TMP0[ADDR_W-1:0];
                cfg_in1 = Y_TMP3[ADDR_W-1:0];
                cfg_out = Y_TMP4[ADDR_W-1:0];
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_ch = 64;
            end
            8'd26: begin // l6_m1_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd5108;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd20;
                cfg_silu_in_mult = 32'd1042055;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1264263;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP4[ADDR_W-1:0];
                cfg_out = Y_TMP2[ADDR_W-1:0];
                cfg_wbase = YWT_l6_m1_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l6_m1_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd27: begin // l6_m1_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd5352;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd21;
                cfg_silu_in_mult = 32'd849270;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1294655;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP2[ADDR_W-1:0];
                cfg_out = Y_TMP3[ADDR_W-1:0];
                cfg_wbase = YWT_l6_m1_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l6_m1_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 3;
                cfg_stride = 1;
                cfg_pad = 1;
            end
            8'd28: begin // l6_add1
                cfg_type = OP_ADD;
                cfg_add_a_mult = 32'd601915; // s_a=0.02401297111 -> s_out=0.04183219174
                cfg_add_b_mult = 32'd1268864; // s_b=0.05062041696 -> s_out=0.04183219174
                cfg_add_mshift = 5'd20;
                cfg_in0 = Y_TMP4[ADDR_W-1:0];
                cfg_in1 = Y_TMP3[ADDR_W-1:0];
                cfg_out = Y_TMP0[ADDR_W-1:0];
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_ch = 64;
            end
            8'd29: begin // l6_m2_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd4379;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd22;
                cfg_silu_in_mult = 32'd1317201;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1438623;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP0[ADDR_W-1:0];
                cfg_out = Y_TMP2[ADDR_W-1:0];
                cfg_wbase = YWT_l6_m2_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l6_m2_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd30: begin // l6_m2_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd4374;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd23;
                cfg_silu_in_mult = 32'd1111924;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd988837;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP2[ADDR_W-1:0];
                cfg_out = Y_TMP3[ADDR_W-1:0];
                cfg_wbase = YWT_l6_m2_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l6_m2_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 3;
                cfg_stride = 1;
                cfg_pad = 1;
            end
            8'd31: begin // l6_add2
                cfg_type = OP_ADD;
                cfg_add_a_mult = 32'd726191; // s_a=0.04183219174 -> s_out=0.06040319308
                cfg_add_b_mult = 32'd1150523; // s_b=0.06627584442 -> s_out=0.06040319308
                cfg_add_mshift = 5'd20;
                cfg_in0 = Y_TMP0[ADDR_W-1:0];
                cfg_in1 = Y_TMP3[ADDR_W-1:0];
                cfg_out = Y_TMP4[ADDR_W-1:0];
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_ch = 64;
            end
            8'd32: begin // l6_cat
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd1048576; // s_a=0.06040319308 -> s_out=0.06040319308
                cfg_cat1_mult = 32'd1010136; // s_b=0.05818883265 -> s_out=0.06040319308
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_TMP4[ADDR_W-1:0];
                cfg_in1 = Y_TMP1[ADDR_W-1:0];
                cfg_out = Y_TMP5[ADDR_W-1:0];
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_a_ch = 64;
                cfg_b_ch = 64;
            end
            8'd33: begin // l6_cv3
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd6806;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd24;
                cfg_silu_in_mult = 32'd837072;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1456304;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_TMP5[ADDR_W-1:0];
                cfg_out = Y_L6_SLOT1[ADDR_W-1:0];
                cfg_wbase = YWT_l6_cv3_w[ADDR_W-1:0];
                cfg_bbase = YWT_l6_cv3_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 128;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd34: begin // L7
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd9;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd2615;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd25;
                cfg_silu_in_mult = 32'd944385;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1668896;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_L6_SLOT1[ADDR_W-1:0];
                cfg_out = Y_BASE_L7[ADDR_W-1:0];
                cfg_wbase = YWT_l7_w[ADDR_W-1:0];
                cfg_bbase = YWT_l7_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 256;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 3;
                cfg_stride = 2;
                cfg_pad = 1;
            end
            8'd35: begin // l8_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd3406;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd26;
                cfg_silu_in_mult = 32'd671401;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd3271752;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L7[ADDR_W-1:0];
                cfg_out = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l8_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l8_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 256;
                cfg_out_ch = 128;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd36: begin // l8_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd4269;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd27;
                cfg_silu_in_mult = 32'd983778;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1117642;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L7[ADDR_W-1:0];
                cfg_out = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l8_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l8_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 256;
                cfg_out_ch = 128;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd37: begin // l8_m0_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd6456;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd28;
                cfg_silu_in_mult = 32'd1260412;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd872406;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l8_m0_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l8_m0_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 128;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd38: begin // l8_m0_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd2573;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd29;
                cfg_silu_in_mult = 32'd1888814;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd582117;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l8_m0_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l8_m0_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 128;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 3;
                cfg_stride = 1;
                cfg_pad = 1;
            end
            8'd39: begin // l8_add0
                cfg_type = OP_ADD;
                cfg_add_a_mult = 32'd313513; // s_a=0.02003086083 -> s_out=0.06699523776
                cfg_add_b_mult = 32'd1762079; // s_b=0.1125820941 -> s_out=0.06699523776
                cfg_add_mshift = 5'd20;
                cfg_in0 = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in1 = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP4[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_ch = 128;
            end
            8'd40: begin // l8_cat
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd1048576; // s_a=0.06699523776 -> s_out=0.06699523776
                cfg_cat1_mult = 32'd917769; // s_b=0.05863773917 -> s_out=0.06699523776
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_B_TMP4[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in1 = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP5[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_a_ch = 128;
                cfg_b_ch = 128;
            end
            8'd41: begin // l8_cv3
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd4063;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd30;
                cfg_silu_in_mult = 32'd1456388;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1010109;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP5[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_BASE_L8[ADDR_W-1:0];
                cfg_wbase = YWT_l8_cv3_w[ADDR_W-1:0];
                cfg_bbase = YWT_l8_cv3_b[ADDR_W-1:0];
                cfg_in_ch = 256;
                cfg_out_ch = 256;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd42: begin // l9_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd5426;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd31;
                cfg_silu_in_mult = 32'd826213;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1330785;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L8[ADDR_W-1:0];
                cfg_out = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l9_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l9_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 256;
                cfg_out_ch = 128;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd43: begin // l9_p1
                cfg_type = OP_MAXP;
                cfg_in0 = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_ch = 128;
            end
            8'd44: begin // l9_p2
                cfg_type = OP_MAXP;
                cfg_in0 = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_ch = 128;
            end
            8'd45: begin // l9_p3
                cfg_type = OP_MAXP;
                cfg_in0 = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_ch = 128;
            end
            8'd46: begin // l9_cat4
                cfg_type = OP_CAT4;
                cfg_cat0_mult = 32'd1050592; // s_0=0.0492461197 -> s_out=0.04915164212
                cfg_cat1_mult = 32'd1050592; // s_1=0.0492461197 -> s_out=0.04915164212
                cfg_cat2_mult = 32'd1050592; // s_2=0.0492461197 -> s_out=0.04915164212
                cfg_cat3_mult = 32'd1050592; // s_3=0.0492461197 -> s_out=0.04915164212
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in1 = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in2 = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in3 = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP4[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_ch = 128;
            end
            8'd47: begin // l9_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd2656;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd32;
                cfg_silu_in_mult = 32'd895565;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd2216057;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP4[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_BASE_L9[ADDR_W-1:0];
                cfg_wbase = YWT_l9_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l9_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 512;
                cfg_out_ch = 256;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd48: begin // L10
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd6788;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd33;
                cfg_silu_in_mult = 32'd955619;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1150575;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L9[ADDR_W-1:0];
                cfg_out = Y_BASE_L10[ADDR_W-1:0];
                cfg_wbase = YWT_l10_w[ADDR_W-1:0];
                cfg_bbase = YWT_l10_b[ADDR_W-1:0];
                cfg_in_ch = 256;
                cfg_out_ch = 128;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd49: begin // L11
                cfg_type = OP_UP;
                cfg_in0 = Y_BASE_L10[ADDR_W-1:0];
                cfg_out = Y_BASE_L11[ADDR_W-1:0];
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_ch = 128;
            end
            8'd50: begin // L12
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd1327202; // s_a=0.05695933244 -> s_out=0.0450015819
                cfg_cat1_mult = 32'd1048576; // s_b=0.0450015819 -> s_out=0.0450015819
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_BASE_L11[ADDR_W-1:0];
                cfg_in1 = Y_L6_SLOT1[ADDR_W-1:0];
                cfg_out = Y_BASE_L12[ADDR_W-1:0];
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_a_ch = 128;
                cfg_b_ch = 128;
            end
            8'd51: begin // l13_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd6011;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd34;
                cfg_silu_in_mult = 32'd848440;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1765777;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L12[ADDR_W-1:0];
                cfg_out = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l13_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l13_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 256;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd52: begin // l13_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd4938;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd35;
                cfg_silu_in_mult = 32'd721612;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1523688;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L12[ADDR_W-1:0];
                cfg_out = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l13_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l13_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 256;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd53: begin // l13_m0_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd10443;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd36;
                cfg_silu_in_mult = 32'd691447;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1598639;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l13_m0_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l13_m0_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd54: begin // l13_m0_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd3640;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd37;
                cfg_silu_in_mult = 32'd793131;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1386293;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l13_m0_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l13_m0_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 3;
                cfg_stride = 1;
                cfg_pad = 1;
            end
            8'd55: begin // l13_cat
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd1122366; // s_a=0.0472742779 -> s_out=0.04416623078
                cfg_cat1_mult = 32'd1021160; // s_b=0.04301144382 -> s_out=0.04416623078
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in1 = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP5[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_a_ch = 64;
                cfg_b_ch = 64;
            end
            8'd56: begin // l13_cv3
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd6209;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd38;
                cfg_silu_in_mult = 32'd1197459;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1856664;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP5[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_BASE_L13[ADDR_W-1:0];
                cfg_wbase = YWT_l13_cv3_w[ADDR_W-1:0];
                cfg_bbase = YWT_l13_cv3_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 128;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd57: begin // L14
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd8128;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd39;
                cfg_silu_in_mult = 32'd725922;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1514642;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L13[ADDR_W-1:0];
                cfg_out = Y_BASE_L14[ADDR_W-1:0];
                cfg_wbase = YWT_l14_w[ADDR_W-1:0];
                cfg_bbase = YWT_l14_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd58: begin // L15
                cfg_type = OP_UP;
                cfg_in0 = Y_BASE_L14[ADDR_W-1:0];
                cfg_out = Y_BASE_L15[ADDR_W-1:0];
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_ch = 64;
            end
            8'd59: begin // L16
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd954036; // s_a=0.04326832013 -> s_out=0.04755598354
                cfg_cat1_mult = 32'd1048576; // s_b=0.04755598354 -> s_out=0.04755598354
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_BASE_L15[ADDR_W-1:0];
                cfg_in1 = Y_L4_SLOT1[ADDR_W-1:0];
                cfg_out = Y_BASE_L16[ADDR_W-1:0];
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_a_ch = 64;
                cfg_b_ch = 64;
            end
            8'd60: begin // l17_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd8484;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd40;
                cfg_silu_in_mult = 32'd726193;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd3030813;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L16[ADDR_W-1:0];
                cfg_out = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l17_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l17_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 32;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd61: begin // l17_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd6;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd12763;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd41;
                cfg_silu_in_mult = 32'd866848;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1268402;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L16[ADDR_W-1:0];
                cfg_out = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l17_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l17_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 32;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd62: begin // l17_m0_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd7995;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd42;
                cfg_silu_in_mult = 32'd594016;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd2286885;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l17_m0_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l17_m0_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 32;
                cfg_out_ch = 32;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd63: begin // l17_m0_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd2241;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd43;
                cfg_silu_in_mult = 32'd1380816;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd796276;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l17_m0_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l17_m0_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 32;
                cfg_out_ch = 32;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 3;
                cfg_stride = 1;
                cfg_pad = 1;
            end
            8'd64: begin // l17_cat
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd1624070; // s_a=0.08230306971 -> s_out=0.05313874793
                cfg_cat1_mult = 32'd1019557; // s_b=0.05166815961 -> s_out=0.05313874793
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in1 = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP5[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_a_ch = 32;
                cfg_b_ch = 32;
            end
            8'd65: begin // l17_cv3
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd5921;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd44;
                cfg_silu_in_mult = 32'd3380143;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd413045;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP5[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_BASE_L17[ADDR_W-1:0];
                cfg_wbase = YWT_l17_cv3_w[ADDR_W-1:0];
                cfg_bbase = YWT_l17_cv3_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 32;
                cfg_out_w = 32;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd66: begin // L18
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd5542;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd45;
                cfg_silu_in_mult = 32'd956415;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1149618;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L17[ADDR_W-1:0];
                cfg_out = Y_BASE_L18[ADDR_W-1:0];
                cfg_wbase = YWT_l18_w[ADDR_W-1:0];
                cfg_bbase = YWT_l18_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 32;
                cfg_in_w = 32;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 3;
                cfg_stride = 2;
                cfg_pad = 1;
            end
            8'd67: begin // L19
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd1136674; // s_a=0.05700676835 -> s_out=0.05258847409
                cfg_cat1_mult = 32'd862739; // s_b=0.04326832013 -> s_out=0.05258847409
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_BASE_L18[ADDR_W-1:0];
                cfg_in1 = Y_BASE_L14[ADDR_W-1:0];
                cfg_out = Y_BASE_L19[ADDR_W-1:0];
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_a_ch = 64;
                cfg_b_ch = 64;
            end
            8'd68: begin // l20_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd8735;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd47;
                cfg_silu_in_mult = 32'd765304;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1492456;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L19[ADDR_W-1:0];
                cfg_out = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l20_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l20_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd69: begin // l20_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd6746;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd48;
                cfg_silu_in_mult = 32'd926035;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1187333;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L19[ADDR_W-1:0];
                cfg_out = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l20_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l20_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd70: begin // l20_m0_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd6862;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd49;
                cfg_silu_in_mult = 32'd1078511;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1647942;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l20_m0_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l20_m0_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd71: begin // l20_m0_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd4614;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd50;
                cfg_silu_in_mult = 32'd1263512;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd870202;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l20_m0_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l20_m0_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = 64;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 3;
                cfg_stride = 1;
                cfg_pad = 1;
            end
            8'd72: begin // l20_cat
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd1253824; // s_a=0.07531121021 -> s_out=0.06298293091
                cfg_cat1_mult = 32'd918934; // s_b=0.05519596986 -> s_out=0.06298293091
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in1 = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP5[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_a_ch = 64;
                cfg_b_ch = 64;
            end
            8'd73: begin // l20_cv3
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd6;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd8857;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd51;
                cfg_silu_in_mult = 32'd2116994;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd530021;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP5[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_BASE_L20[ADDR_W-1:0];
                cfg_wbase = YWT_l20_cv3_w[ADDR_W-1:0];
                cfg_bbase = YWT_l20_cv3_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 128;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 16;
                cfg_out_w = 16;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd74: begin // L21
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd8;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd5367;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd52;
                cfg_silu_in_mult = 32'd1239812;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd886837;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L20[ADDR_W-1:0];
                cfg_out = Y_BASE_L21[ADDR_W-1:0];
                cfg_wbase = YWT_l21_w[ADDR_W-1:0];
                cfg_bbase = YWT_l21_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 128;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 3;
                cfg_stride = 2;
                cfg_pad = 1;
            end
            8'd75: begin // L22
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd1048664; // s_a=0.07389854822 -> s_out=0.07389234558
                cfg_cat1_mult = 32'd808287; // s_b=0.05695933244 -> s_out=0.07389234558
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_BASE_L21[ADDR_W-1:0];
                cfg_in1 = Y_BASE_L10[ADDR_W-1:0];
                cfg_out = Y_BASE_L22[ADDR_W-1:0];
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_a_ch = 128;
                cfg_b_ch = 128;
            end
            8'd76: begin // l23_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd7719;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd54;
                cfg_silu_in_mult = 32'd1113546;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd987612;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L22[ADDR_W-1:0];
                cfg_out = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l23_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l23_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 256;
                cfg_out_ch = 128;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd77: begin // l23_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd5365;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd55;
                cfg_silu_in_mult = 32'd1271846;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd864500;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_BASE_L22[ADDR_W-1:0];
                cfg_out = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l23_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l23_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 256;
                cfg_out_ch = 128;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd78: begin // l23_m0_cv1
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd11376;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd56;
                cfg_silu_in_mult = 32'd1089970;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1519650;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP0[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l23_m0_cv1_w[ADDR_W-1:0];
                cfg_bbase = YWT_l23_m0_cv1_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 128;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd79: begin // l23_m0_cv2
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd9;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd2561;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd57;
                cfg_silu_in_mult = 32'd1720716;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd638985;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP2[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_wbase = YWT_l23_m0_cv2_w[ADDR_W-1:0];
                cfg_bbase = YWT_l23_m0_cv2_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = 128;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 3;
                cfg_stride = 1;
                cfg_pad = 1;
            end
            8'd80: begin // l23_cat
                cfg_type = OP_CAT2;
                cfg_cat0_mult = 32'd1733376; // s_a=0.102562649 -> s_out=0.06204352792
                cfg_cat1_mult = 32'd1281204; // s_b=0.07580795438 -> s_out=0.06204352792
                cfg_cat_mshift = 5'd20;
                cfg_in0 = Y_B_TMP3[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in1 = Y_B_TMP1[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_B_TMP5[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_a_ch = 128;
                cfg_b_ch = 128;
            end
            8'd81: begin // l23_cv3
                cfg_type = OP_CONV;
                cfg_conv_shift = 5'd7;
                cfg_conv_mshift = 5'd20;
                cfg_conv_mult = 32'd6661;
                cfg_act_silu = 1'b1;
                cfg_silu_lut_id = 8'd58;
                cfg_silu_in_mult = 32'd1591571;
                cfg_silu_in_shift = 5'd20;
                cfg_silu_out_mult = 32'd1064444;
                cfg_silu_out_shift = 5'd20;
                cfg_in0 = Y_B_TMP5[ADDR_W-1:0]; // FIX210_STAGEB_TMP_REMAP_SMOKE
                cfg_out = Y_BASE_L23[ADDR_W-1:0];
                cfg_wbase = YWT_l23_cv3_w[ADDR_W-1:0];
                cfg_bbase = YWT_l23_cv3_b[ADDR_W-1:0];
                cfg_in_ch = 256;
                cfg_out_ch = 256;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_out_h = 8;
                cfg_out_w = 8;
                cfg_k = 1;
                cfg_stride = 1;
            end
            8'd82: begin // DET_S
                cfg_type = OP_DET;
                cfg_det_shift = 5'd8;
                cfg_det_mshift = 5'd20;
                cfg_det_mult = 32'd7178;
                cfg_in0 = Y_BASE_L17[ADDR_W-1:0];
                cfg_wbase = YWT_l24_s_w[ADDR_W-1:0];
                cfg_bbase = YWT_l24_s_b[ADDR_W-1:0];
                cfg_in_ch = 64;
                cfg_out_ch = DET_CH;
                cfg_in_h = 32;
                cfg_in_w = 32;
            end
            8'd83: begin // DET_M
                cfg_type = OP_DET;
                cfg_det_shift = 5'd8;
                cfg_det_mshift = 5'd20;
                cfg_det_mult = 32'd7126;
                cfg_in0 = Y_BASE_L20[ADDR_W-1:0];
                cfg_wbase = YWT_l24_m_w[ADDR_W-1:0];
                cfg_bbase = YWT_l24_m_b[ADDR_W-1:0];
                cfg_in_ch = 128;
                cfg_out_ch = DET_CH;
                cfg_in_h = 16;
                cfg_in_w = 16;
                cfg_branch = 1;
            end
            8'd84: begin // DET_L
                cfg_type = OP_DET;
                cfg_det_shift = 5'd9;
                cfg_det_mshift = 5'd20;
                cfg_det_mult = 32'd5059;
                cfg_in0 = Y_BASE_L23[ADDR_W-1:0];
                cfg_wbase = YWT_l24_l_w[ADDR_W-1:0];
                cfg_bbase = YWT_l24_l_b[ADDR_W-1:0];
                cfg_in_ch = 256;
                cfg_out_ch = DET_CH;
                cfg_in_h = 8;
                cfg_in_w = 8;
                cfg_branch = 2;
            end
            default: begin cfg_type = OP_NONE; end
        endcase
    end

    wire conv_start = (state == S_START) && (active_type == OP_CONV);
    wire conv_quad_weight_en = (active_type == OP_CONV); // FIX222B_CFG_LATCH_NEXTOP_FIX // FIX164_MAC16_VECTOR_MULTIBEAT: enable group16/multibeat weights
    wire up_start = (state == S_START) && (active_type == OP_UP);
    wire cat2_start = (state == S_START) && (active_type == OP_CAT2);
    wire cat4_start = (state == S_START) && (active_type == OP_CAT4);
    wire add_start = (state == S_START) && (active_type == OP_ADD);
    wire maxp_start = (state == S_START) && (active_type == OP_MAXP);
    wire det_start = (state == S_START) && (active_type == OP_DET);

    // Shared conv engine
    wire [4:0] CONV_OUT_SHIFT_W = active_conv_shift;
    wire conv_rd_req, conv_wr_req, conv_done;
    wire [ADDR_W-1:0] conv_rd_addr, conv_wr_addr;
    wire [DATA_W-1:0] conv_wr_data;
    wire conv_rd32_req;                 // FIX164_MAC4_WEIGHT_PACK32
    wire [ADDR_W-1:0] conv_rd32_addr;   // FIX164_MAC4_WEIGHT_PACK32
    wire conv_rd64_req;                 // FIX164_MAC8_VECTOR_PACK64
    wire [ADDR_W-1:0] conv_rd64_addr;   // FIX164_MAC8_VECTOR_PACK64
    wire conv_wr64_req;                 // FIX_CONV_WR64_PACKER
    wire [ADDR_W-1:0] conv_wr64_addr;   // FIX_CONV_WR64_PACKER
    wire [63:0] conv_wr64_data;         // FIX_CONV_WR64_PACKER
    wire [4:0] conv_prof_state_w;       // FIX164_MAC16_DETAILED_PROFILER
    conv2d_ddr_core #(.DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W), .MAC_LANES(MAC_LANES)) u_shared_conv (
        .clk(clk), .rst_n(rst_n), .start(conv_start),
        .in_ch(active_in_ch), .out_ch(active_out_ch), .in_w(active_in_w), .in_h(active_in_h), .out_w(active_out_w), .out_h(active_out_h),
        .k_size(active_k), .stride(active_stride), .pad(active_pad),
        .input_base(active_in0), .output_base(active_out), .weight_base(active_wbase), .bias_base(active_bbase), .quad_weight_en(conv_quad_weight_en), .out_shift(CONV_OUT_SHIFT_W), .out_mult(active_conv_mult), .out_mult_shift(active_conv_mshift), .act_silu_en(active_act_silu), .silu_lut_id(active_silu_lut_id), .silu_in_mult(active_silu_in_mult), .silu_in_shift(active_silu_in_shift), .silu_out_mult(active_silu_out_mult), .silu_out_shift(active_silu_out_shift),
        .mem_rd_req(conv_rd_req), .mem_rd_addr(conv_rd_addr), .mem_rd_ready((active_type==OP_CONV)?mem_rd_ready:1'b0), .mem_rd_valid((active_type==OP_CONV)?mem_rd_valid:1'b0), .mem_rd_data(mem_rd_data),
        .mem_rd32_req(conv_rd32_req), .mem_rd32_addr(conv_rd32_addr), .mem_rd32_ready((active_type==OP_CONV)?mem_rd32_ready:1'b0), .mem_rd32_valid((active_type==OP_CONV)?mem_rd32_valid:1'b0), .mem_rd32_data(mem_rd32_data), // FIX164_MAC4_WEIGHT_PACK32
        .mem_rd64_req(conv_rd64_req), .mem_rd64_addr(conv_rd64_addr), .mem_rd64_ready((active_type==OP_CONV)?mem_rd64_ready:1'b0), .mem_rd64_valid((active_type==OP_CONV)?mem_rd64_valid:1'b0), .mem_rd64_data(mem_rd64_data), // FIX164_MAC8_VECTOR_PACK64
        .mem_wr_req(conv_wr_req), .mem_wr_addr(conv_wr_addr), .mem_wr_data(conv_wr_data), .mem_wr_ready((active_type==OP_CONV)?mem_wr_ready:1'b0),
        .mem_wr64_req(conv_wr64_req), .mem_wr64_addr(conv_wr64_addr), .mem_wr64_data(conv_wr64_data), .mem_wr64_ready((active_type==OP_CONV)?mem_wr64_ready:1'b0), // FIX_CONV_WR64_PACKER
        .busy(), .done(conv_done), .out_valid(), .out_data(), .final_wr_en(), .final_wr_addr(), .final_wr_data(),
        .prof_state(conv_prof_state_w) // FIX164_MAC16_DETAILED_PROFILER
    );

    assign conv_prof_state = (active_type == OP_CONV) ? conv_prof_state_w : 5'h1f; // FIX165_PROF_5BIT_WCACHE

    wire up_rd_req, up_wr_req, up_done;
    wire [ADDR_W-1:0] up_rd_addr, up_wr_addr;
    wire [DATA_W-1:0] up_wr_data;
    wire up_rd64_req, up_wr64_req;                 // FIX204_COMBO_NONCONV64
    wire [ADDR_W-1:0] up_rd64_addr, up_wr64_addr;  // FIX204_COMBO_NONCONV64
    wire [63:0] up_wr64_data;                      // FIX204_COMBO_NONCONV64
    shared_upsample_ddr_engine #(.DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W)) u_shared_up (
        .clk(clk), .rst_n(rst_n), .start(up_start), .ch_n(active_ch), .in_h(active_in_h), .in_w(active_in_w),
        .input_base(active_in0), .output_base(active_out),
        .mem_rd_req(up_rd_req), .mem_rd_addr(up_rd_addr), .mem_rd_ready((active_type==OP_UP)?mem_rd_ready:1'b0), .mem_rd_valid((active_type==OP_UP)?mem_rd_valid:1'b0), .mem_rd_data(mem_rd_data),
        .mem_rd64_req(up_rd64_req), .mem_rd64_addr(up_rd64_addr), .mem_rd64_ready((active_type==OP_UP)?mem_rd64_ready:1'b0), .mem_rd64_valid((active_type==OP_UP)?mem_rd64_valid:1'b0), .mem_rd64_data(mem_rd64_data), // FIX204_COMBO_NONCONV64
        .mem_wr_req(up_wr_req), .mem_wr_addr(up_wr_addr), .mem_wr_data(up_wr_data), .mem_wr_ready((active_type==OP_UP)?mem_wr_ready:1'b0),
        .mem_wr64_req(up_wr64_req), .mem_wr64_addr(up_wr64_addr), .mem_wr64_data(up_wr64_data), .mem_wr64_ready((active_type==OP_UP)?mem_wr64_ready:1'b0), // FIX204_COMBO_NONCONV64
        .busy(), .done(up_done)
    );

    wire cat2_rd_req, cat2_wr_req, cat2_done;
    wire [ADDR_W-1:0] cat2_rd_addr, cat2_wr_addr;
    wire [DATA_W-1:0] cat2_wr_data;
    wire cat2_rd64_req, cat2_wr64_req;                 // FIX202_CAT2_RD64_WR64
    wire [ADDR_W-1:0] cat2_rd64_addr, cat2_wr64_addr;  // FIX202_CAT2_RD64_WR64
    wire [63:0] cat2_wr64_data;                        // FIX202_CAT2_RD64_WR64
    shared_concat2_ddr_engine #(.DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W)) u_shared_cat2 (
        .clk(clk), .rst_n(rst_n), .start(cat2_start), .a_ch(active_a_ch), .b_ch(active_b_ch), .h(active_in_h), .w(active_in_w),
        .input_a_base(active_in0), .input_b_base(active_in1), .output_base(active_out),
        .a_mult(active_cat0_mult), .b_mult(active_cat1_mult), .mult_shift(active_cat_mshift),
        .mem_rd_req(cat2_rd_req), .mem_rd_addr(cat2_rd_addr), .mem_rd_ready((active_type==OP_CAT2)?mem_rd_ready:1'b0), .mem_rd_valid((active_type==OP_CAT2)?mem_rd_valid:1'b0), .mem_rd_data(mem_rd_data),
        .mem_rd64_req(cat2_rd64_req), .mem_rd64_addr(cat2_rd64_addr), .mem_rd64_ready((active_type==OP_CAT2)?mem_rd64_ready:1'b0), .mem_rd64_valid((active_type==OP_CAT2)?mem_rd64_valid:1'b0), .mem_rd64_data(mem_rd64_data), // FIX202_CAT2_RD64_WR64
        .mem_wr_req(cat2_wr_req), .mem_wr_addr(cat2_wr_addr), .mem_wr_data(cat2_wr_data), .mem_wr_ready((active_type==OP_CAT2)?mem_wr_ready:1'b0),
        .mem_wr64_req(cat2_wr64_req), .mem_wr64_addr(cat2_wr64_addr), .mem_wr64_data(cat2_wr64_data), .mem_wr64_ready((active_type==OP_CAT2)?mem_wr64_ready:1'b0), // FIX202_CAT2_RD64_WR64
        .busy(), .done(cat2_done)
    );

    wire cat4_rd_req, cat4_wr_req, cat4_done;
    wire [ADDR_W-1:0] cat4_rd_addr, cat4_wr_addr;
    wire [DATA_W-1:0] cat4_wr_data;
    wire cat4_rd64_req, cat4_wr64_req;                 // FIX204_COMBO_NONCONV64
    wire [ADDR_W-1:0] cat4_rd64_addr, cat4_wr64_addr;  // FIX204_COMBO_NONCONV64
    wire [63:0] cat4_wr64_data;                        // FIX204_COMBO_NONCONV64
    shared_concat4_ddr_engine #(.DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W)) u_shared_cat4 (
        .clk(clk), .rst_n(rst_n), .start(cat4_start), .ch(active_ch), .h(active_in_h), .w(active_in_w),
        .base0(active_in0), .base1(active_in1), .base2(active_in2), .base3(active_in3), .output_base(active_out),
        .m0(active_cat0_mult), .m1(active_cat1_mult), .m2(active_cat2_mult), .m3(active_cat3_mult), .mult_shift(active_cat_mshift),
        .mem_rd_req(cat4_rd_req), .mem_rd_addr(cat4_rd_addr), .mem_rd_ready((active_type==OP_CAT4)?mem_rd_ready:1'b0), .mem_rd_valid((active_type==OP_CAT4)?mem_rd_valid:1'b0), .mem_rd_data(mem_rd_data),
        .mem_rd64_req(cat4_rd64_req), .mem_rd64_addr(cat4_rd64_addr), .mem_rd64_ready((active_type==OP_CAT4)?mem_rd64_ready:1'b0), .mem_rd64_valid((active_type==OP_CAT4)?mem_rd64_valid:1'b0), .mem_rd64_data(mem_rd64_data), // FIX204_COMBO_NONCONV64
        .mem_wr_req(cat4_wr_req), .mem_wr_addr(cat4_wr_addr), .mem_wr_data(cat4_wr_data), .mem_wr_ready((active_type==OP_CAT4)?mem_wr_ready:1'b0),
        .mem_wr64_req(cat4_wr64_req), .mem_wr64_addr(cat4_wr64_addr), .mem_wr64_data(cat4_wr64_data), .mem_wr64_ready((active_type==OP_CAT4)?mem_wr64_ready:1'b0), // FIX204_COMBO_NONCONV64
        .busy(), .done(cat4_done)
    );

    wire add_rd_req, add_wr_req, add_done;
    wire [ADDR_W-1:0] add_rd_addr, add_wr_addr;
    wire [DATA_W-1:0] add_wr_data;
    wire add_rd64_req, add_wr64_req;                 // FIX204_COMBO_NONCONV64
    wire [ADDR_W-1:0] add_rd64_addr, add_wr64_addr;  // FIX204_COMBO_NONCONV64
    wire [63:0] add_wr64_data;                       // FIX204_COMBO_NONCONV64
    shared_add_ddr_engine #(.DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W)) u_shared_add (
        .clk(clk), .rst_n(rst_n), .start(add_start), .ch(active_ch), .h(active_in_h), .w(active_in_w),
        .input_a_base(active_in0), .input_b_base(active_in1), .output_base(active_out),
        .a_mult(active_add_a_mult), .b_mult(active_add_b_mult), .mult_shift(active_add_mshift),
        .mem_rd_req(add_rd_req), .mem_rd_addr(add_rd_addr), .mem_rd_ready((active_type==OP_ADD)?mem_rd_ready:1'b0), .mem_rd_valid((active_type==OP_ADD)?mem_rd_valid:1'b0), .mem_rd_data(mem_rd_data),
        .mem_rd64_req(add_rd64_req), .mem_rd64_addr(add_rd64_addr), .mem_rd64_ready((active_type==OP_ADD)?mem_rd64_ready:1'b0), .mem_rd64_valid((active_type==OP_ADD)?mem_rd64_valid:1'b0), .mem_rd64_data(mem_rd64_data), // FIX204_COMBO_NONCONV64
        .mem_wr_req(add_wr_req), .mem_wr_addr(add_wr_addr), .mem_wr_data(add_wr_data), .mem_wr_ready((active_type==OP_ADD)?mem_wr_ready:1'b0),
        .mem_wr64_req(add_wr64_req), .mem_wr64_addr(add_wr64_addr), .mem_wr64_data(add_wr64_data), .mem_wr64_ready((active_type==OP_ADD)?mem_wr64_ready:1'b0), // FIX204_COMBO_NONCONV64
        .busy(), .done(add_done)
    );

    wire maxp_rd_req, maxp_wr_req, maxp_done;
    wire [ADDR_W-1:0] maxp_rd_addr, maxp_wr_addr;
    wire [DATA_W-1:0] maxp_wr_data;
    shared_maxpool5x5_ddr_engine #(.DATA_W(DATA_W), .ADDR_W(ADDR_W)) u_shared_maxp (
        .clk(clk), .rst_n(rst_n), .start(maxp_start), .ch_n(active_ch), .h(active_in_h), .w(active_in_w),
        .input_base(active_in0), .output_base(active_out),
        .mem_rd_req(maxp_rd_req), .mem_rd_addr(maxp_rd_addr), .mem_rd_ready((active_type==OP_MAXP)?mem_rd_ready:1'b0), .mem_rd_valid((active_type==OP_MAXP)?mem_rd_valid:1'b0), .mem_rd_data(mem_rd_data),
        .mem_wr_req(maxp_wr_req), .mem_wr_addr(maxp_wr_addr), .mem_wr_data(maxp_wr_data), .mem_wr_ready((active_type==OP_MAXP)?mem_wr_ready:1'b0), .busy(), .done(maxp_done)
    );

    wire det_rd_req, det_done, det_raw_valid;
    wire [ADDR_W-1:0] det_rd_addr;
    wire [1:0] det_raw_branch;
    wire [31:0] det_raw_index;
    wire signed [ACC_W-1:0] det_raw_data;
    shared_detect_raw_ddr_engine #(.DATA_W(DATA_W), .ADDR_W(ADDR_W), .ACC_W(ACC_W), .DET_CH(DET_CH)) u_shared_det (
        .clk(clk), .rst_n(rst_n), .start(det_start), .branch_id(active_branch), .in_ch(active_in_ch), .h(active_in_h), .w(active_in_w),
        .input_base(active_in0), .weight_base(active_wbase), .bias_base(active_bbase), .out_shift(active_det_shift), .out_mult(active_det_mult), .out_mult_shift(active_det_mshift),
        .mem_rd_req(det_rd_req), .mem_rd_addr(det_rd_addr), .mem_rd_ready((active_type==OP_DET)?mem_rd_ready:1'b0), .mem_rd_valid((active_type==OP_DET)?mem_rd_valid:1'b0), .mem_rd_data(mem_rd_data),
        .busy(), .done(det_done), .raw_ready((active_type==OP_DET)?det_ready:1'b0), .raw_valid(det_raw_valid), .raw_branch(det_raw_branch), .raw_index(det_raw_index), .raw_data(det_raw_data),
        .prof_state(det_prof_state) // FIX186_DET_PROFILER
    );

    assign det_valid  = (active_type == OP_DET) ? det_raw_valid  : 1'b0;
    assign det_branch = (active_type == OP_DET) ? det_raw_branch : 2'd0;
    assign det_index  = (active_type == OP_DET) ? det_raw_index  : 32'd0;
    assign det_data   = (active_type == OP_DET) ? det_raw_data   : {ACC_W{1'b0}};

    assign mem_rd32_req  = (active_type == OP_CONV) ? conv_rd32_req  : 1'b0; // FIX164_MAC4_WEIGHT_PACK32
    assign mem_rd32_addr = (active_type == OP_CONV) ? conv_rd32_addr : {ADDR_W{1'b0}}; // FIX164_MAC4_WEIGHT_PACK32
    assign mem_rd64_req  = (active_type == OP_CONV) ? conv_rd64_req  : (active_type == OP_CAT2) ? cat2_rd64_req  : (active_type == OP_UP) ? up_rd64_req : (active_type == OP_CAT4) ? cat4_rd64_req : (active_type == OP_ADD) ? add_rd64_req : 1'b0; // FIX204_COMBO_NONCONV64
    assign mem_rd64_addr = (active_type == OP_CONV) ? conv_rd64_addr : (active_type == OP_CAT2) ? cat2_rd64_addr : (active_type == OP_UP) ? up_rd64_addr : (active_type == OP_CAT4) ? cat4_rd64_addr : (active_type == OP_ADD) ? add_rd64_addr : {ADDR_W{1'b0}}; // FIX204_COMBO_NONCONV64

    assign mem_rd_req = (active_type == OP_CONV) ? conv_rd_req :
                        (active_type == OP_UP)   ? up_rd_req   :
                        (active_type == OP_CAT2) ? cat2_rd_req :
                        (active_type == OP_CAT4) ? cat4_rd_req :
                        (active_type == OP_ADD)  ? add_rd_req  :
                        (active_type == OP_MAXP) ? maxp_rd_req :
                        (active_type == OP_DET)  ? det_rd_req  : 1'b0;
    assign mem_rd_addr = (active_type == OP_CONV) ? conv_rd_addr :
                         (active_type == OP_UP)   ? up_rd_addr   :
                         (active_type == OP_CAT2) ? cat2_rd_addr :
                         (active_type == OP_CAT4) ? cat4_rd_addr :
                         (active_type == OP_ADD)  ? add_rd_addr  :
                         (active_type == OP_MAXP) ? maxp_rd_addr :
                         (active_type == OP_DET)  ? det_rd_addr  : {ADDR_W{1'b0}};
    assign mem_wr_req = (active_type == OP_CONV) ? conv_wr_req :
                        (active_type == OP_UP)   ? up_wr_req   :
                        (active_type == OP_CAT2) ? cat2_wr_req :
                        (active_type == OP_CAT4) ? cat4_wr_req :
                        (active_type == OP_ADD)  ? add_wr_req  :
                        (active_type == OP_MAXP) ? maxp_wr_req : 1'b0;
    assign mem_wr_addr = (active_type == OP_CONV) ? conv_wr_addr :
                         (active_type == OP_UP)   ? up_wr_addr   :
                         (active_type == OP_CAT2) ? cat2_wr_addr :
                         (active_type == OP_CAT4) ? cat4_wr_addr :
                         (active_type == OP_ADD)  ? add_wr_addr  :
                         (active_type == OP_MAXP) ? maxp_wr_addr : {ADDR_W{1'b0}};
    assign mem_wr_data = (active_type == OP_CONV) ? conv_wr_data :
                         (active_type == OP_UP)   ? up_wr_data   :
                         (active_type == OP_CAT2) ? cat2_wr_data :
                         (active_type == OP_CAT4) ? cat4_wr_data :
                         (active_type == OP_ADD)  ? add_wr_data  :
                         (active_type == OP_MAXP) ? maxp_wr_data : {DATA_W{1'b0}};

    assign mem_wr64_req  = (active_type == OP_CONV) ? conv_wr64_req  : (active_type == OP_CAT2) ? cat2_wr64_req  : (active_type == OP_UP) ? up_wr64_req : (active_type == OP_CAT4) ? cat4_wr64_req : (active_type == OP_ADD) ? add_wr64_req : 1'b0; // FIX_CONV_WR64_PACKER
    assign mem_wr64_addr = (active_type == OP_CONV) ? conv_wr64_addr : (active_type == OP_CAT2) ? cat2_wr64_addr : (active_type == OP_UP) ? up_wr64_addr : (active_type == OP_CAT4) ? cat4_wr64_addr : (active_type == OP_ADD) ? add_wr64_addr : {ADDR_W{1'b0}}; // FIX_CONV_WR64_PACKER
    assign mem_wr64_data = (active_type == OP_CONV) ? conv_wr64_data : (active_type == OP_CAT2) ? cat2_wr64_data : (active_type == OP_UP) ? up_wr64_data : (active_type == OP_CAT4) ? cat4_wr64_data : (active_type == OP_ADD) ? add_wr64_data : 64'd0; // FIX_CONV_WR64_PACKER
    wire active_done = (active_type == OP_CONV) ? conv_done :
                       (active_type == OP_UP)   ? up_done   :
                       (active_type == OP_CAT2) ? cat2_done :
                       (active_type == OP_CAT4) ? cat4_done :
                       (active_type == OP_ADD)  ? add_done  :
                       (active_type == OP_MAXP) ? maxp_done :
                       (active_type == OP_DET)  ? det_done  : 1'b1;


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            op_id <= 8'd0; // FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE
            run_start_lat <= 8'd0; // FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE
            run_end_lat <= 8'd84; // FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE
            active_type <= OP_NONE;
            active_in0 <= {ADDR_W{1'b0}}; active_in1 <= {ADDR_W{1'b0}}; active_in2 <= {ADDR_W{1'b0}}; active_in3 <= {ADDR_W{1'b0}};
            active_out <= {ADDR_W{1'b0}}; active_wbase <= {ADDR_W{1'b0}}; active_bbase <= {ADDR_W{1'b0}};
            active_in_ch <= 16'd0; active_out_ch <= 16'd0; active_in_h <= 16'd0; active_in_w <= 16'd0; active_out_h <= 16'd0; active_out_w <= 16'd0;
            active_k <= 4'd0; active_stride <= 4'd0; active_pad <= 4'd0;
            active_a_ch <= 16'd0; active_b_ch <= 16'd0; active_ch <= 16'd0; active_branch <= 2'd0;
            active_conv_shift <= 5'd0; active_conv_mult <= 32'd0; active_conv_mshift <= 5'd20;
            active_det_shift <= 5'd0; active_det_mult <= 32'd0; active_det_mshift <= 5'd20;
            active_silu_lut_id <= 8'd255; active_silu_in_mult <= 32'd1048576; active_silu_in_shift <= 5'd20; active_silu_out_mult <= 32'd1048576; active_silu_out_shift <= 5'd20;
            active_add_a_mult <= 32'd1048576; active_add_b_mult <= 32'd1048576; active_add_mshift <= 5'd20;
            active_cat0_mult <= 32'd1048576; active_cat1_mult <= 32'd1048576; active_cat2_mult <= 32'd1048576; active_cat3_mult <= 32'd1048576; active_cat_mshift <= 5'd20;
            active_act_silu <= 1'b0; // FIX222_CFG_LATCH_PIPE_75
        end else begin
            case (state)
                S_IDLE: begin
                    active_type <= OP_NONE;
                    if (start) begin
                        run_start_lat <= run_start_op; // FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE
                        run_end_lat <= run_end_op; // FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE
                        op_id <= run_start_op; // FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE
                        state <= S_LATCH; // FIX222_CFG_LATCH_PIPE_75
                    end
                end
                S_LATCH: begin // FIX222_CFG_LATCH_PIPE_75
                    // Capture decoded cfg for current op_id, then launch engine next cycle.
                    active_type <= cfg_type;
                    active_in0 <= cfg_in0; active_in1 <= cfg_in1; active_in2 <= cfg_in2; active_in3 <= cfg_in3;
                    active_out <= cfg_out; active_wbase <= cfg_wbase; active_bbase <= cfg_bbase;
                    active_in_ch <= cfg_in_ch; active_out_ch <= cfg_out_ch; active_in_h <= cfg_in_h; active_in_w <= cfg_in_w; active_out_h <= cfg_out_h; active_out_w <= cfg_out_w;
                    active_k <= cfg_k; active_stride <= cfg_stride; active_pad <= cfg_pad;
                    active_a_ch <= cfg_a_ch; active_b_ch <= cfg_b_ch; active_ch <= cfg_ch; active_branch <= cfg_branch;
                    active_conv_shift <= cfg_conv_shift; active_conv_mult <= cfg_conv_mult; active_conv_mshift <= cfg_conv_mshift;
                    active_det_shift <= cfg_det_shift; active_det_mult <= cfg_det_mult; active_det_mshift <= cfg_det_mshift;
                    active_silu_lut_id <= cfg_silu_lut_id; active_silu_in_mult <= cfg_silu_in_mult; active_silu_in_shift <= cfg_silu_in_shift; active_silu_out_mult <= cfg_silu_out_mult; active_silu_out_shift <= cfg_silu_out_shift;
                    active_add_a_mult <= cfg_add_a_mult; active_add_b_mult <= cfg_add_b_mult; active_add_mshift <= cfg_add_mshift;
                    active_cat0_mult <= cfg_cat0_mult; active_cat1_mult <= cfg_cat1_mult; active_cat2_mult <= cfg_cat2_mult; active_cat3_mult <= cfg_cat3_mult; active_cat_mshift <= cfg_cat_mshift;
                    active_act_silu <= cfg_act_silu;
                    state <= S_START;
                end
                S_START: begin
                    // Engine start pulses now see stable active_* cfg regs.
                    state <= S_WAIT;
                end
                S_WAIT: begin
                    if (active_done) begin
                        if (op_id == run_end_lat) begin // FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE
                            active_type <= OP_NONE;
                            state <= S_DONE;
                        end else begin
                            op_id <= op_id + 1'b1;
                            active_type <= OP_NONE;
                            state <= S_LATCH; // FIX222B_CFG_LATCH_NEXTOP_FIX
                        end
                    end
                end
                S_DONE: begin
                    state <= S_IDLE;
                end
                default: begin
                    state <= S_IDLE;
                    active_type <= OP_NONE;
                end
            endcase
        end
    end
endmodule
