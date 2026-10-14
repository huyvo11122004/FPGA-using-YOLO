// FIX221_WR_ADDR_DEEP_PIPE_75: preserve wrapper write pipe regs.
// FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE: run op00..33 then op34..84 using one core.
// yolo_kit_ddr_axi_apb.v
// Real MPFS kit wrapper for top_yolov5n_ddr.
// - APB control/status plane, compatible with generic-uio style mapping.
// - AXI4 master data plane to real DDR through CoreAXI4Interconnect/FIC.
// - Weights remain in .mem files used by the YOLO core.
// - DDR stores input image, activations/features, and raw detect output records.

`timescale 1ns/1ps

module yolo_kit_ddr_axi_apb #(
    parameter integer AXI_ADDR_WIDTH = 40,
    parameter integer AXI_DATA_WIDTH = 64,
    parameter integer ADDR_W = 32,
    parameter integer DATA_W = 8,
    parameter integer ACC_W  = 32,
    parameter integer DET_CH = 33,
    parameter integer SYS_CLK_HZ = 54_347_826,
    parameter integer MAC_LANES = 64 // FIX180_FULL_MAC64_GROUP64
)(
    input  wire                         PCLK,
    input  wire                         PRESETN,

    input  wire                         PSEL,
    input  wire                         PENABLE,
    input  wire                         PWRITE,
    input  wire [11:0]                  PADDR,
    input  wire [31:0]                  PWDATA,
    output reg  [31:0]                  PRDATA,
    output wire                         PREADY,
    output wire                         PSLVERR,
    output wire [3:0]                   M_AXI_AWID,
    output wire [AXI_ADDR_WIDTH-1:0]    M_AXI_AWADDR,
    output wire [7:0]                   M_AXI_AWLEN,
    output wire [2:0]                   M_AXI_AWSIZE,
    output wire [1:0]                   M_AXI_AWBURST,
    output wire                         M_AXI_AWLOCK,
    output wire [3:0]                   M_AXI_AWCACHE,
    output wire [2:0]                   M_AXI_AWPROT,
    output wire [3:0]                   M_AXI_AWQOS,
    output wire                         M_AXI_AWVALID,
    input  wire                         M_AXI_AWREADY,

    output wire [AXI_DATA_WIDTH-1:0]    M_AXI_WDATA,
    output wire [(AXI_DATA_WIDTH/8)-1:0] M_AXI_WSTRB,
    output wire                         M_AXI_WLAST,
    output wire                         M_AXI_WVALID,
    input  wire                         M_AXI_WREADY,
    input  wire [3:0]                   M_AXI_BID,
    input  wire [1:0]                   M_AXI_BRESP,
    input  wire                         M_AXI_BVALID,
    output wire                         M_AXI_BREADY,
    output wire [AXI_ADDR_WIDTH-1:0]    M_AXI_ARADDR,
    output wire [7:0]                   M_AXI_ARLEN,
    output wire [2:0]                   M_AXI_ARSIZE,
    output wire [1:0]                   M_AXI_ARBURST,
    output wire                         M_AXI_ARLOCK,
    output wire [3:0]                   M_AXI_ARCACHE,
    output wire [2:0]                   M_AXI_ARPROT,
    output wire [3:0]                   M_AXI_ARQOS,
    output wire                         M_AXI_ARVALID,
    input  wire                         M_AXI_ARREADY,
    output wire [3:0]                   M_AXI_ARID,
    input  wire [3:0]                   M_AXI_RID,
    input  wire [AXI_DATA_WIDTH-1:0]    M_AXI_RDATA,
    input  wire [1:0]                   M_AXI_RRESP,
    input  wire                         M_AXI_RLAST,
    input  wire                         M_AXI_RVALID,
    output wire                         M_AXI_RREADY,

    input  wire                         SPI_SCK,
    input  wire                         SPI_MOSI,
    input  wire                         SPI_CS_N,

    // Native pixel interface from Microchip MIPI CSI-2 Receiver Decoder.
    input  wire                         MIPI_PIXEL_CLK,
    input  wire [9:0]                   MIPI_RAW10,
    input  wire                         MIPI_LINE_VALID,
    input  wire                         MIPI_LINE_START,
    input  wire                         MIPI_LINE_END,
    input  wire                         MIPI_FRAME_VALID,
    // CSI_FRAME_EVENTS_V3: connect these six outputs of IMX219_CSI2_RX.
    input  wire                         MIPI_FRAME_START,
    input  wire                         MIPI_FRAME_END,
    input  wire                         MIPI_EBD_VALID,
    input  wire [7:0]                   MIPI_DATA_TYPE,
    input  wire [15:0]                  MIPI_WORD_COUNT,
    input  wire [7:0]                   MIPI_VIRTUAL_CHANNEL,
    input  wire                         MIPI_ECC_ERROR,
    input  wire                         MIPI_CRC_ERROR,

    // MIPI_LINK_DEBUG_V1: read-only probes; connect all in SmartDesign.
    input wire MIPI_BYTE_CLK,
    input wire MIPI_IOD_CLK_DONE,
    input wire MIPI_IOD_CLK_ERROR,
    input wire MIPI_PLL_LOCK,
    input wire MIPI_TRAIN0_DONE,
    input wire MIPI_TRAIN1_DONE,
    input wire MIPI_TRAIN0_ACTIVE,
    input wire MIPI_TRAIN1_ACTIVE,
    input wire [7:0] MIPI_TAP0,
    input wire [7:0] MIPI_TAP1,

    // Raspberry Pi Camera V2 control pins on the MPFS Discovery Kit J11.
    output wire                         CAM_ENABLE,
    output wire                         CAM_GPIO1,
    inout  wire                         CAM_I2C_SCL,
    inout  wire                         CAM_I2C_SDA
    
);

// MIPI_LINK_DEBUG_V1: diagnostic logic resets with PRESETN, independently
// of frontend soft reset. Existing control/data paths are unchanged.
wire [31:0] link_live, link_seen_high, link_seen_low, link_taps;
wire [31:0] link_byte_beats, link_pixel_beats;
wire [1:0] link_beat_edges;
wire [511:0] link_frozen;
localparam integer LINK_GAP_CYCLES = (SYS_CLK_HZ / 100000 < 2) ? 2 : SYS_CLK_HZ / 100000;
localparam integer LINK_WINDOW = (SYS_CLK_HZ / 1000 < 2) ? 2 : SYS_CLK_HZ / 1000;
mipi_link_debug_v1 #(.WINDOW_CYCLES(LINK_WINDOW)) u_mipi_link_debug (
    .sys_clk(PCLK), .rst_n(PRESETN),
    .byte_clk(MIPI_BYTE_CLK), .pixel_clk(MIPI_PIXEL_CLK),
    .flags_async({MIPI_TRAIN1_ACTIVE, MIPI_TRAIN0_ACTIVE,
                  MIPI_TRAIN1_DONE, MIPI_TRAIN0_DONE, MIPI_PLL_LOCK,
                  MIPI_IOD_CLK_ERROR, MIPI_IOD_CLK_DONE}),
    .taps_async({MIPI_TAP1, MIPI_TAP0}),
    .live(link_live), .seen_high(link_seen_high), .seen_low(link_seen_low),
    .taps(link_taps), .byte_beats(link_byte_beats), .pixel_beats(link_pixel_beats),
    .beat_edges(link_beat_edges)
);

mipi_link_events_v2 #(.GAP_CYCLES(LINK_GAP_CYCLES)) u_link_events (
    .sys_clk(PCLK), .rst_n(PRESETN),
    .snapshot(PSEL && PENABLE && !PWRITE && (PADDR == 12'h1e0)),
    .live(link_live), .taps(link_taps),
    .byte_beats(link_byte_beats), .pixel_beats(link_pixel_beats),
    .byte_edge(link_beat_edges[0]), .pixel_edge(link_beat_edges[1]),
    .frozen(link_frozen)
);

// CSI3_SERIAL_ROUTING_FIX: native snapshot travels as sixteen 32-bit words.
// Stop-clock intervals leave the previous published bank unchanged.
wire [511:0] csi_frozen;
wire [31:0] csi_live, csi_sequence, csi_loss_in_frame;
wire csi_snapshot_pending;
mipi_decoder_debug_v3 u_csi_debug (
    .sys_clk(PCLK), .pixel_clk(MIPI_PIXEL_CLK), .rst_n(PRESETN),
    .snapshot(PSEL && PENABLE && !PWRITE && (PADDR == 12'h1e0)),
    .fs(MIPI_FRAME_START), .fe(MIPI_FRAME_END),
    .ls(MIPI_LINE_START), .le(MIPI_LINE_END),
    .fv(MIPI_FRAME_VALID), .lv(MIPI_LINE_VALID), .ebd(MIPI_EBD_VALID),
    .ecc(MIPI_ECC_ERROR), .crc(MIPI_CRC_ERROR),
    .dt(MIPI_DATA_TYPE), .wc(MIPI_WORD_COUNT), .vc(MIPI_VIRTUAL_CHANNEL),
    .pll_lock(link_live[2]), .live_flags(csi_live),
    .pending(csi_snapshot_pending), .sequence_count(csi_sequence),
    .loss_in_frame(csi_loss_in_frame), .frozen(csi_frozen)
);

assign PREADY  = 1'b1;
assign PSLVERR = 1'b0;

// Register map, APB byte offsets:
// 0x00 CTRL      W bit0=start pulse, bit1=clear, bit2=soft_reset while 1
// 0x04 STATUS    R bit0=busy, bit1=done_latched, bit2=error, bit3=bridge_busy, bit4=det_overflow
// 0x08 BASE_LO   RW fabric DDR base low, default 0x08000000
// 0x0c BASE_HI   RW fabric DDR base high, default 0x14
// 0x10 DET_LO    RW detect output local offset low, default 0x03000000
// 0x14 DET_HI    reserved/high, currently ignored for local offset
// 0x18 DET_COUNT R raw detect records stored
// 0x1c DEBUG     R {26'd0, debug_op_id}
// 0x20 RD_COUNT  R bridge byte reads
// 0x24 WR_COUNT  R bridge byte writes
// 0x28 BUILD_MAC R 0x4D43_0000 | MAC_LANES                    // FIX164_PROFILER_COUNTERS_V2
// 0x2c BUILD_DET R 0x4443_0000 | DET_CH                       // FIX164_PROFILER_COUNTERS_V2
// 0x30 PROF_TOTAL_CYCLES       R busy/detect/bridge cycles    // FIX164_PROFILER_COUNTERS_V2
// 0x34 PROF_RD_REQ_CYCLES      R y_mem_rd_req cycles          // FIX164_PROFILER_COUNTERS_V2
// 0x38 PROF_RD_WAIT_CYCLES     R y_mem_rd_req && !valid       // FIX164_PROFILER_COUNTERS_V2
// 0x3c PROF_WR_REQ_CYCLES      R write request cycles         // FIX164_PROFILER_COUNTERS_V2
// 0x40 PROF_WR_WAIT_CYCLES     R write req && !ready          // FIX164_PROFILER_COUNTERS_V2
// 0x44 PROF_CORE_ACTIVE_CYCLES R y_busy no direct IO req       // FIX164_PROFILER_COUNTERS_V2
// 0x48 PROF_BRIDGE_BUSY_CYCLES R bridge_busy cycles           // FIX164_PROFILER_COUNTERS_V2
// 0x4c PROF_DET_BUSY_CYCLES    R detect writer busy cycles    // FIX164_PROFILER_COUNTERS_V2
// 0x50 PROF_NO_PROGRESS_CYCLES R busy but RD/WR/DET frozen    // FIX164_PROFILER_COUNTERS_V2
// 0x54 PROF_CUR_STATE          R {op_id,state}              // FIX164_MAC16_DETAILED_PROFILER
// 0x58..0x7c CONV state cycle counters                    // FIX164_MAC16_DETAILED_PROFILER
// 0x80..0x94 RD8/RD32/RD64 req/wait counters              // FIX164_MAC16_DETAILED_PROFILER

reg [AXI_ADDR_WIDTH-1:0] ddr_base;
reg [ADDR_W-1:0] det_base;
reg done_latched;
reg soft_reset;
reg start_req;
reg clear_req;

wire apb_write = PSEL && PENABLE && PWRITE;
wire apb_read  = PSEL && !PWRITE;
reg [31:0] prdata_comb; // FIX203_COMBO_MARGIN_K1RD64

wire core_rst_n = PRESETN && !soft_reset;
wire clear_all = clear_req;
wire start_pulse = start_req;

// FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE:
// One physical YOLO core is started twice:
//   Stage A: op00..33
//   Stage B: op34..84
// Resource stays near single-core; output should match full op00..84.
localparam [2:0] SC_IDLE    = 3'd0;
localparam [2:0] SC_START_A = 3'd1;
localparam [2:0] SC_WAIT_A  = 3'd2;
localparam [2:0] SC_START_B = 3'd3;
localparam [2:0] SC_WAIT_B  = 3'd4;
reg [2:0] sc_state;

wire sc_core_start = (sc_state == SC_START_A) || (sc_state == SC_START_B);
wire [7:0] sc_run_start_op = ((sc_state == SC_START_B) || (sc_state == SC_WAIT_B)) ? 8'd34 : 8'd0;
wire [7:0] sc_run_end_op   = ((sc_state == SC_START_B) || (sc_state == SC_WAIT_B)) ? 8'd84 : 8'd33;
wire sc_busy = (sc_state != SC_IDLE);
wire sc_done = (sc_state == SC_WAIT_B) && y_core_done;
assign M_AXI_AWID = 4'h0;
assign M_AXI_ARID = 4'h0;

always @(posedge PCLK or negedge PRESETN) begin
    if (!PRESETN) begin
        sc_state <= SC_IDLE;
    end else if (soft_reset || clear_all) begin
        sc_state <= SC_IDLE;
    end else begin
        case (sc_state)
            SC_IDLE: begin
                if (start_pulse)
                    sc_state <= SC_START_A;
            end
            SC_START_A: begin
                sc_state <= SC_WAIT_A;
            end
            SC_WAIT_A: begin
                if (y_core_done)
                    sc_state <= SC_START_B;
            end
            SC_START_B: begin
                sc_state <= SC_WAIT_B;
            end
            SC_WAIT_B: begin
                if (y_core_done)
                    sc_state <= SC_IDLE;
            end
            default: begin
                sc_state <= SC_IDLE;
            end
        endcase
    end
end

wire y_busy;
wire y_done;
wire y_core_busy;
wire y_core_done;
assign y_busy = sc_busy;
assign y_done = sc_done;
wire y_mem_rd_req;
wire [ADDR_W-1:0] y_mem_rd_addr;
wire y_mem_rd_ready;
wire y_mem_rd_valid;
wire [DATA_W-1:0] y_mem_rd_data;
wire y_mem_rd32_req;                 // FIX164_MAC4_WEIGHT_PACK32
wire [ADDR_W-1:0] y_mem_rd32_addr;   // FIX164_MAC4_WEIGHT_PACK32
wire y_mem_rd32_ready;               // FIX164_MAC4_WEIGHT_PACK32
wire y_mem_rd32_valid;               // FIX164_MAC4_WEIGHT_PACK32
wire [31:0] y_mem_rd32_data;         // FIX164_MAC4_WEIGHT_PACK32
wire y_mem_rd64_req;                 // FIX164_MAC8_VECTOR_PACK64
wire [ADDR_W-1:0] y_mem_rd64_addr;   // FIX164_MAC8_VECTOR_PACK64
wire y_mem_rd64_ready;               // FIX164_MAC8_VECTOR_PACK64
wire y_mem_rd64_valid;               // FIX164_MAC8_VECTOR_PACK64
wire [63:0] y_mem_rd64_data;         // FIX164_MAC8_VECTOR_PACK64
wire y_mem_wr64_req;                  // FIX202_CAT2_RD64_WR64
wire [ADDR_W-1:0] y_mem_wr64_addr;    // FIX202_CAT2_RD64_WR64
wire [63:0] y_mem_wr64_data;          // FIX202_CAT2_RD64_WR64
wire y_mem_wr64_ready;                // FIX202_CAT2_RD64_WR64
wire y_mem_wr_req;
wire [ADDR_W-1:0] y_mem_wr_addr;
wire [DATA_W-1:0] y_mem_wr_data;
wire y_mem_wr_ready;
wire det_valid;
wire [1:0] det_branch;
wire [31:0] det_index;
wire signed [ACC_W-1:0] det_data;
wire [5:0] debug_op_id;
wire [7:0] y_prof_op_id;             // FIX164_MAC16_OP_PROFILER
wire [4:0] y_conv_prof_state;        // FIX165_PROF_5BIT_WCACHE
wire [3:0] y_det_prof_state;         // FIX186_DET_PROFILER

wire det_mem_wr_req;
wire [ADDR_W-1:0] det_mem_wr_addr;
wire [DATA_W-1:0] det_mem_wr_data;
wire det_mem_wr_ready;
wire det_mem_wr64_req; // FIX187E_DET_RECORD_WR64_COMPLETE
wire [ADDR_W-1:0] det_mem_wr64_addr; // FIX187E_DET_RECORD_WR64_COMPLETE
wire [63:0] det_mem_wr64_data; // FIX187E_DET_RECORD_WR64_COMPLETE
wire det_mem_wr64_ready; // FIX187E_DET_RECORD_WR64_COMPLETE
wire [31:0] det_count;
wire det_overflow;
wire det_writer_busy;
wire det_writer_ready;

wire bridge_mem_wr_req;
wire [ADDR_W-1:0] bridge_mem_wr_addr;
wire [DATA_W-1:0] bridge_mem_wr_data;
wire bridge_mem_wr_ready;

// FIX210B_WR_ADDR_PIPE_NORMAL_ONLY:
// cut op_id/cfg_out -> u_bridge/wr_axi_addr timing path for normal byte writes.
wire spi_mem_wr_req;
wire [ADDR_W-1:0] spi_mem_wr_addr;
wire [DATA_W-1:0] spi_mem_wr_data;
wire spi_mem_wr_ready;
wire spi_frame_loaded;
wire spi_busy;
wire spi_error;
wire spi_overrun_error;
// FIX_SPI_DROP_WHEN_BUSY:
// Accept a new SPI transaction only when YOLO/DET/DDR bridge is idle.
wire spi_accept_enable;
wire [7:0] spi_debug_state;
wire [31:0] spi_payload_count;
wire [31:0] spi_frames_loaded_count;

// IMX219 camera frontend.  Camera and SPI remain alternative frame sources.
wire cam_mem_wr_req;
wire [ADDR_W-1:0] cam_mem_wr_addr;
wire [DATA_W-1:0] cam_mem_wr_data;
wire cam_mem_wr_ready;
wire cam_frame_done_pulse;
wire cam_frame_loaded;
wire cam_busy;
wire cam_error;
wire cam_overrun_error;
wire cam_accept_enable;
wire [31:0] cam_payload_count;
wire [31:0] cam_frames_loaded_count;
wire [7:0] cam_debug_state;
wire cam_init_done;
wire cam_init_busy;
wire cam_init_error;
wire [15:0] cam_chip_id;
wire [31:0] cam_frame_count;
wire [31:0] cam_pixel_count;
wire cam_size_error;
wire cam_ecc_error;
wire cam_crc_error;
wire [31:0] cam_last_frame_pixels, cam_last_frame_info;
wire [31:0] bridge_debug_status;
wire [AXI_ADDR_WIDTH-1:0] bridge_first_error_addr, bridge_last_write_addr;

wire frontend_frame_loaded = cam_frame_loaded | spi_frame_loaded;
wire frontend_busy         = cam_busy | spi_busy;
wire frontend_error        = cam_error | spi_error;
wire frontend_overrun      = cam_overrun_error | spi_overrun_error;

wire raw_bridge_mem_wr_req;
wire [ADDR_W-1:0] raw_bridge_mem_wr_addr;
wire [DATA_W-1:0] raw_bridge_mem_wr_data;
(* syn_preserve = 1, syn_keep = 1 *) reg  bridge_mem_wr_req_r; // FIX221_WR_ADDR_DEEP_PIPE_75
(* syn_preserve = 1, syn_keep = 1 *) reg  [ADDR_W-1:0] bridge_mem_wr_addr_r; // FIX221_WR_ADDR_DEEP_PIPE_75
(* syn_preserve = 1, syn_keep = 1 *) reg  [DATA_W-1:0] bridge_mem_wr_data_r; // FIX221_WR_ADDR_DEEP_PIPE_75
wire bridge_mem_wr_can_accept;

wire bridge_mem_wr64_req; // FIX187E_DET_RECORD_WR64_COMPLETE
wire [ADDR_W-1:0] bridge_mem_wr64_addr; // FIX187E_DET_RECORD_WR64_COMPLETE
wire [63:0] bridge_mem_wr64_data; // FIX187E_DET_RECORD_WR64_COMPLETE
wire bridge_mem_wr64_ready; // FIX187E_DET_RECORD_WR64_COMPLETE
reg bridge_mem_wr64_req_r;
reg [ADDR_W-1:0] bridge_mem_wr64_addr_r;
reg [63:0] bridge_mem_wr64_data_r;
wire bridge_mem_wr64_can_accept;
wire bridge_mem_wr64_take_det;
wire bridge_mem_wr64_take_yolo;
wire bridge_busy;
wire bridge_error;
wire [31:0] bridge_rd_count;
wire [31:0] bridge_wr_count;



// FIX164_MAC16_DETAILED_PROFILER: cycle counters by conv FSM state and read type.
// These are diagnostic-only counters.  They reset on reset/start/clear.
reg [31:0] prof_st_bias_req;
reg [31:0] prof_st_bias_wait;
reg [31:0] prof_st_bias_check;
reg [31:0] prof_st_w_req;
reg [31:0] prof_st_w_wait;
reg [31:0] prof_st_w64_req;
reg [31:0] prof_st_w64_wait;
reg [31:0] prof_st_act_req;
reg [31:0] prof_st_act_wait;
reg [31:0] prof_st_mac;
reg [31:0] prof_st_write;
reg [31:0] prof_st_other;
// FIX165_PROF_5BIT_WCACHE: split former S_OTHER alias after full CONV weight cache.
reg [31:0] prof_st_write_prep;
reg [31:0] prof_st_wcache_check;
reg [31:0] prof_st_wcache_load_req;
reg [31:0] prof_st_wcache_load_wait;
reg [31:0] prof_st_wcache_use_req;
reg [31:0] prof_st_wcache_use_wait;
reg [31:0] prof_rd8_req_cycles;
reg [31:0] prof_rd8_wait_cycles;
reg [31:0] prof_rd32_req_cycles;
reg [31:0] prof_rd32_wait_cycles;
reg [31:0] prof_rd64_req_cycles;
reg [31:0] prof_rd64_wait_cycles;

always @(posedge PCLK or negedge PRESETN) begin
    if (!PRESETN) begin
        prof_st_bias_req    <= 32'd0;
        prof_st_bias_wait   <= 32'd0;
        prof_st_bias_check  <= 32'd0;
        prof_st_w_req       <= 32'd0;
        prof_st_w_wait      <= 32'd0;
        prof_st_w64_req     <= 32'd0;
        prof_st_w64_wait    <= 32'd0;
        prof_st_act_req     <= 32'd0;
        prof_st_act_wait    <= 32'd0;
        prof_st_mac         <= 32'd0;
        prof_st_write       <= 32'd0;
        prof_st_other       <= 32'd0;
        prof_st_write_prep  <= 32'd0;
        prof_st_wcache_check <= 32'd0;
        prof_st_wcache_load_req <= 32'd0;
        prof_st_wcache_load_wait <= 32'd0;
        prof_st_wcache_use_req <= 32'd0;
        prof_st_wcache_use_wait <= 32'd0;
        prof_rd8_req_cycles <= 32'd0;
        prof_rd8_wait_cycles <= 32'd0;
        prof_rd32_req_cycles <= 32'd0;
        prof_rd32_wait_cycles <= 32'd0;
        prof_rd64_req_cycles <= 32'd0;
        prof_rd64_wait_cycles <= 32'd0;
    end else begin
        if (start_pulse || clear_all) begin
            prof_st_bias_req    <= 32'd0;
            prof_st_bias_wait   <= 32'd0;
            prof_st_bias_check  <= 32'd0;
            prof_st_w_req       <= 32'd0;
            prof_st_w_wait      <= 32'd0;
            prof_st_w64_req     <= 32'd0;
            prof_st_w64_wait    <= 32'd0;
            prof_st_act_req     <= 32'd0;
            prof_st_act_wait    <= 32'd0;
            prof_st_mac         <= 32'd0;
            prof_st_write       <= 32'd0;
            prof_st_other       <= 32'd0;
            prof_rd8_req_cycles <= 32'd0;
            prof_rd8_wait_cycles <= 32'd0;
            prof_rd32_req_cycles <= 32'd0;
            prof_rd32_wait_cycles <= 32'd0;
            prof_rd64_req_cycles <= 32'd0;
            prof_rd64_wait_cycles <= 32'd0;
        end else begin
            if (y_busy) begin
                case (y_conv_prof_state)
                    5'd1:  prof_st_bias_req   <= prof_st_bias_req + 32'd1;
                    5'd2:  prof_st_bias_wait  <= prof_st_bias_wait + 32'd1;
                    5'd3:  prof_st_mac        <= prof_st_mac + 32'd1;
                    5'd4:  prof_st_w_req      <= prof_st_w_req + 32'd1;
                    5'd5:  prof_st_w_wait     <= prof_st_w_wait + 32'd1;
                    5'd6:  prof_st_act_req    <= prof_st_act_req + 32'd1;
                    5'd7:  prof_st_act_wait   <= prof_st_act_wait + 32'd1;
                    5'd8:  prof_st_write      <= prof_st_write + 32'd1;
                    5'd12: prof_st_w64_req    <= prof_st_w64_req + 32'd1;
                    5'd13: prof_st_w64_wait   <= prof_st_w64_wait + 32'd1;
                    5'd14: prof_st_bias_check <= prof_st_bias_check + 32'd1;
                    5'd15: prof_st_write_prep <= prof_st_write_prep + 32'd1;
                    5'd16: prof_st_wcache_check <= prof_st_wcache_check + 32'd1;
                    5'd17: prof_st_wcache_load_req <= prof_st_wcache_load_req + 32'd1;
                    5'd18: prof_st_wcache_load_wait <= prof_st_wcache_load_wait + 32'd1;
                    5'd19: prof_st_wcache_use_req <= prof_st_wcache_use_req + 32'd1;
                    5'd20: prof_st_wcache_use_wait <= prof_st_wcache_use_wait + 32'd1;
                    default: prof_st_other    <= prof_st_other + 32'd1;
                endcase
            end

            if (y_busy && y_mem_rd_req)
                prof_rd8_req_cycles <= prof_rd8_req_cycles + 32'd1;
            if (y_busy && y_mem_rd_req && !y_mem_rd_valid)
                prof_rd8_wait_cycles <= prof_rd8_wait_cycles + 32'd1;
            if (y_busy && y_mem_rd32_req)
                prof_rd32_req_cycles <= prof_rd32_req_cycles + 32'd1;
            if (y_busy && y_mem_rd32_req && !y_mem_rd32_valid)
                prof_rd32_wait_cycles <= prof_rd32_wait_cycles + 32'd1;
            if (y_busy && y_mem_rd64_req)
                prof_rd64_req_cycles <= prof_rd64_req_cycles + 32'd1;
            if (y_busy && y_mem_rd64_req && !y_mem_rd64_valid)
                prof_rd64_wait_cycles <= prof_rd64_wait_cycles + 32'd1;
        end
    end
end



// FIX186_DET_PROFILER: detailed counters for OP_DET / shared_detect_raw_ddr_engine.
// These are diagnostic-only and reset on start/clear/reset.
reg [31:0] prof_det_total_cycles;
reg [31:0] prof_det_s_cycles;
reg [31:0] prof_det_m_cycles;
reg [31:0] prof_det_l_cycles;
reg [31:0] prof_det_bias_cycles;
reg [31:0] prof_det_wcache_cycles;
reg [31:0] prof_det_pixel_init_cycles;
reg [31:0] prof_det_act_req_cycles;
reg [31:0] prof_det_act_wait_cycles;
reg [31:0] prof_det_oc_mac_cycles;
reg [31:0] prof_det_out_cycles;
reg [31:0] prof_det_gap_cycles;
reg [31:0] prof_det_raw_valid_cycles;
reg [31:0] prof_det_raw_busy_cycles;
reg [31:0] prof_det_wr_req_cycles;
reg [31:0] prof_det_wr_wait_cycles;
// FIX188B_PER_OP_PROFILER:
// Per-op counters, read through APB window 0x800..0xfff.
//   addr = 0x800 + op_id*16 + metric*4
//   metric 0 = cycles
//   metric 1 = ACT_WAIT-state cycles
//   metric 2 = RD8 wait cycles
//   metric 3 = WR wait cycles
integer prof_op_i; // FIX188B_PER_OP_PROFILER
reg [31:0] prof_op_cycles   [0:127]; // FIX188B_PER_OP_PROFILER
reg [31:0] prof_op_act_wait [0:127]; // FIX188B_PER_OP_PROFILER
reg [31:0] prof_op_rd8_wait [0:127]; // FIX188B_PER_OP_PROFILER
reg [31:0] prof_op_wr_wait  [0:127]; // FIX188B_PER_OP_PROFILER
wire        prof_op_active = y_busy && (y_prof_op_id < 8'd128); // FIX188B_PER_OP_PROFILER
wire [6:0]  prof_op_idx    = y_prof_op_id[6:0]; // FIX188B_PER_OP_PROFILER


wire prof_det_active = y_busy && (y_prof_op_id >= 8'd82) && (y_prof_op_id <= 8'd84);

always @(posedge PCLK or negedge PRESETN) begin
    if (!PRESETN) begin
        prof_det_total_cycles      <= 32'd0;
        prof_det_s_cycles          <= 32'd0;
        prof_det_m_cycles          <= 32'd0;
        prof_det_l_cycles          <= 32'd0;
        prof_det_bias_cycles       <= 32'd0;
        prof_det_wcache_cycles     <= 32'd0;
        prof_det_pixel_init_cycles <= 32'd0;
        prof_det_act_req_cycles    <= 32'd0;
        prof_det_act_wait_cycles   <= 32'd0;
        prof_det_oc_mac_cycles     <= 32'd0;
        prof_det_out_cycles        <= 32'd0;
        prof_det_gap_cycles        <= 32'd0;
        prof_det_raw_valid_cycles  <= 32'd0;
        prof_det_raw_busy_cycles   <= 32'd0;
        prof_det_wr_req_cycles     <= 32'd0;
        prof_det_wr_wait_cycles    <= 32'd0;
            for (prof_op_i = 0; prof_op_i < 128; prof_op_i = prof_op_i + 1) begin // FIX188B_PER_OP_PROFILER
                prof_op_cycles[prof_op_i]   <= 32'd0; // FIX188B_PER_OP_PROFILER
                prof_op_act_wait[prof_op_i] <= 32'd0; // FIX188B_PER_OP_PROFILER
                prof_op_rd8_wait[prof_op_i] <= 32'd0; // FIX188B_PER_OP_PROFILER
                prof_op_wr_wait[prof_op_i]  <= 32'd0; // FIX188B_PER_OP_PROFILER
            end // FIX188B_PER_OP_PROFILER
    end else begin
        if (start_pulse || clear_all) begin
            prof_det_total_cycles      <= 32'd0;
            prof_det_s_cycles          <= 32'd0;
            prof_det_m_cycles          <= 32'd0;
            prof_det_l_cycles          <= 32'd0;
            prof_det_bias_cycles       <= 32'd0;
            prof_det_wcache_cycles     <= 32'd0;
            prof_det_pixel_init_cycles <= 32'd0;
            prof_det_act_req_cycles    <= 32'd0;
            prof_det_act_wait_cycles   <= 32'd0;
            prof_det_oc_mac_cycles     <= 32'd0;
            prof_det_out_cycles        <= 32'd0;
            prof_det_gap_cycles        <= 32'd0;
            prof_det_raw_valid_cycles  <= 32'd0;
            prof_det_raw_busy_cycles   <= 32'd0;
            prof_det_wr_req_cycles     <= 32'd0;
            prof_det_wr_wait_cycles    <= 32'd0;
            for (prof_op_i = 0; prof_op_i < 128; prof_op_i = prof_op_i + 1) begin // FIX188B_PER_OP_PROFILER
                prof_op_cycles[prof_op_i]   <= 32'd0; // FIX188B_PER_OP_PROFILER
                prof_op_act_wait[prof_op_i] <= 32'd0; // FIX188B_PER_OP_PROFILER
                prof_op_rd8_wait[prof_op_i] <= 32'd0; // FIX188B_PER_OP_PROFILER
                prof_op_wr_wait[prof_op_i]  <= 32'd0; // FIX188B_PER_OP_PROFILER
            end // FIX188B_PER_OP_PROFILER
        end else begin
            if (prof_op_active) begin // FIX188B_PER_OP_PROFILER
                prof_op_cycles[prof_op_idx] <= prof_op_cycles[prof_op_idx] + 32'd1; // FIX188B_PER_OP_PROFILER
                if (y_conv_prof_state == 5'd7) begin // S_ACT_WAIT in conv2d_ddr_core
                    prof_op_act_wait[prof_op_idx] <= prof_op_act_wait[prof_op_idx] + 32'd1; // FIX188B_PER_OP_PROFILER
                end
                if (y_mem_rd_req && !y_mem_rd_valid) begin
                    prof_op_rd8_wait[prof_op_idx] <= prof_op_rd8_wait[prof_op_idx] + 32'd1; // FIX188B_PER_OP_PROFILER
                end
                if (y_mem_wr_req && !y_mem_wr_ready) begin
                    prof_op_wr_wait[prof_op_idx] <= prof_op_wr_wait[prof_op_idx] + 32'd1; // FIX188B_PER_OP_PROFILER
                end
            end

            if (prof_det_active) begin
                prof_det_total_cycles <= prof_det_total_cycles + 32'd1;
                if (y_prof_op_id == 8'd82) prof_det_s_cycles <= prof_det_s_cycles + 32'd1;
                if (y_prof_op_id == 8'd83) prof_det_m_cycles <= prof_det_m_cycles + 32'd1;
                if (y_prof_op_id == 8'd84) prof_det_l_cycles <= prof_det_l_cycles + 32'd1;

                case (y_det_prof_state)
                    4'd1, 4'd2: prof_det_bias_cycles       <= prof_det_bias_cycles + 32'd1;
                    4'd3, 4'd4: prof_det_wcache_cycles     <= prof_det_wcache_cycles + 32'd1;
                    4'd5:       prof_det_pixel_init_cycles <= prof_det_pixel_init_cycles + 32'd1;
                    4'd6:       prof_det_act_req_cycles    <= prof_det_act_req_cycles + 32'd1;
                    4'd7:       prof_det_act_wait_cycles   <= prof_det_act_wait_cycles + 32'd1;
                    4'd8:       prof_det_oc_mac_cycles     <= prof_det_oc_mac_cycles + 32'd1;
                    4'd9:       prof_det_out_cycles        <= prof_det_out_cycles + 32'd1;
                    4'd10:      prof_det_gap_cycles        <= prof_det_gap_cycles + 32'd1;
                    default: begin end
                endcase
            end

            if (det_valid) begin
                prof_det_raw_valid_cycles <= prof_det_raw_valid_cycles + 32'd1;
                if (det_writer_busy) prof_det_raw_busy_cycles <= prof_det_raw_busy_cycles + 32'd1;
            end
            if (det_mem_wr_req || det_mem_wr64_req) begin
                prof_det_wr_req_cycles <= prof_det_wr_req_cycles + 32'd1;
                if ((det_mem_wr_req && !det_mem_wr_ready) || (det_mem_wr64_req && !det_mem_wr64_ready))
                    prof_det_wr_wait_cycles <= prof_det_wr_wait_cycles + 32'd1;
            end
        end
    end
end

// FIX164_MAC16_OP_PROFILER: lightweight per-op profiler.
// Captures current, last completed, and max-cycle op.  Diagnostic only.
reg        op_prof_active;
reg [7:0]  op_prof_cur_id;
reg [31:0] op_prof_cur_cycles;
reg [31:0] op_prof_cur_w64_wait;
reg [31:0] op_prof_cur_act_wait;
reg [31:0] op_prof_cur_rd8_wait;
reg [31:0] op_prof_cur_rd64_wait;

reg [7:0]  op_prof_last_id;
reg [31:0] op_prof_last_cycles;
reg [31:0] op_prof_last_w64_wait;
reg [31:0] op_prof_last_act_wait;
reg [31:0] op_prof_last_rd8_wait;
reg [31:0] op_prof_last_rd64_wait;

reg [7:0]  op_prof_max_id;
reg [31:0] op_prof_max_cycles;
reg [31:0] op_prof_max_w64_wait;
reg [31:0] op_prof_max_act_wait;

always @(posedge PCLK or negedge PRESETN) begin
    if (!PRESETN) begin
        op_prof_active <= 1'b0;
        op_prof_cur_id <= 8'd0;
        op_prof_cur_cycles <= 32'd0;
        op_prof_cur_w64_wait <= 32'd0;
        op_prof_cur_act_wait <= 32'd0;
        op_prof_cur_rd8_wait <= 32'd0;
        op_prof_cur_rd64_wait <= 32'd0;
        op_prof_last_id <= 8'd0;
        op_prof_last_cycles <= 32'd0;
        op_prof_last_w64_wait <= 32'd0;
        op_prof_last_act_wait <= 32'd0;
        op_prof_last_rd8_wait <= 32'd0;
        op_prof_last_rd64_wait <= 32'd0;
        op_prof_max_id <= 8'd0;
        op_prof_max_cycles <= 32'd0;
        op_prof_max_w64_wait <= 32'd0;
        op_prof_max_act_wait <= 32'd0;
    end else begin
        if (start_pulse || clear_all) begin
            op_prof_active <= 1'b0;
            op_prof_cur_id <= 8'd0;
            op_prof_cur_cycles <= 32'd0;
            op_prof_cur_w64_wait <= 32'd0;
            op_prof_cur_act_wait <= 32'd0;
            op_prof_cur_rd8_wait <= 32'd0;
            op_prof_cur_rd64_wait <= 32'd0;
            op_prof_last_id <= 8'd0;
            op_prof_last_cycles <= 32'd0;
            op_prof_last_w64_wait <= 32'd0;
            op_prof_last_act_wait <= 32'd0;
            op_prof_last_rd8_wait <= 32'd0;
            op_prof_last_rd64_wait <= 32'd0;
            op_prof_max_id <= 8'd0;
            op_prof_max_cycles <= 32'd0;
            op_prof_max_w64_wait <= 32'd0;
            op_prof_max_act_wait <= 32'd0;
        end else if (y_busy) begin
            if (!op_prof_active) begin
                op_prof_active <= 1'b1;
                op_prof_cur_id <= y_prof_op_id;
                op_prof_cur_cycles <= 32'd1;
                op_prof_cur_w64_wait <= (y_conv_prof_state == 5'd13) ? 32'd1 : 32'd0;
                op_prof_cur_act_wait <= (y_conv_prof_state == 5'd7)  ? 32'd1 : 32'd0;
                op_prof_cur_rd8_wait <= (y_mem_rd_req && !y_mem_rd_valid) ? 32'd1 : 32'd0;
                op_prof_cur_rd64_wait <= (y_mem_rd64_req && !y_mem_rd64_valid) ? 32'd1 : 32'd0;
            end else if (y_prof_op_id != op_prof_cur_id) begin
                // Finish old op.
                op_prof_last_id <= op_prof_cur_id;
                op_prof_last_cycles <= op_prof_cur_cycles;
                op_prof_last_w64_wait <= op_prof_cur_w64_wait;
                op_prof_last_act_wait <= op_prof_cur_act_wait;
                op_prof_last_rd8_wait <= op_prof_cur_rd8_wait;
                op_prof_last_rd64_wait <= op_prof_cur_rd64_wait;
                if (op_prof_cur_cycles > op_prof_max_cycles) begin
                    op_prof_max_id <= op_prof_cur_id;
                    op_prof_max_cycles <= op_prof_cur_cycles;
                    op_prof_max_w64_wait <= op_prof_cur_w64_wait;
                    op_prof_max_act_wait <= op_prof_cur_act_wait;
                end
                // Start new op with this cycle counted.
                op_prof_cur_id <= y_prof_op_id;
                op_prof_cur_cycles <= 32'd1;
                op_prof_cur_w64_wait <= (y_conv_prof_state == 5'd13) ? 32'd1 : 32'd0;
                op_prof_cur_act_wait <= (y_conv_prof_state == 5'd7)  ? 32'd1 : 32'd0;
                op_prof_cur_rd8_wait <= (y_mem_rd_req && !y_mem_rd_valid) ? 32'd1 : 32'd0;
                op_prof_cur_rd64_wait <= (y_mem_rd64_req && !y_mem_rd64_valid) ? 32'd1 : 32'd0;
            end else begin
                op_prof_cur_cycles <= op_prof_cur_cycles + 32'd1;
                if (y_conv_prof_state == 5'd13) op_prof_cur_w64_wait <= op_prof_cur_w64_wait + 32'd1;
                if (y_conv_prof_state == 5'd7)  op_prof_cur_act_wait <= op_prof_cur_act_wait + 32'd1;
                if (y_mem_rd_req && !y_mem_rd_valid) op_prof_cur_rd8_wait <= op_prof_cur_rd8_wait + 32'd1;
                if (y_mem_rd64_req && !y_mem_rd64_valid) op_prof_cur_rd64_wait <= op_prof_cur_rd64_wait + 32'd1;
            end
        end else if (op_prof_active) begin
            op_prof_active <= 1'b0;
            op_prof_last_id <= op_prof_cur_id;
            op_prof_last_cycles <= op_prof_cur_cycles;
            op_prof_last_w64_wait <= op_prof_cur_w64_wait;
            op_prof_last_act_wait <= op_prof_cur_act_wait;
            op_prof_last_rd8_wait <= op_prof_cur_rd8_wait;
            op_prof_last_rd64_wait <= op_prof_cur_rd64_wait;
            if (op_prof_cur_cycles > op_prof_max_cycles) begin
                op_prof_max_id <= op_prof_cur_id;
                op_prof_max_cycles <= op_prof_cur_cycles;
                op_prof_max_w64_wait <= op_prof_cur_w64_wait;
                op_prof_max_act_wait <= op_prof_cur_act_wait;
            end
        end
    end
end

// FIX164_PROFILER_COUNTERS_V2: non-intrusive runtime counters.
// These only observe existing handshake/status signals; they do not change FSM flow.
reg [31:0] prof_total_cycles;
reg [31:0] prof_rd_req_cycles;
reg [31:0] prof_rd_wait_cycles;
reg [31:0] prof_wr_req_cycles;
reg [31:0] prof_wr_wait_cycles;
reg [31:0] prof_core_active_cycles;
reg [31:0] prof_bridge_busy_cycles;
reg [31:0] prof_det_busy_cycles;
reg [31:0] prof_no_progress_cycles;
reg [31:0] prof_last_rd_count;
reg [31:0] prof_last_wr_count;
reg [31:0] prof_last_det_count;
wire prof_any_busy = y_busy | det_writer_busy | bridge_busy;
wire prof_no_progress_now = prof_any_busy &&
                            (bridge_rd_count == prof_last_rd_count) &&
                            (bridge_wr_count == prof_last_wr_count) &&
                            (det_count        == prof_last_det_count);
// FIX210B_WR_ADDR_PIPE_NORMAL_ONLY: raw mux before registered bridge write stage.

// FT232H SPI frame receiver -> DDR input buffer.
// Packet format: RAW 256x256x3 payload only, no YSP1 header.
// Payload expected by spi_yolo_frame_loader is 256*256*3 = 196608 bytes.
// Drop whole SPI transactions while YOLO/DET/bridge is busy, so video frames
// cannot overwrite the DDR input while the core is detecting.
assign spi_accept_enable = (!y_busy) && (!det_writer_busy) && (!bridge_busy) &&
                           (!cam_busy) && (!cam_frame_loaded);

spi_yolo_frontend #(
    .ADDR_W(ADDR_W)
) u_spi_yolo_frontend (
    .clk(PCLK),
    .rst_n(core_rst_n),
    .clear(clear_all),
    .accept_enable(spi_accept_enable),

    .spi_sck(SPI_SCK),
    .spi_mosi(SPI_MOSI),
    .spi_cs_n(SPI_CS_N),

    .mem_wr_req(spi_mem_wr_req),
    .mem_wr_addr(spi_mem_wr_addr),
    .mem_wr_data(spi_mem_wr_data),
    .mem_wr_ready(spi_mem_wr_ready),

    .frame_loaded(spi_frame_loaded),
    .busy(spi_busy),
    .error(spi_error),
    .overrun_error(spi_overrun_error),
    .debug_state(spi_debug_state),
    .payload_count(spi_payload_count),
    .frames_loaded_count(spi_frames_loaded_count)
);

// IMX219 RAW10 -> edge -> 256x256x3 CHW byte writer.
// ENABLE_TEST_PATTERN=1 is intentional for first bring-up.  Set it to 0 only
// after chip ID, frame geometry and DDR payload have all passed.
assign cam_accept_enable = (!y_busy) && (!det_writer_busy) && (!bridge_busy) &&
                           (!spi_busy) && (!spi_mem_wr_req);

imx219_edge_yolo_frontend #(
    .ADDR_W(ADDR_W),
    .INPUT_BASE({ADDR_W{1'b0}}),
    .SYS_CLK_HZ(SYS_CLK_HZ),
    .SOBEL_THRESHOLD(11'd160),
    .ENABLE_TEST_PATTERN(1),
    .EDGE_ON_VALUE(8'd127)
) u_imx219_frontend (
    .clk(PCLK),
    .rst_n(core_rst_n),
    .clear(clear_all),
    .accept_enable(cam_accept_enable),

    .mipi_pixel_clk(MIPI_PIXEL_CLK),
    .mipi_raw10(MIPI_RAW10),
    .mipi_line_valid(MIPI_LINE_VALID),
    .mipi_line_start(MIPI_LINE_START),
    .mipi_line_end(MIPI_LINE_END),
    .mipi_frame_valid(MIPI_FRAME_VALID),
    .mipi_frame_start(MIPI_FRAME_START),
    .mipi_frame_end(MIPI_FRAME_END),
    .mipi_ebd_valid(MIPI_EBD_VALID),
    .mipi_data_type(MIPI_DATA_TYPE),
    .mipi_ecc_error(MIPI_ECC_ERROR),
    .mipi_crc_error(MIPI_CRC_ERROR),

    .cam_enable(CAM_ENABLE),
    .cam_gpio1(CAM_GPIO1),
    .cam_i2c_scl(CAM_I2C_SCL),
    .cam_i2c_sda(CAM_I2C_SDA),

    .mem_wr_req(cam_mem_wr_req),
    .mem_wr_addr(cam_mem_wr_addr),
    .mem_wr_data(cam_mem_wr_data),
    .mem_wr_ready(cam_mem_wr_ready),

    .frame_done_pulse(cam_frame_done_pulse),
    .frame_loaded(cam_frame_loaded),
    .busy(cam_busy),
    .error(cam_error),
    .overrun_error(cam_overrun_error),
    .payload_count(cam_payload_count),
    .frames_loaded_count(cam_frames_loaded_count),
    .debug_state(cam_debug_state),

    .cam_init_done(cam_init_done),
    .cam_init_busy(cam_init_busy),
    .cam_init_error(cam_init_error),
    .cam_chip_id(cam_chip_id),
    .cam_frame_count(cam_frame_count),
    .cam_pixel_count(cam_pixel_count),
    .cam_size_error(cam_size_error),
    .cam_ecc_error(cam_ecc_error),
    .cam_crc_error(cam_crc_error),
    .cam_last_frame_pixels(cam_last_frame_pixels),
    .cam_last_frame_info(cam_last_frame_info)
);

// Byte-write source priority: camera, SPI, detector, YOLO core.
assign raw_bridge_mem_wr_req  = cam_mem_wr_req ? 1'b1 :
                                spi_mem_wr_req ? 1'b1 :
                                (det_mem_wr_req ? 1'b1 : y_mem_wr_req);
assign raw_bridge_mem_wr_addr = cam_mem_wr_req ? cam_mem_wr_addr :
                                spi_mem_wr_req ? spi_mem_wr_addr :
                                (det_mem_wr_req ? det_mem_wr_addr : y_mem_wr_addr);
assign raw_bridge_mem_wr_data = cam_mem_wr_req ? cam_mem_wr_data :
                                spi_mem_wr_req ? spi_mem_wr_data :
                                (det_mem_wr_req ? det_mem_wr_data : y_mem_wr_data);

assign bridge_mem_wr_can_accept = (!bridge_mem_wr_req_r) || bridge_mem_wr_ready;

assign cam_mem_wr_ready = cam_mem_wr_req ? bridge_mem_wr_can_accept : 1'b0;
assign spi_mem_wr_ready = (!cam_mem_wr_req && spi_mem_wr_req) ? bridge_mem_wr_can_accept : 1'b0;
assign det_mem_wr_ready = (!cam_mem_wr_req && !spi_mem_wr_req && det_mem_wr_req) ?
                          bridge_mem_wr_can_accept : 1'b0;
assign y_mem_wr_ready   = (!cam_mem_wr_req && !spi_mem_wr_req && !det_mem_wr_req) ?
                          bridge_mem_wr_can_accept : 1'b0;

assign bridge_mem_wr_req  = bridge_mem_wr_req_r;
assign bridge_mem_wr_addr = bridge_mem_wr_addr_r;
assign bridge_mem_wr_data = bridge_mem_wr_data_r;

always @(posedge PCLK or negedge core_rst_n) begin
    if (!core_rst_n) begin
        bridge_mem_wr_req_r  <= 1'b0;
        bridge_mem_wr_addr_r <= {ADDR_W{1'b0}};
        bridge_mem_wr_data_r <= {DATA_W{1'b0}};
    end else if (bridge_mem_wr_can_accept) begin
        bridge_mem_wr_req_r  <= raw_bridge_mem_wr_req;
        bridge_mem_wr_addr_r <= raw_bridge_mem_wr_addr;
        bridge_mem_wr_data_r <= raw_bridge_mem_wr_data;
    end
end

// One-entry elastic buffer on the 64-bit write channel.  Besides preserving
// DET priority, this removes the long active-op/address-mux path into
// u_bridge/wr_axi_addr_r.  A source is acknowledged when its command is safely
// captured here; the buffer then holds address/data until the AXI response.
assign bridge_mem_wr64_can_accept = (!bridge_mem_wr64_req_r) || bridge_mem_wr64_ready;
assign bridge_mem_wr64_take_det   = det_mem_wr64_req;
assign bridge_mem_wr64_take_yolo  = (!det_mem_wr64_req) && y_mem_wr64_req;

assign det_mem_wr64_ready = bridge_mem_wr64_take_det && bridge_mem_wr64_can_accept;
assign y_mem_wr64_ready   = bridge_mem_wr64_take_yolo && bridge_mem_wr64_can_accept;

assign bridge_mem_wr64_req  = bridge_mem_wr64_req_r;
assign bridge_mem_wr64_addr = bridge_mem_wr64_addr_r;
assign bridge_mem_wr64_data = bridge_mem_wr64_data_r;

always @(posedge PCLK or negedge core_rst_n) begin
    if (!core_rst_n) begin
        bridge_mem_wr64_req_r  <= 1'b0;
        bridge_mem_wr64_addr_r <= {ADDR_W{1'b0}};
        bridge_mem_wr64_data_r <= 64'd0;
    end else begin
        if (bridge_mem_wr64_ready)
            bridge_mem_wr64_req_r <= 1'b0;

        // The second assignment intentionally wins when an old command
        // completes while a new command is accepted in the same cycle.
        if (bridge_mem_wr64_can_accept) begin
            if (bridge_mem_wr64_take_det) begin
                bridge_mem_wr64_req_r  <= 1'b1;
                bridge_mem_wr64_addr_r <= det_mem_wr64_addr;
                bridge_mem_wr64_data_r <= det_mem_wr64_data;
            end else if (bridge_mem_wr64_take_yolo) begin
                bridge_mem_wr64_req_r  <= 1'b1;
                bridge_mem_wr64_addr_r <= y_mem_wr64_addr;
                bridge_mem_wr64_data_r <= y_mem_wr64_data;
            end
        end
    end
end
// FIX164_PROFILER_COUNTERS_V2: profiling counters reset on reset/start/clear.
always @(posedge PCLK or negedge PRESETN) begin
    if (!PRESETN) begin
        prof_total_cycles       <= 32'd0;
        prof_rd_req_cycles      <= 32'd0;
        prof_rd_wait_cycles     <= 32'd0;
        prof_wr_req_cycles      <= 32'd0;
        prof_wr_wait_cycles     <= 32'd0;
        prof_core_active_cycles <= 32'd0;
        prof_bridge_busy_cycles <= 32'd0;
        prof_det_busy_cycles    <= 32'd0;
        prof_no_progress_cycles <= 32'd0;
        prof_last_rd_count      <= 32'd0;
        prof_last_wr_count      <= 32'd0;
        prof_last_det_count     <= 32'd0;
    end else begin
        if (start_pulse || clear_all) begin
            prof_total_cycles       <= 32'd0;
            prof_rd_req_cycles      <= 32'd0;
            prof_rd_wait_cycles     <= 32'd0;
            prof_wr_req_cycles      <= 32'd0;
            prof_wr_wait_cycles     <= 32'd0;
            prof_core_active_cycles <= 32'd0;
            prof_bridge_busy_cycles <= 32'd0;
            prof_det_busy_cycles    <= 32'd0;
            prof_no_progress_cycles <= 32'd0;
            prof_last_rd_count      <= bridge_rd_count;
            prof_last_wr_count      <= bridge_wr_count;
            prof_last_det_count     <= det_count;
        end else begin
            if (prof_any_busy)
                prof_total_cycles <= prof_total_cycles + 32'd1;
            if (y_busy && (y_mem_rd_req || y_mem_rd32_req || y_mem_rd64_req)) // FIX164_MAC8_VECTOR_PACK64
                prof_rd_req_cycles <= prof_rd_req_cycles + 32'd1;
            if (y_busy && ((y_mem_rd_req && !y_mem_rd_valid) || (y_mem_rd32_req && !y_mem_rd32_valid) || (y_mem_rd64_req && !y_mem_rd64_valid))) // FIX164_MAC8_VECTOR_PACK64
                prof_rd_wait_cycles <= prof_rd_wait_cycles + 32'd1;
            if (y_busy && (y_mem_wr_req || y_mem_wr64_req)) // FIX202_CAT2_RD64_WR64
                prof_wr_req_cycles <= prof_wr_req_cycles + 32'd1;
            if ((bridge_mem_wr_req && !bridge_mem_wr_ready) || (bridge_mem_wr64_req && !bridge_mem_wr64_ready)) // FIX202_CAT2_RD64_WR64
                prof_wr_wait_cycles <= prof_wr_wait_cycles + 32'd1;
            if (y_busy && !y_mem_rd_req && !y_mem_wr_req)
                prof_core_active_cycles <= prof_core_active_cycles + 32'd1;
            if (bridge_busy)
                prof_bridge_busy_cycles <= prof_bridge_busy_cycles + 32'd1;
            if (det_writer_busy)
                prof_det_busy_cycles <= prof_det_busy_cycles + 32'd1;
            if (prof_no_progress_now)
                prof_no_progress_cycles <= prof_no_progress_cycles + 32'd1;

            prof_last_rd_count  <= bridge_rd_count;
            prof_last_wr_count  <= bridge_wr_count;
            prof_last_det_count <= det_count;
        end
    end
end


always @(posedge PCLK or negedge PRESETN) begin
    if (!PRESETN) begin
        ddr_base <= 40'h1408_000000;
        det_base <= 32'h01c0_0000;
        done_latched <= 1'b0;
        soft_reset <= 1'b0;
        start_req <= 1'b0;
        clear_req <= 1'b0;
    end else begin
        start_req <= 1'b0;
        clear_req <= 1'b0;

        if (y_done)
            done_latched <= 1'b1;

        if (apb_write) begin
            case (PADDR[7:0])
                8'h00: begin
                    if (PWDATA[0]) begin
                        start_req <= 1'b1;
                        done_latched <= 1'b0;
                    end
                    if (PWDATA[1]) begin
                        clear_req <= 1'b1;
                        done_latched <= 1'b0;
                    end
                    soft_reset <= PWDATA[2];
                end
                8'h08: ddr_base[31:0] <= PWDATA;
                8'h0c: ddr_base[AXI_ADDR_WIDTH-1:32] <= PWDATA[AXI_ADDR_WIDTH-33:0];
                8'h10: det_base <= PWDATA[ADDR_W-1:0];
                default: begin end
            endcase
        end
    end
end

always @(*) begin
    prdata_comb = 32'd0; // FIX203_COMBO_MARGIN_K1RD64
    if (PADDR[11]) begin // FIX188B_PER_OP_PROFILER: 0x800..0xfff per-op profiler window
        case (PADDR[3:2])
            2'd0: prdata_comb = prof_op_cycles[PADDR[10:4]];
            2'd1: prdata_comb = prof_op_act_wait[PADDR[10:4]];
            2'd2: prdata_comb = prof_op_rd8_wait[PADDR[10:4]];
            default: prdata_comb = prof_op_wr_wait[PADDR[10:4]];
        endcase
    end else begin
    case (PADDR[11:0]) // FIX186_DET_PROFILER: allow extended APB debug offsets
        8'h00: prdata_comb = {29'd0, soft_reset, 2'b00};
        8'h04: prdata_comb = {23'd0, frontend_overrun, frontend_error, frontend_busy, frontend_frame_loaded, det_overflow, bridge_busy, bridge_error, done_latched, y_busy};
        8'h08: prdata_comb = ddr_base[31:0];
        8'h0c: prdata_comb = {{(32-(AXI_ADDR_WIDTH-32)){1'b0}}, ddr_base[AXI_ADDR_WIDTH-1:32]};
        8'h10: prdata_comb = det_base;
        8'h14: prdata_comb = 32'd0;
        8'h18: prdata_comb = det_count;
        8'h1c: prdata_comb = {26'd0, debug_op_id};
        8'h20: prdata_comb = bridge_rd_count;
        8'h24: prdata_comb = bridge_wr_count;
        12'h0f8: prdata_comb = {8'd0, spi_debug_state, spi_payload_count[15:0]};
        12'h0fc: prdata_comb = spi_frames_loaded_count;
        8'h28: prdata_comb = 32'h4D43_0000 | MAC_LANES; // FIX164_PROFILER_COUNTERS_V2
        8'h2c: prdata_comb = 32'h4443_0000 | DET_CH;    // FIX164_PROFILER_COUNTERS_V2
        8'h30: prdata_comb = prof_total_cycles;          // FIX164_PROFILER_COUNTERS_V2
        8'h34: prdata_comb = prof_rd_req_cycles;         // FIX164_PROFILER_COUNTERS_V2
        8'h38: prdata_comb = prof_rd_wait_cycles;        // FIX164_PROFILER_COUNTERS_V2
        8'h3c: prdata_comb = prof_wr_req_cycles;         // FIX164_PROFILER_COUNTERS_V2
        8'h40: prdata_comb = prof_wr_wait_cycles;        // FIX164_PROFILER_COUNTERS_V2
        8'h44: prdata_comb = prof_core_active_cycles;    // FIX164_PROFILER_COUNTERS_V2
        8'h48: prdata_comb = prof_bridge_busy_cycles;    // FIX164_PROFILER_COUNTERS_V2
        8'h4c: prdata_comb = prof_det_busy_cycles;       // FIX164_PROFILER_COUNTERS_V2
        8'h50: prdata_comb = prof_no_progress_cycles;    // FIX164_PROFILER_COUNTERS_V2
        8'h54: prdata_comb = {21'd0, debug_op_id, y_conv_prof_state}; // FIX172_FULL_MAC32_REAL
        8'h58: prdata_comb = prof_st_bias_req;       // FIX164_MAC16_DETAILED_PROFILER
        8'h5c: prdata_comb = prof_st_bias_wait;      // FIX164_MAC16_DETAILED_PROFILER
        8'h60: prdata_comb = prof_st_bias_check;     // FIX164_MAC16_DETAILED_PROFILER
        8'h64: prdata_comb = prof_st_w_req;          // FIX164_MAC16_DETAILED_PROFILER
        8'h68: prdata_comb = prof_st_w_wait;         // FIX164_MAC16_DETAILED_PROFILER
        8'h6c: prdata_comb = prof_st_w64_req;        // FIX164_MAC16_DETAILED_PROFILER
        8'h70: prdata_comb = prof_st_w64_wait;       // FIX164_MAC16_DETAILED_PROFILER
        8'h74: prdata_comb = prof_st_act_req;        // FIX164_MAC16_DETAILED_PROFILER
        8'h78: prdata_comb = prof_st_act_wait;       // FIX164_MAC16_DETAILED_PROFILER
        8'h7c: prdata_comb = prof_st_mac;            // FIX164_MAC16_DETAILED_PROFILER
        8'h80: prdata_comb = prof_st_write;          // FIX164_MAC16_DETAILED_PROFILER
        8'h84: prdata_comb = prof_st_other;          // FIX164_MAC16_DETAILED_PROFILER
        8'h88: prdata_comb = prof_rd8_req_cycles;   // FIX164_MAC16_DETAILED_PROFILER
        8'h8c: prdata_comb = prof_rd8_wait_cycles;  // FIX164_MAC16_DETAILED_PROFILER
        8'h90: prdata_comb = prof_rd32_req_cycles;  // FIX164_MAC16_DETAILED_PROFILER
        8'h94: prdata_comb = prof_rd32_wait_cycles; // FIX164_MAC16_DETAILED_PROFILER
        8'h98: prdata_comb = prof_rd64_req_cycles;  // FIX164_MAC16_DETAILED_PROFILER
        8'h9c: prdata_comb = prof_rd64_wait_cycles; // FIX164_MAC16_DETAILED_PROFILER
        8'he0: prdata_comb = prof_st_write_prep;        // FIX165_PROF_5BIT_WCACHE
        8'he4: prdata_comb = prof_st_wcache_check;      // FIX165_PROF_5BIT_WCACHE
        8'he8: prdata_comb = prof_st_wcache_load_req;   // FIX165_PROF_5BIT_WCACHE
        8'hec: prdata_comb = prof_st_wcache_load_wait;  // FIX165_PROF_5BIT_WCACHE
        8'hf0: prdata_comb = prof_st_wcache_use_req;    // FIX165_PROF_5BIT_WCACHE
        8'hf4: prdata_comb = prof_st_wcache_use_wait;   // FIX165_PROF_5BIT_WCACHE
        8'ha0: prdata_comb = {15'd0, op_prof_active, op_prof_cur_id, y_conv_prof_state}; // FIX164_MAC16_OP_PROFILER
        8'ha4: prdata_comb = op_prof_cur_cycles;      // FIX164_MAC16_OP_PROFILER
        8'ha8: prdata_comb = op_prof_cur_w64_wait;    // FIX164_MAC16_OP_PROFILER
        8'hac: prdata_comb = op_prof_cur_act_wait;    // FIX164_MAC16_OP_PROFILER
        8'hb0: prdata_comb = op_prof_cur_rd8_wait;    // FIX164_MAC16_OP_PROFILER
        8'hb4: prdata_comb = op_prof_cur_rd64_wait;   // FIX164_MAC16_OP_PROFILER
        8'hb8: prdata_comb = {16'd0, op_prof_last_id, 4'd0}; // FIX164_MAC16_OP_PROFILER
        8'hbc: prdata_comb = op_prof_last_cycles;     // FIX164_MAC16_OP_PROFILER
        8'hc0: prdata_comb = op_prof_last_w64_wait;   // FIX164_MAC16_OP_PROFILER
        8'hc4: prdata_comb = op_prof_last_act_wait;   // FIX164_MAC16_OP_PROFILER
        8'hc8: prdata_comb = op_prof_last_rd8_wait;   // FIX164_MAC16_OP_PROFILER
        8'hcc: prdata_comb = op_prof_last_rd64_wait;  // FIX164_MAC16_OP_PROFILER
        8'hd0: prdata_comb = {16'd0, op_prof_max_id, 4'd0}; // FIX164_MAC16_OP_PROFILER
        8'hd4: prdata_comb = op_prof_max_cycles;      // FIX164_MAC16_OP_PROFILER
        8'hd8: prdata_comb = op_prof_max_w64_wait;    // FIX164_MAC16_OP_PROFILER
        8'hdc: prdata_comb = op_prof_max_act_wait;    // FIX164_MAC16_OP_PROFILER
        12'h100: prdata_comb = prof_det_total_cycles;      // FIX186_DET_PROFILER
        12'h104: prdata_comb = prof_det_s_cycles;          // FIX186_DET_PROFILER
        12'h108: prdata_comb = prof_det_m_cycles;          // FIX186_DET_PROFILER
        12'h10c: prdata_comb = prof_det_l_cycles;          // FIX186_DET_PROFILER
        12'h110: prdata_comb = prof_det_bias_cycles;       // FIX186_DET_PROFILER
        12'h114: prdata_comb = prof_det_wcache_cycles;     // FIX186_DET_PROFILER
        12'h118: prdata_comb = prof_det_pixel_init_cycles; // FIX186_DET_PROFILER
        12'h11c: prdata_comb = prof_det_act_req_cycles;    // FIX186_DET_PROFILER
        12'h120: prdata_comb = prof_det_act_wait_cycles;   // FIX186_DET_PROFILER
        12'h124: prdata_comb = prof_det_oc_mac_cycles;     // FIX186_DET_PROFILER
        12'h128: prdata_comb = prof_det_out_cycles;        // FIX186_DET_PROFILER
        12'h12c: prdata_comb = prof_det_gap_cycles;        // FIX186_DET_PROFILER
        12'h130: prdata_comb = prof_det_raw_valid_cycles;  // FIX186_DET_PROFILER
        12'h134: prdata_comb = prof_det_raw_busy_cycles;   // FIX186_DET_PROFILER
        12'h138: prdata_comb = prof_det_wr_req_cycles;     // FIX186_DET_PROFILER
        12'h13c: prdata_comb = prof_det_wr_wait_cycles;    // FIX186_DET_PROFILER
        // IMX219 frontend debug window.  These offsets do not overlap the
        // existing detector profiler or 0x800..0xfff per-op profiler window.
        12'h180: prdata_comb = {18'd0,
                                cam_size_error,
                                cam_crc_error,
                                cam_ecc_error,
                                cam_overrun_error,
                                cam_error,
                                cam_busy,
                                cam_frame_loaded,
                                cam_init_error,
                                cam_init_busy,
                                cam_init_done,
                                4'd0};
        12'h184: prdata_comb = {16'd0, cam_chip_id};
        12'h188: prdata_comb = cam_frame_count;
        12'h18c: prdata_comb = cam_pixel_count;
        12'h190: prdata_comb = cam_payload_count;
        12'h194: prdata_comb = cam_frames_loaded_count;
        12'h198: prdata_comb = {16'd0, cam_debug_state, 8'd0};
        // FRAME_AXI_FIX_V2: no old offsets or external ports changed.
        12'h19c: prdata_comb = 32'h46415832; // ASCII FAX2 build marker
        12'h1a0: prdata_comb = bridge_debug_status;
        12'h1a4: prdata_comb = bridge_first_error_addr[31:0];
        12'h1a8: prdata_comb = bridge_first_error_addr >> 32;
        12'h1ac: prdata_comb = bridge_last_write_addr[31:0];
        12'h1b0: prdata_comb = bridge_last_write_addr >> 32;
        12'h1b4: prdata_comb = cam_last_frame_pixels;
        12'h1b8: prdata_comb = cam_last_frame_info;
        12'h1c0: prdata_comb = link_live;
        12'h1c4: prdata_comb = link_seen_high;
        12'h1c8: prdata_comb = link_taps;
        12'h1cc: prdata_comb = link_byte_beats;
        12'h1d0: prdata_comb = link_pixel_beats;
        12'h1d4: prdata_comb = 32'h4d444232; // MDB2
        12'h1d8: prdata_comb = LINK_WINDOW;
        12'h1dc: prdata_comb = link_seen_low;
        12'h1e0: prdata_comb = 32'h534e4150; // reading freezes snapshot bank
        12'h200: prdata_comb = link_frozen[0 +: 32];
        12'h204: prdata_comb = link_frozen[32 +: 32];
        12'h208: prdata_comb = link_frozen[64 +: 32];
        12'h20c: prdata_comb = link_frozen[96 +: 32];
        12'h210: prdata_comb = link_frozen[128 +: 32];
        12'h214: prdata_comb = link_frozen[160 +: 32];
        12'h218: prdata_comb = link_frozen[192 +: 32];
        12'h21c: prdata_comb = link_frozen[224 +: 32];
        12'h220: prdata_comb = link_frozen[256 +: 32];
        12'h224: prdata_comb = link_frozen[288 +: 32];
        12'h228: prdata_comb = link_frozen[320 +: 32];
        12'h22c: prdata_comb = link_frozen[352 +: 32];
        12'h230: prdata_comb = link_frozen[384 +: 32];
        12'h234: prdata_comb = link_frozen[416 +: 32];
        12'h238: prdata_comb = link_frozen[448 +: 32];
        12'h23c: prdata_comb = link_frozen[480 +: 32];
        // CSI_FRAME_EVENTS_V3, additional offsets; old map is preserved.
        12'h240: prdata_comb = 32'h43534933; // CSI3
        12'h244: prdata_comb = csi_live;
        12'h248: prdata_comb = {31'd0, csi_snapshot_pending};
        12'h24c: prdata_comb = csi_sequence;
        12'h250: prdata_comb = csi_frozen[0 +: 32];
        12'h254: prdata_comb = csi_frozen[32 +: 32];
        12'h258: prdata_comb = csi_frozen[64 +: 32];
        12'h25c: prdata_comb = csi_frozen[96 +: 32];
        12'h260: prdata_comb = csi_frozen[128 +: 32];
        12'h264: prdata_comb = csi_frozen[160 +: 32];
        12'h268: prdata_comb = csi_frozen[192 +: 32];
        12'h26c: prdata_comb = csi_frozen[224 +: 32];
        12'h270: prdata_comb = csi_frozen[256 +: 32];
        12'h274: prdata_comb = csi_frozen[288 +: 32];
        12'h278: prdata_comb = csi_frozen[320 +: 32];
        12'h27c: prdata_comb = csi_frozen[352 +: 32];
        12'h280: prdata_comb = csi_frozen[384 +: 32];
        12'h284: prdata_comb = csi_frozen[416 +: 32];
        12'h288: prdata_comb = csi_frozen[448 +: 32];
        12'h28c: prdata_comb = csi_frozen[480 +: 32];
        12'h290: prdata_comb = csi_loss_in_frame;
        12'h294: prdata_comb = 32'h53455231; // SER1: serialized CSI debug

        default: prdata_comb = 32'd0;
    endcase
    end
end
// FIX203_COMBO_MARGIN_K1RD64: register APB read data to break long PRDATA path.
always @(posedge PCLK or negedge PRESETN) begin
    if (!PRESETN) begin
        PRDATA <= 32'd0;
    end else if (apb_read) begin
        PRDATA <= prdata_comb;
    end
end



top_yolov5n_ddr #( // FIX219_SINGLE_CORE_2STATE_RUNTIME_STAGE
    .DATA_W(DATA_W),
    .ADDR_W(ADDR_W),
    .ACC_W(ACC_W),
    .DET_CH(DET_CH),
    .MAC_LANES(MAC_LANES)
) u_yolo (
    .clk(PCLK),
    .rst_n(core_rst_n),
    .start(sc_core_start),
    .run_start_op(sc_run_start_op),
    .run_end_op(sc_run_end_op),
    .mem_rd_req(y_mem_rd_req),
    .mem_rd_addr(y_mem_rd_addr),
    .mem_rd_ready(y_mem_rd_ready),
    .mem_rd_valid(y_mem_rd_valid),
    .mem_rd_data(y_mem_rd_data),
    .mem_rd32_req(y_mem_rd32_req),
    .mem_rd32_addr(y_mem_rd32_addr),
    .mem_rd32_ready(y_mem_rd32_ready),
    .mem_rd32_valid(y_mem_rd32_valid),
    .mem_rd32_data(y_mem_rd32_data), // FIX164_MAC4_WEIGHT_PACK32
    .mem_rd64_req(y_mem_rd64_req),
    .mem_rd64_addr(y_mem_rd64_addr),
    .mem_rd64_ready(y_mem_rd64_ready),
    .mem_rd64_valid(y_mem_rd64_valid),
    .mem_rd64_data(y_mem_rd64_data), // FIX164_MAC8_VECTOR_PACK64
    .mem_wr_req(y_mem_wr_req),
    .mem_wr_addr(y_mem_wr_addr),
    .mem_wr_data(y_mem_wr_data),
    .mem_wr_ready(y_mem_wr_ready),
      .mem_wr64_req(y_mem_wr64_req), // FIX202_CAT2_RD64_WR64
      .mem_wr64_addr(y_mem_wr64_addr), // FIX202_CAT2_RD64_WR64
      .mem_wr64_data(y_mem_wr64_data), // FIX202_CAT2_RD64_WR64
      .mem_wr64_ready(y_mem_wr64_ready), // FIX202_CAT2_RD64_WR64
    .busy(y_core_busy),
    .done(y_core_done),
    .det_valid(det_valid),
    .det_branch(det_branch),
    .det_index(det_index),
    .det_data(det_data),
    .det_ready(det_writer_ready),
    .debug_op_id(debug_op_id),
    .prof_op_id(y_prof_op_id), // FIX164_MAC16_OP_PROFILER
    .conv_prof_state(y_conv_prof_state), // FIX164_MAC16_DETAILED_PROFILER
    .det_prof_state(y_det_prof_state) // FIX186_DET_PROFILER
);

detect_output_mem_writer #(
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W),
    .ACC_W(ACC_W),
    .DEFAULT_DET_BASE(32'h01c0_0000)
) u_det_writer (
    .clk(PCLK),
    .rst_n(core_rst_n),
    .clear(clear_all),
    .det_base(det_base),
    .det_valid(det_valid),
    .det_branch(det_branch),
    .det_index(det_index),
    .det_data(det_data),
    .mem_wr_req(det_mem_wr_req),
    .mem_wr_addr(det_mem_wr_addr),
    .mem_wr_data(det_mem_wr_data),
    .mem_wr_ready(det_mem_wr_ready),
    .mem_wr64_req(det_mem_wr64_req), // FIX187E_DET_RECORD_WR64_COMPLETE
    .mem_wr64_addr(det_mem_wr64_addr), // FIX187E_DET_RECORD_WR64_COMPLETE
    .mem_wr64_data(det_mem_wr64_data), // FIX187E_DET_RECORD_WR64_COMPLETE
    .mem_wr64_ready(det_mem_wr64_ready), // FIX187E_DET_RECORD_WR64_COMPLETE
    .det_count(det_count),
    .overflow(det_overflow),
    .busy(det_writer_busy),
    .ready(det_writer_ready)
);

yolo_mem_axi_bridge #(
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W)
) u_bridge (
    .clk(PCLK),
    .rst_n(core_rst_n),
    .ddr_base(ddr_base),
    .mem_rd_req(y_mem_rd_req),
    .mem_rd_addr(y_mem_rd_addr),
    .mem_rd_ready(y_mem_rd_ready),
    .mem_rd_valid(y_mem_rd_valid),
    .mem_rd_data(y_mem_rd_data),
    .mem_rd32_req(y_mem_rd32_req),
    .mem_rd32_addr(y_mem_rd32_addr),
    .mem_rd32_ready(y_mem_rd32_ready),
    .mem_rd32_valid(y_mem_rd32_valid),
    .mem_rd32_data(y_mem_rd32_data), // FIX164_MAC4_WEIGHT_PACK32
    .mem_rd64_req(y_mem_rd64_req),
    .mem_rd64_addr(y_mem_rd64_addr),
    .mem_rd64_ready(y_mem_rd64_ready),
    .mem_rd64_valid(y_mem_rd64_valid),
    .mem_rd64_data(y_mem_rd64_data), // FIX164_MAC8_VECTOR_PACK64
    .mem_wr_req(bridge_mem_wr_req),
    .mem_wr_addr(bridge_mem_wr_addr),
    .mem_wr_data(bridge_mem_wr_data),
    .mem_wr_ready(bridge_mem_wr_ready),
    .mem_wr64_req(bridge_mem_wr64_req), // FIX187E_DET_RECORD_WR64_COMPLETE
    .mem_wr64_addr(bridge_mem_wr64_addr), // FIX187E_DET_RECORD_WR64_COMPLETE
    .mem_wr64_data(bridge_mem_wr64_data), // FIX187E_DET_RECORD_WR64_COMPLETE
    .mem_wr64_ready(bridge_mem_wr64_ready), // FIX187E_DET_RECORD_WR64_COMPLETE
    .busy(bridge_busy),
    .error(bridge_error),
    .rd_count(bridge_rd_count),
    .wr_count(bridge_wr_count),
    .debug_status(bridge_debug_status),
    .first_error_addr(bridge_first_error_addr),
    .last_write_addr(bridge_last_write_addr),
    .M_AXI_AWADDR(M_AXI_AWADDR),
    .M_AXI_AWLEN(M_AXI_AWLEN),
    .M_AXI_AWSIZE(M_AXI_AWSIZE),
    .M_AXI_AWBURST(M_AXI_AWBURST),
    .M_AXI_AWLOCK(M_AXI_AWLOCK),
    .M_AXI_AWCACHE(M_AXI_AWCACHE),
    .M_AXI_AWPROT(M_AXI_AWPROT),
    .M_AXI_AWQOS(M_AXI_AWQOS),
    .M_AXI_AWVALID(M_AXI_AWVALID),
    .M_AXI_AWREADY(M_AXI_AWREADY),
    .M_AXI_WDATA(M_AXI_WDATA),
    .M_AXI_WSTRB(M_AXI_WSTRB),
    .M_AXI_WLAST(M_AXI_WLAST),
    .M_AXI_WVALID(M_AXI_WVALID),
    .M_AXI_WREADY(M_AXI_WREADY),
    .M_AXI_BRESP(M_AXI_BRESP),
    .M_AXI_BVALID(M_AXI_BVALID),
    .M_AXI_BREADY(M_AXI_BREADY),
    .M_AXI_ARADDR(M_AXI_ARADDR),
    .M_AXI_ARLEN(M_AXI_ARLEN),
    .M_AXI_ARSIZE(M_AXI_ARSIZE),
    .M_AXI_ARBURST(M_AXI_ARBURST),
    .M_AXI_ARLOCK(M_AXI_ARLOCK),
    .M_AXI_ARCACHE(M_AXI_ARCACHE),
    .M_AXI_ARPROT(M_AXI_ARPROT),
    .M_AXI_ARQOS(M_AXI_ARQOS),
    .M_AXI_ARVALID(M_AXI_ARVALID),
    .M_AXI_ARREADY(M_AXI_ARREADY),
    .M_AXI_RDATA(M_AXI_RDATA),
    .M_AXI_RRESP(M_AXI_RRESP),
    .M_AXI_RLAST(M_AXI_RLAST),
    .M_AXI_RVALID(M_AXI_RVALID),
    .M_AXI_RREADY(M_AXI_RREADY)
);

endmodule

// Diagnostic heartbeat: bit 7 changes every 128 source-clock rising edges.
// Two-stage synchronizers receive these slow toggles; no source clock is gated.
// Counters report heartbeat changes during WINDOW_CYCLES system clocks.
// They measure activity, not a calibrated frequency. Avoid interpreting pulses
// shorter than two sys-clock periods as guaranteed captured status events.
module mipi_link_debug_v1 #(
    parameter integer WINDOW_CYCLES = 75000
)(
    input wire sys_clk, rst_n, byte_clk, pixel_clk,
    input wire [6:0] flags_async,
    input wire [15:0] taps_async,
    output wire [31:0] live, seen_high, seen_low, taps,
    output reg [31:0] byte_beats, pixel_beats,
    output wire [1:0] beat_edges
);
    localparam [31:0] LAST_WINDOW_CYCLE = WINDOW_CYCLES - 1;
    reg [7:0] byte_div, pixel_div;
    always @(posedge byte_clk or negedge rst_n)
        if (!rst_n) byte_div <= 0; else byte_div <= byte_div + 1'b1;
    always @(posedge pixel_clk or negedge rst_n)
        if (!rst_n) pixel_div <= 0; else pixel_div <= pixel_div + 1'b1;
    (* syn_async_reg = 1 *) reg [6:0] flags_s1, flags_s2;
    (* syn_async_reg = 1 *) reg [15:0] taps_s1, taps_s2;
    (* syn_async_reg = 1 *) reg [1:0] beat_s1, beat_s2;
    reg [1:0] beat_previous, recent, ever;
    reg [6:0] high_flags, low_flags;
    reg [31:0] window_count, byte_acc, pixel_acc;
    wire [1:0] changed = beat_s2 ^ beat_previous;
    assign beat_edges = changed;
    assign live = {23'd0, recent, flags_s2};
    assign seen_high = {23'd0, ever, high_flags};
    assign seen_low = {25'd0, low_flags};
    assign taps = {16'd0, taps_s2};
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            flags_s1 <= 0; flags_s2 <= 0; taps_s1 <= 0; taps_s2 <= 0;
            beat_s1 <= 0; beat_s2 <= 0; beat_previous <= 0;
            recent <= 0; ever <= 0; high_flags <= 0; low_flags <= 0;
            window_count <= 0; byte_acc <= 0; pixel_acc <= 0;
            byte_beats <= 0; pixel_beats <= 0;
        end else begin
            flags_s1 <= flags_async; flags_s2 <= flags_s1;
            taps_s1 <= taps_async; taps_s2 <= taps_s1;
            beat_s1 <= {pixel_div[7], byte_div[7]}; beat_s2 <= beat_s1;
            beat_previous <= beat_s2;
            high_flags <= high_flags | flags_s2;
            low_flags <= low_flags | ~flags_s2;
            ever <= ever | changed;
            if (window_count == LAST_WINDOW_CYCLE) begin
                byte_beats <= byte_acc + {31'd0, changed[0]};
                pixel_beats <= pixel_acc + {31'd0, changed[1]};
                recent[0] <= (byte_acc != 0) || changed[0];
                recent[1] <= (pixel_acc != 0) || changed[1];
                window_count <= 0; byte_acc <= 0; pixel_acc <= 0;
            end else begin
                window_count <= window_count + 1'b1;
                byte_acc <= byte_acc + {31'd0, changed[0]};
                pixel_acc <= pixel_acc + {31'd0, changed[1]};
            end
        end
    end
endmodule

// All inputs are already synchronized into sys_clk by mipi_link_debug_v1.
// Read SNAP_TRIGGER (APB 0x1e0) before reading the frozen 0x200..0x23c bank.
module mipi_link_events_v2 #(
    parameter integer GAP_CYCLES = 543
)(
    input wire sys_clk, rst_n, snapshot,
    input wire [31:0] live, taps, byte_beats, pixel_beats,
    input wire byte_edge, pixel_edge,
    output reg [511:0] frozen
);
    localparam [31:0] GAP_THRESHOLD = GAP_CYCLES;
    reg lock_previous, lock_seen, byte_seen, pixel_seen, first_loss_valid;
    reg [31:0] loss_count, byte_gap_count;
    reg [31:0] byte_idle, pixel_idle, byte_max, pixel_max;
    reg [31:0] first_byte_idle, first_pixel_idle, first_live, first_taps;
    reg [31:0] sequence_count;
    wire falling_lock = lock_previous && !live[2];
    wire [31:0] next_byte_idle = byte_edge ? 32'd0 :
        (byte_seen ? sat_inc(byte_idle) : 32'd0);
    wire [31:0] next_pixel_idle = pixel_edge ? 32'd0 :
        (pixel_seen ? sat_inc(pixel_idle) : 32'd0);
    function [31:0] sat_inc;
        input [31:0] value;
        begin sat_inc = (&value) ? value : value + 1'b1; end
    endfunction
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            lock_previous <= 0; lock_seen <= 0;
            byte_seen <= 0; pixel_seen <= 0; first_loss_valid <= 0;
            loss_count <= 0; byte_gap_count <= 0;
            byte_idle <= 0; pixel_idle <= 0; byte_max <= 0; pixel_max <= 0;
            first_byte_idle <= 0; first_pixel_idle <= 0;
            first_live <= 0; first_taps <= 0;
            sequence_count <= 0; frozen <= 0;
        end else begin
            lock_previous <= live[2];
            if (live[2]) lock_seen <= 1;
            if (byte_edge) byte_seen <= 1;
            if (pixel_edge) pixel_seen <= 1;
            byte_idle <= next_byte_idle;
            pixel_idle <= next_pixel_idle;
            if (next_byte_idle > byte_max) byte_max <= next_byte_idle;
            if (next_pixel_idle > pixel_max) pixel_max <= next_pixel_idle;
            if (byte_seen && !byte_edge && next_byte_idle == GAP_THRESHOLD)
                byte_gap_count <= sat_inc(byte_gap_count);
            if (falling_lock && lock_seen) begin
                loss_count <= sat_inc(loss_count);
                if (!first_loss_valid) begin
                    first_loss_valid <= 1;
                    first_byte_idle <= next_byte_idle;
                    first_pixel_idle <= next_pixel_idle;
                    first_live <= live;
                    first_taps <= taps;
                end
            end
            // One coherent PCLK snapshot; reads of frozen words never update it.
            // An event on this same edge is visible in the next snapshot.
            if (snapshot) begin
                sequence_count <= sequence_count + 1'b1;
                frozen[0*32 +: 32] <= {15'd0,live[8:0],4'd0,
                    pixel_seen,byte_seen,first_loss_valid,lock_seen};
                frozen[1*32 +: 32] <= loss_count;
                frozen[2*32 +: 32] <= byte_gap_count;
                frozen[3*32 +: 32] <= byte_max;
                frozen[4*32 +: 32] <= byte_idle;
                frozen[5*32 +: 32] <= pixel_max;
                frozen[6*32 +: 32] <= pixel_idle;
                frozen[7*32 +: 32] <= first_byte_idle;
                frozen[8*32 +: 32] <= first_pixel_idle;
                frozen[9*32 +: 32] <= first_live;
                frozen[10*32 +: 32] <= first_taps;
                frozen[11*32 +: 32] <= byte_beats;
                frozen[12*32 +: 32] <= pixel_beats;
                frozen[13*32 +: 32] <= taps;
                frozen[14*32 +: 32] <= sequence_count + 1'b1;
                frozen[15*32 +: 32] <= GAP_CYCLES;
            end
        end
    end
endmodule

`timescale 1ns/1ps
// Diagnostic counters in the native pixel domain, then a request/ack mailbox.
// Counters/metadata in the frozen bank belong to one pixel-clock edge.
`timescale 1ns/1ps
// CSI3_SERIAL_ROUTING_FIX: freeze a native snapshot, then transfer sixteen
// 32-bit words using request/ack. Only 32 data wires cross clock domains.
// The previous published bank stays intact until the full transfer completes.
module mipi_decoder_debug_v3 (
    input wire sys_clk, pixel_clk, rst_n, snapshot,
    input wire fs, fe, ls, le, fv, lv, ebd, ecc, crc,
    input wire [7:0] dt, vc,
    input wire [15:0] wc,
    input wire pll_lock,
    output wire [31:0] live_flags,
    output wire pending,
    output reg [31:0] sequence_count, loss_in_frame,
    output reg [511:0] frozen
);
    reg fs_d, fe_d, ls_d, le_d, ecc_d, crc_d;
    reg [15:0] fs_count, fe_count, ls_count, le_count, ecc_count, crc_count;
    reg [31:0] lv_cycles, ebd_cycles, raw_cycles;
    reg [31:0] last_packet, last_raw_packet;
    reg [15:0] line_pixels, last_line_pixels;
    reg [31:0] frame_pixels, last_frame_pixels;
    wire raw_valid = lv && !ebd && dt[5:0] == 6'h2b;
    wire fs_edge = fs && !fs_d;
    wire fe_edge = fe && !fe_d;
    wire ls_edge = ls && !ls_d;
    wire le_edge = le && !le_d;
    wire [9:0] native_flags = {raw_valid,crc,ecc,ebd,fe,fs,le,ls,lv,fv};
    reg request_toggle, ack_toggle;
    (* syn_async_reg = 1 *) reg req_s1, req_s2;
    reg [3:0] word_index;
    reg [3:0] index_s1, index_s2;
    reg req_seen;
    reg [511:0] native_snapshot;
    reg [31:0] word_mailbox;
    function [31:0] inc;
        input [31:0] x;
        begin inc = (&x) ? x : x + 1'b1; end
    endfunction
    function [15:0] inc16;
        input [15:0] x;
        begin inc16 = (&x) ? x : x + 1'b1; end
    endfunction
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            fs_d <= 0; fe_d <= 0; ls_d <= 0; le_d <= 0; ecc_d <= 0; crc_d <= 0;
            fs_count <= 0; fe_count <= 0; ls_count <= 0; le_count <= 0;
            lv_cycles <= 0; ebd_cycles <= 0; ecc_count <= 0; crc_count <= 0;
            raw_cycles <= 0; last_packet <= 0; last_raw_packet <= 0;
            line_pixels <= 0; frame_pixels <= 0;
            last_line_pixels <= 0; last_frame_pixels <= 0;
            req_s1 <= 0; req_s2 <= 0; req_seen <= 0;
            index_s1 <= 0; index_s2 <= 0;
            ack_toggle <= 0; native_snapshot <= 0; word_mailbox <= 0;
        end else begin
            fs_d <= fs; fe_d <= fe; ls_d <= ls; le_d <= le;
            ecc_d <= ecc; crc_d <= crc;
            req_s1 <= request_toggle; req_s2 <= req_s1;
            index_s1 <= word_index; index_s2 <= index_s1;
            if (fs_edge) fs_count <= inc16(fs_count);
            if (fe_edge) fe_count <= inc16(fe_count);
            if (ls_edge) ls_count <= inc16(ls_count);
            if (le_edge) le_count <= inc16(le_count);
            if (lv) lv_cycles <= inc(lv_cycles);
            if (ebd) ebd_cycles <= inc(ebd_cycles);
            if (ecc && !ecc_d) ecc_count <= inc16(ecc_count);
            if (crc && !crc_d) crc_count <= inc16(crc_count);
            // DATA_TYPE / WORD_COUNT / VC are a native-domain tuple.
            if (lv || ebd || fs_edge || fe_edge || ls_edge || le_edge)
                last_packet <= {wc,vc,dt};
            if (raw_valid) begin
                raw_cycles <= inc(raw_cycles);
                last_raw_packet <= {wc,vc,dt};
            end
            if (fs_edge) frame_pixels <= raw_valid ? 32'd1 : 32'd0;
            else if (raw_valid) frame_pixels <= inc(frame_pixels);
            if (ls_edge) line_pixels <= raw_valid ? 16'd1 : 16'd0;
            else if (raw_valid) line_pixels <= inc16(line_pixels);
            if (le_edge) last_line_pixels <= raw_valid ? inc16(line_pixels) : line_pixels;
            if (fe_edge) last_frame_pixels <= raw_valid ? inc(frame_pixels) : frame_pixels;
            if (req_s2 != req_seen) begin
                req_seen <= req_s2;
                if (index_s2 == 0) begin
                    // Capture once; counters continue running throughout the
                    // transfer. High halves of narrow counters are constants.
                    native_snapshot <= {
                        last_frame_pixels, {16'd0,last_line_pixels}, raw_cycles,
                        last_raw_packet, last_packet,
                        {16'd0,crc_count}, {16'd0,ecc_count},
                        ebd_cycles, lv_cycles, {16'd0,le_count},
                        {16'd0,ls_count}, {16'd0,fe_count}, {16'd0,fs_count},
                        {22'd0,native_flags}, 32'h43534933, 32'd0
                    };
                    word_mailbox <= 0; // word 0 is assigned by the system
                end else begin
                    word_mailbox <= native_snapshot[index_s2*32 +: 32];
                end
                ack_toggle <= req_s2;
            end
        end
    end
    // Each word/index is bundled data: it is held until its synchronized
    // handshake completes. The data has two stages, the ack has three.
    (* syn_async_reg = 1 *) reg [2:0] ack_sync;
    (* syn_async_reg = 1 *) reg [9:0] flags_s1, flags_s2;
    reg [31:0] bus_s1, bus_s2;
    reg [511:0] staging;
    reg ack_seen, pll_d, transfer_active;
    assign pending = transfer_active;
    assign live_flags = {21'd0,pll_lock,flags_s2};
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            request_toggle <= 0; ack_sync <= 0; ack_seen <= 0;
            flags_s1 <= 0; flags_s2 <= 0; bus_s1 <= 0; bus_s2 <= 0;
            word_index <= 0; staging <= 0; transfer_active <= 0;
            frozen <= 0; sequence_count <= 0; loss_in_frame <= 0; pll_d <= 0;
        end else begin
            ack_sync <= {ack_sync[1:0],ack_toggle};
            flags_s1 <= native_flags; flags_s2 <= flags_s1;
            bus_s1 <= word_mailbox; bus_s2 <= bus_s1;
            pll_d <= pll_lock;
            // Correlation only: this is synchronized FRAME_VALID, not a
            // guarantee about the position of a loss within a CSI packet.
            if (pll_d && !pll_lock && flags_s2[0])
                loss_in_frame <= inc(loss_in_frame);
            if (snapshot && !pending && ack_sync[2] == ack_seen) begin
                word_index <= 0;
                transfer_active <= 1;
                request_toggle <= ~request_toggle;
            end
            if (ack_sync[2] != ack_seen) begin
                ack_seen <= ack_sync[2];
                staging[word_index*32 +: 32] <= bus_s2;
                if (word_index == 15) begin
                    frozen <= staging;
                    frozen[15*32 +: 32] <= bus_s2;
                    frozen[0 +: 32] <= sequence_count + 1'b1;
                    // Expose known constants explicitly so synthesis can
                    // remove unused upper bits from the staging/publish banks.
                    frozen[1*32 +: 32] <= 32'h43534933;
                    frozen[2*32 +: 32] <= {22'd0,staging[2*32 +: 10]};
                    frozen[3*32 +: 32] <= {16'd0,staging[3*32 +: 16]};
                    frozen[4*32 +: 32] <= {16'd0,staging[4*32 +: 16]};
                    frozen[5*32 +: 32] <= {16'd0,staging[5*32 +: 16]};
                    frozen[6*32 +: 32] <= {16'd0,staging[6*32 +: 16]};
                    frozen[9*32 +: 32] <= {16'd0,staging[9*32 +: 16]};
                    frozen[10*32 +: 32] <= {16'd0,staging[10*32 +: 16]};
                    frozen[14*32 +: 32] <= {16'd0,staging[14*32 +: 16]};
                    sequence_count <= sequence_count + 1'b1;
                    transfer_active <= 0;
                end else begin
                    word_index <= word_index + 1'b1;
                    request_toggle <= ~request_toggle;
                end
            end
        end
    end
endmodule
