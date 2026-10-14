`timescale 1ns/1ps

// ============================================================================
// OV7670 binary-edge frontend for the PolarFire SoC YOLO design.
//
// Camera path (cam_pclk domain):
//   OV7670 Y channel -> Gaussian 3x3 -> Sobel -> threshold -> edge refine
//   -> sample 640x480 to 160x120 -> one-bit dual-clock frame buffer.
//
// DDR path (clk domain):
//   160x120 one-bit edge frame -> nearest-neighbour 256x192
//   -> black letterbox to 256x256 -> write signed-int8 YOLO input.
//
// YOLO_INPUT_CHANNELS = 3 (default):
//   The same binary edge plane is written to R, G and B.  This is compatible
//   with the current 3-channel accelerator, but the input is still an edge
//   image, not a colour image.
//
// YOLO_INPUT_CHANNELS = 1:
//   Only one 256x256 plane is written.  Use this mode only after retraining the
//   model for one input channel and changing the first convolution/scheduler
//   and weight layout in the accelerator.
//
// The file contains three synthesizable modules:
//   1) ov7670_edge_yolo_frontend
//   2) edge_frame_buffer_160x120
//   3) edge_160x120_to_chw_ddr
//
// Existing project dependencies:
//   cam_xclk_gen_ov7670.v
//   ov7670_start_pulse.v
//   cam_sccb_init_ov7670.v
//   cam_capture_ov7670.v
//   gray_conv_ov7670.v
//   line_buffer_3x3.v
//   gaussian_smooth_3x3.v
//   sobel_core_fix.v
//   edge_refine_3x3.v
//   cam_xclk_24m_ccc / PF_CCC_C0 generated Libero components
// ============================================================================

module ov7670_edge_yolo_frontend #(
    parameter integer ADDR_W                  = 32,
    parameter [ADDR_W-1:0] INPUT_BASE         = {ADDR_W{1'b0}},

    // clk is the YOLO/APB clock.  The supplied YOLO project targets 75 MHz.
    parameter integer SYS_CLK_HZ              = 54_347_826,

    parameter integer SRC_W                   = 640,
    parameter integer SRC_H                   = 480,
    parameter integer EDGE_W                  = 160,
    parameter integer EDGE_H                  = 120,

    parameter [10:0]  SOBEL_THRESHOLD         = 11'd250,
    parameter integer ENABLE_COLOR_BAR        = 0,
    parameter integer PRE_GAUSSIAN_ENABLE     = 1,
    parameter integer EDGE_REFINE_ENABLE      = 1,
    parameter integer EDGE_KEEP_MIN           = 2,
    parameter integer EDGE_FILL_MIN           = 4,

    parameter integer CAP_Y_ON_ODD_BYTE       = 1,
    parameter integer CAP_VSYNC_ACTIVE_HIGH   = 0,
    parameter integer CAP_HREF_ACTIVE_HIGH    = 1,

    // Keep 3 for the unmodified YOLO accelerator.  Set to 1 only after the
    // accelerator and weights have been converted to a one-channel model.
    parameter integer YOLO_INPUT_CHANNELS     = 3,
    parameter [7:0]   EDGE_ON_VALUE           = 8'd127
) (
    // System/DDR clock domain.
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 clear,
    input  wire                 accept_enable,

    // Dedicated 50 MHz board reference used by the camera CCC.  This is kept
    // separate from clk because the YOLO design may run clk at 75 MHz.
    input  wire                 ref_clk_50m,

    // OV7670 interface.
    input  wire                 cam_pclk,
    input  wire                 cam_vsync,
    input  wire                 cam_href,
    input  wire [7:0]           cam_d,
    inout  wire                 cam_siod,
    output wire                 cam_sioc,
    output wire                 cam_xclk,
    output wire                 cam_reset_n,
    output wire                 cam_pwdn,

    // Byte-write interface.  Addresses are local offsets from INPUT_BASE;
    // yolo_mem_axi_bridge adds the physical DDR base.
    output wire                 mem_wr_req,
    output wire [ADDR_W-1:0]    mem_wr_addr,
    output wire [7:0]           mem_wr_data,
    input  wire                 mem_wr_ready,

    // Status interface compatible with the signals used by
    // yolo_kit_ddr_axi_apb.v for spi_yolo_frontend.
    output wire                 frame_done_pulse,
    output wire                 frame_loaded,
    output wire                 busy,
    output wire                 error,
    output wire                 overrun_error,
    output wire [31:0]          last_frame_id,
    output wire [31:0]          payload_count,
    output wire [31:0]          rx_byte_count,
    output wire [31:0]          frames_loaded_count,
    output wire [7:0]           debug_state,
    output wire [31:0]          debug_first4,
    output wire [31:0]          debug_next4,

    // Camera debug/status.
    output wire                 cam_init_done,
    output wire                 cam_init_busy,
    output wire                 cam_init_error,
    output wire [31:0]          cam_frame_count,
    output wire [31:0]          cam_pixel_count
);

    localparam integer X_STEP = SRC_W / EDGE_W;
    localparam integer Y_STEP = SRC_H / EDGE_H;
    localparam integer X_OFFSET = X_STEP / 2;
    localparam integer Y_OFFSET = Y_STEP / 2;

    // ------------------------------------------------------------------------
    // Camera clocks and reset release
    // ------------------------------------------------------------------------
    wire xclk_internal;
    wire xclk_locked;
    wire ccc_clk_50m_unused;

    cam_xclk_gen_ov7670 u_xclk (
        .clk_in  (ref_clk_50m),
        .rst_n   (rst_n),
        .xclk    (xclk_internal),
        .clk_50m (ccc_clk_50m_unused),
        .locked  (xclk_locked)
    );

    assign cam_xclk = xclk_internal;

    wire cam_pclk_global;

    CLKINT u_cam_pclk_buffer (
        .A (cam_pclk),
        .Y (cam_pclk_global)
    );

    // Synchronize the PLL lock indication before releasing logic in each
    // clock domain.
    reg lock_sys_s1;
    reg lock_sys_s2;
    (* syn_preserve = 1, syn_srlstyle = "registers" *) reg lock_cam_s1;
    (* syn_preserve = 1, syn_srlstyle = "registers" *) reg lock_cam_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lock_sys_s1 <= 1'b0;
            lock_sys_s2 <= 1'b0;
        end else begin
            lock_sys_s1 <= xclk_locked;
            lock_sys_s2 <= lock_sys_s1;
        end
    end

    // Use a synchronous reset in the CAM_PCLK domain.  This keeps the
    // asynchronous reset pin out of the clock-buffer timing path while the
    // two registers still hold the camera pipeline in reset for two PCLKs.
    always @(posedge cam_pclk_global) begin
        if (!rst_n) begin
            lock_cam_s1 <= 1'b0;
            lock_cam_s2 <= 1'b0;
        end else begin
            lock_cam_s1 <= xclk_locked;
            lock_cam_s2 <= lock_cam_s1;
        end
    end

    wire sys_rst_n = rst_n & lock_sys_s2;
    wire cam_rst_n = rst_n & lock_cam_s2;

    // ------------------------------------------------------------------------
    // SCCB initialization
    // ------------------------------------------------------------------------
    wire init_start;
    wire init_done_pulse;
    wire init_error_level;

    reg init_done_latched;
    reg init_error_latched;

    ov7670_start_pulse #(
        .DELAY_CYCLES(SYS_CLK_HZ / 4),
        .PULSE_CYCLES(1)
    ) u_init_start (
        .clk         (clk),
        .rst_n       (sys_rst_n),
        .start_pulse (init_start)
    );

    cam_sccb_init_ov7670 #(
        .CLK_HZ           (SYS_CLK_HZ),
        .SCCB_HZ          (100_000),
        .ENABLE_COLOR_BAR (ENABLE_COLOR_BAR),
        // Check the OV7670 ACK instead of optimizing the error path to zero.
        .IGNORE_ACK_ERROR (0)
    ) u_sccb (
        .clk         (clk),
        .rst_n       (sys_rst_n),
        .start       (init_start),
        .busy        (cam_init_busy),
        .done        (init_done_pulse),
        .error       (init_error_level),
        .sioc        (cam_sioc),
        .cam_reset_n (cam_reset_n),
        .cam_pwdn    (cam_pwdn),
        .siod        (cam_siod)
    );

    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            init_done_latched  <= 1'b0;
            init_error_latched <= 1'b0;
        end else begin
            if (init_done_pulse)
                init_done_latched <= 1'b1;
            if (init_error_level)
                init_error_latched <= 1'b1;
        end
    end

    assign cam_init_done  = init_done_latched;
    assign cam_init_error = init_error_latched & ~init_done_latched;

    // ------------------------------------------------------------------------
    // Capture Y bytes from OV7670
    // ------------------------------------------------------------------------
    wire [7:0]  cap_pixel;
    wire        cap_valid;
    wire        cap_frame_start;
    wire        cap_line_start;
    wire [11:0] cap_x;
    wire [11:0] cap_y;

    cam_capture_ov7670 #(
        .DATA_WIDTH        (8),
        .X_WIDTH           (12),
        .Y_WIDTH           (12),
        .YUV422_Y_ONLY     (1),
        .Y_ON_ODD_BYTE     (CAP_Y_ON_ODD_BYTE),
        .VSYNC_ACTIVE_HIGH (CAP_VSYNC_ACTIVE_HIGH),
        .HREF_ACTIVE_HIGH  (CAP_HREF_ACTIVE_HIGH),
        .ACTIVE_PIXELS     (SRC_W),
        .ACTIVE_LINES      (SRC_H)
    ) u_capture (
        .rst_n       (cam_rst_n),
        .cam_pclk    (cam_pclk_global),
        .cam_vsync   (cam_vsync),
        .cam_href    (cam_href),
        .cam_d       (cam_d),
        .pixel_data  (cap_pixel),
        .pixel_valid (cap_valid),
        .frame_start (cap_frame_start),
        .line_start  (cap_line_start),
        .x           (cap_x),
        .y           (cap_y),
        .frame_count (cam_frame_count),
        .pixel_count (cam_pixel_count)
    );

    // Only toggle the image-processing pipeline while a new frame is needed.
    // The capture counters remain alive, but the line buffers/Sobel stop
    // switching while YOLO owns the DDR input, which reduces dynamic power.
    wire writer_busy;
    wire writer_frame_loaded;
    wire capture_enable_sys;

    assign capture_enable_sys = accept_enable &&
                                init_done_latched &&
                                !writer_busy &&
                                !writer_frame_loaded &&
                                !clear;

    reg process_en_cam_s1;
    reg process_en_cam_s2;

    always @(posedge cam_pclk_global or negedge cam_rst_n) begin
        if (!cam_rst_n) begin
            process_en_cam_s1 <= 1'b0;
            process_en_cam_s2 <= 1'b0;
        end else begin
            process_en_cam_s1 <= capture_enable_sys;
            process_en_cam_s2 <= process_en_cam_s1;
        end
    end

    wire cap_valid_enabled = cap_valid & process_en_cam_s2;

    // ------------------------------------------------------------------------
    // Y/grayscale -> optional Gaussian
    // ------------------------------------------------------------------------
    wire       gray_valid;
    wire [7:0] gray_pixel;

    gray_conv_ov7670 #(
        .BYPASS_Y(1)
    ) u_gray (
        .clk             (cam_pclk_global),
        .rst_n           (cam_rst_n),
        .pixel_valid_in  (cap_valid_enabled),
        .pixel_in        (cap_pixel),
        .pixel_valid_out (gray_valid),
        .gray_out        (gray_pixel)
    );

    wire [7:0]  g_p00, g_p01, g_p02;
    wire [7:0]  g_p10, g_p11, g_p12;
    wire [7:0]  g_p20, g_p21, g_p22;
    wire        g_window_valid;
    wire [11:0] g_x_out;
    wire [11:0] g_y_out;

    line_buffer_3x3 #(
        .PIXEL_WIDTH (8),
        .IMG_WIDTH   (SRC_W),
        .X_WIDTH     (12),
        .Y_WIDTH     (12)
    ) u_lb_gaussian (
        .clk          (cam_pclk_global),
        .rst_n        (cam_rst_n),
        .pixel_in     (gray_pixel),
        .pixel_valid  (gray_valid),
        .x_in         (cap_x),
        .y_in         (cap_y),
        .p00          (g_p00), .p01(g_p01), .p02(g_p02),
        .p10          (g_p10), .p11(g_p11), .p12(g_p12),
        .p20          (g_p20), .p21(g_p21), .p22(g_p22),
        .window_valid (g_window_valid),
        .x_out        (g_x_out),
        .y_out        (g_y_out)
    );

    wire        smooth_valid;
    wire [7:0]  smooth_pixel;
    wire [11:0] smooth_x;
    wire [11:0] smooth_y;

    gaussian_smooth_3x3 #(
        .PIXEL_WIDTH (8),
        .X_WIDTH     (12),
        .Y_WIDTH     (12)
    ) u_gaussian (
        .clk          (cam_pclk_global),
        .rst_n        (cam_rst_n),
        .window_valid (g_window_valid),
        .w00(g_p00), .w01(g_p01), .w02(g_p02),
        .w10(g_p10), .w11(g_p11), .w12(g_p12),
        .w20(g_p20), .w21(g_p21), .w22(g_p22),
        .x_in         (g_x_out),
        .y_in         (g_y_out),
        .smooth_valid (smooth_valid),
        .smooth_pixel (smooth_pixel),
        .x_out        (smooth_x),
        .y_out        (smooth_y)
    );

    wire        pre_sobel_valid;
    wire [7:0]  pre_sobel_pixel;
    wire [11:0] pre_sobel_x;
    wire [11:0] pre_sobel_y;

    assign pre_sobel_valid = (PRE_GAUSSIAN_ENABLE != 0) ? smooth_valid : gray_valid;
    assign pre_sobel_pixel = (PRE_GAUSSIAN_ENABLE != 0) ? smooth_pixel : gray_pixel;
    assign pre_sobel_x     = (PRE_GAUSSIAN_ENABLE != 0) ? smooth_x     : cap_x;
    assign pre_sobel_y     = (PRE_GAUSSIAN_ENABLE != 0) ? smooth_y     : cap_y;

    // ------------------------------------------------------------------------
    // Sobel edge magnitude
    // ------------------------------------------------------------------------
    wire [7:0]  s_p00, s_p01, s_p02;
    wire [7:0]  s_p10, s_p11, s_p12;
    wire [7:0]  s_p20, s_p21, s_p22;
    wire        s_window_valid;
    wire [11:0] s_x_out;
    wire [11:0] s_y_out;

    line_buffer_3x3 #(
        .PIXEL_WIDTH (8),
        .IMG_WIDTH   (SRC_W),
        .X_WIDTH     (12),
        .Y_WIDTH     (12)
    ) u_lb_sobel (
        .clk          (cam_pclk_global),
        .rst_n        (cam_rst_n),
        .pixel_in     (pre_sobel_pixel),
        .pixel_valid  (pre_sobel_valid),
        .x_in         (pre_sobel_x),
        .y_in         (pre_sobel_y),
        .p00          (s_p00), .p01(s_p01), .p02(s_p02),
        .p10          (s_p10), .p11(s_p11), .p12(s_p12),
        .p20          (s_p20), .p21(s_p21), .p22(s_p22),
        .window_valid (s_window_valid),
        .x_out        (s_x_out),
        .y_out        (s_y_out)
    );

    wire        sobel_valid;
    wire [10:0] sobel_mag;

    sobel_core_fix u_sobel (
        .clk          (cam_pclk_global),
        .rst_n        (cam_rst_n),
        .window_valid (s_window_valid),
        .w00(s_p00), .w01(s_p01), .w02(s_p02),
        .w10(s_p10), .w11(s_p11), .w12(s_p12),
        .w20(s_p20), .w21(s_p21), .w22(s_p22),
        .grad_valid   (sobel_valid),
        .grad_mag     (sobel_mag)
    );

    // sobel_core_fix delays valid/magnitude by three clocks.
    reg [11:0] sobel_x_d1, sobel_x_d2, sobel_x_d3;
    reg [11:0] sobel_y_d1, sobel_y_d2, sobel_y_d3;

    always @(posedge cam_pclk_global or negedge cam_rst_n) begin
        if (!cam_rst_n) begin
            sobel_x_d1 <= 12'd0;
            sobel_x_d2 <= 12'd0;
            sobel_x_d3 <= 12'd0;
            sobel_y_d1 <= 12'd0;
            sobel_y_d2 <= 12'd0;
            sobel_y_d3 <= 12'd0;
        end else begin
            sobel_x_d1 <= s_x_out;
            sobel_x_d2 <= sobel_x_d1;
            sobel_x_d3 <= sobel_x_d2;
            sobel_y_d1 <= s_y_out;
            sobel_y_d2 <= sobel_y_d1;
            sobel_y_d3 <= sobel_y_d2;
        end
    end

    wire [11:0] edge_x_aligned = sobel_x_d3;
    wire [11:0] edge_y_aligned = sobel_y_d3;
    wire        edge_bit_raw   = (sobel_mag >= SOBEL_THRESHOLD);

    // ------------------------------------------------------------------------
    // Binary edge refinement
    // ------------------------------------------------------------------------
    wire [0:0] e_p00, e_p01, e_p02;
    wire [0:0] e_p10, e_p11, e_p12;
    wire [0:0] e_p20, e_p21, e_p22;
    wire       e_window_valid;
    wire [11:0] e_x_out;
    wire [11:0] e_y_out;

    line_buffer_3x3 #(
        .PIXEL_WIDTH (1),
        .IMG_WIDTH   (SRC_W),
        .X_WIDTH     (12),
        .Y_WIDTH     (12)
    ) u_lb_edge_refine (
        .clk          (cam_pclk_global),
        .rst_n        (cam_rst_n),
        .pixel_in     (edge_bit_raw),
        .pixel_valid  (sobel_valid),
        .x_in         (edge_x_aligned),
        .y_in         (edge_y_aligned),
        .p00          (e_p00), .p01(e_p01), .p02(e_p02),
        .p10          (e_p10), .p11(e_p11), .p12(e_p12),
        .p20          (e_p20), .p21(e_p21), .p22(e_p22),
        .window_valid (e_window_valid),
        .x_out        (e_x_out),
        .y_out        (e_y_out)
    );

    wire        edge_refined_valid;
    wire        edge_refined_bit;
    wire [11:0] edge_refined_x;
    wire [11:0] edge_refined_y;

    edge_refine_3x3 #(
        .X_WIDTH  (12),
        .Y_WIDTH  (12),
        .KEEP_MIN (EDGE_KEEP_MIN),
        .FILL_MIN (EDGE_FILL_MIN)
    ) u_edge_refine (
        .clk          (cam_pclk_global),
        .rst_n        (cam_rst_n),
        .window_valid (e_window_valid),
        .p00(e_p00[0]), .p01(e_p01[0]), .p02(e_p02[0]),
        .p10(e_p10[0]), .p11(e_p11[0]), .p12(e_p12[0]),
        .p20(e_p20[0]), .p21(e_p21[0]), .p22(e_p22[0]),
        .x_in         (e_x_out),
        .y_in         (e_y_out),
        .edge_valid   (edge_refined_valid),
        .edge_out     (edge_refined_bit),
        .x_out        (edge_refined_x),
        .y_out        (edge_refined_y)
    );

    wire edge_final_valid = (EDGE_REFINE_ENABLE != 0) ? edge_refined_valid : sobel_valid;
    wire edge_final_bit   = (EDGE_REFINE_ENABLE != 0) ? edge_refined_bit   : edge_bit_raw;
    wire [11:0] edge_final_x = (EDGE_REFINE_ENABLE != 0) ? edge_refined_x : edge_x_aligned;
    wire [11:0] edge_final_y = (EDGE_REFINE_ENABLE != 0) ? edge_refined_y : edge_y_aligned;

    // ------------------------------------------------------------------------
    // One-bit dual-clock frame buffer
    // ------------------------------------------------------------------------
    wire [14:0] edge_rd_addr;
    wire        edge_rd_data;
    wire        frame_ready_pulse;
    wire        capture_busy_cam;
    wire        consume_toggle;

    edge_frame_buffer_160x120 #(
        .SRC_W    (SRC_W),
        .SRC_H    (SRC_H),
        .IMG_W    (EDGE_W),
        .IMG_H    (EDGE_H),
        .X_STEP   (X_STEP),
        .Y_STEP   (Y_STEP),
        .X_OFFSET (X_OFFSET),
        .Y_OFFSET (Y_OFFSET)
    ) u_edge_frame_buffer (
        .wr_clk               (cam_pclk_global),
        .wr_rst_n             (cam_rst_n),
        .capture_enable_async (capture_enable_sys),
        .frame_start          (cap_frame_start),
        .x                    (edge_final_x),
        .y                    (edge_final_y),
        .pixel_in             (edge_final_bit),
        .pixel_valid          (edge_final_valid),
        .capture_busy         (capture_busy_cam),
        .consume_toggle_async (consume_toggle),
        .rd_clk               (clk),
        .rd_rst_n             (sys_rst_n),
        .rd_addr              (edge_rd_addr),
        .rd_data              (edge_rd_data),
        .frame_ready_pulse    (frame_ready_pulse)
    );

    reg capture_busy_s1;
    reg capture_busy_s2;

    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            capture_busy_s1 <= 1'b0;
            capture_busy_s2 <= 1'b0;
        end else begin
            capture_busy_s1 <= capture_busy_cam;
            capture_busy_s2 <= capture_busy_s1;
        end
    end

    // ------------------------------------------------------------------------
    // Resize/letterbox and write edge tensor to DDR
    // ------------------------------------------------------------------------
    wire writer_error;

    edge_160x120_to_chw_ddr #(
        .ADDR_W              (ADDR_W),
        .INPUT_BASE          (INPUT_BASE),
        .YOLO_INPUT_CHANNELS (YOLO_INPUT_CHANNELS),
        .EDGE_ON_VALUE       (EDGE_ON_VALUE)
    ) u_edge_ddr_writer (
        .clk                 (clk),
        .rst_n               (sys_rst_n),
        .clear               (clear),
        .frame_ready_pulse   (frame_ready_pulse),
        .edge_rd_addr        (edge_rd_addr),
        .edge_rd_data        (edge_rd_data),
        .consume_toggle      (consume_toggle),
        .mem_wr_req          (mem_wr_req),
        .mem_wr_addr         (mem_wr_addr),
        .mem_wr_data         (mem_wr_data),
        .mem_wr_ready        (mem_wr_ready),
        .frame_done_pulse    (frame_done_pulse),
        .frame_loaded        (writer_frame_loaded),
        .busy                (writer_busy),
        .error               (writer_error),
        .overrun_error       (overrun_error),
        .last_frame_id       (last_frame_id),
        .payload_count       (payload_count),
        .frames_loaded_count (frames_loaded_count),
        .debug_state         (debug_state)
    );

    assign frame_loaded = writer_frame_loaded;
    assign busy         = writer_busy | capture_busy_s2;
    assign error        = writer_error | cam_init_error;
    assign rx_byte_count = payload_count;
    assign debug_first4 = 32'd0;
    assign debug_next4  = 32'd0;

endmodule


// ============================================================================
// One-bank 160x120x1-bit dual-clock edge frame buffer.
//
// The RAM inference attribute requests PolarFire LSRAM.  After synthesis,
// verify the RAM report.  If the tool does not infer LSRAM in the installed
// Libero/Synplify version, replace only frame_mem with a PF_TPSRAM component;
// the surrounding module interface does not need to change.
// ============================================================================
module edge_frame_buffer_160x120 #(
    parameter integer SRC_W    = 640,
    parameter integer SRC_H    = 480,
    parameter integer IMG_W    = 160,
    parameter integer IMG_H    = 120,
    parameter integer X_STEP   = 4,
    parameter integer Y_STEP   = 4,
    parameter integer X_OFFSET = 2,
    parameter integer Y_OFFSET = 2
) (
    input  wire        wr_clk,
    input  wire        wr_rst_n,
    input  wire        capture_enable_async,
    input  wire        frame_start,
    input  wire [11:0] x,
    input  wire [11:0] y,
    input  wire        pixel_in,
    input  wire        pixel_valid,
    output wire        capture_busy,
    input  wire        consume_toggle_async,

    input  wire        rd_clk,
    input  wire        rd_rst_n,
    input  wire [14:0] rd_addr,
    output reg         rd_data,
    output reg         frame_ready_pulse
);

    localparam integer TOTAL_PIXELS = IMG_W * IMG_H;
    localparam integer ADDR_WIDTH = (TOTAL_PIXELS <= 2) ? 1 : $clog2(TOTAL_PIXELS);
    localparam integer SX_WIDTH = (IMG_W <= 2) ? 1 : $clog2(IMG_W);
    localparam [ADDR_WIDTH-1:0] LAST_ADDR = TOTAL_PIXELS - 1;
    localparam [11:0] X_OFF_12 = X_OFFSET;
    localparam [11:0] Y_OFF_12 = Y_OFFSET;

    // One bit per pixel: 19,200 bits for 160x120.
    (* syn_ramstyle = "lsram, no_rw_check" *) reg frame_mem [0:TOTAL_PIXELS-1];

    reg [ADDR_WIDTH-1:0] wr_addr;
    reg [ADDR_WIDTH-1:0] sample_count;
    reg [SX_WIDTH-1:0]   sample_x_index;
    reg [11:0]           next_x;
    reg [11:0]           next_y;

    reg capture_active;
    reg frame_locked;
    reg ready_toggle;

    reg capture_en_s1;
    reg capture_en_s2;
    reg consume_s1;
    reg consume_s2;
    reg consume_seen;

    reg [11:0] prev_y;
    reg        prev_pixel_valid;
    reg        near_origin_d;

    wire consume_event = consume_s2 ^ consume_seen;
    wire y_wrap = pixel_valid && prev_pixel_valid && (y < prev_y);
    wire near_origin = pixel_valid &&
                       (y <= (Y_OFF_12 + 12'd4)) &&
                       (x <= (X_OFF_12 + 12'd8));
    wire near_origin_rise = near_origin && !near_origin_d;

    wire start_capture = capture_en_s2 &&
                         !capture_active &&
                         !frame_locked &&
                         (frame_start || y_wrap || near_origin_rise);

    wire hit_target = pixel_valid &&
                      capture_active &&
                      !frame_locked &&
                      (y >= next_y) &&
                      (x >= next_x);

    assign capture_busy = capture_active | frame_locked;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_addr          <= {ADDR_WIDTH{1'b0}};
            sample_count     <= {ADDR_WIDTH{1'b0}};
            sample_x_index   <= {SX_WIDTH{1'b0}};
            next_x           <= X_OFF_12;
            next_y           <= Y_OFF_12;
            capture_active   <= 1'b0;
            frame_locked     <= 1'b0;
            ready_toggle     <= 1'b0;
            capture_en_s1    <= 1'b0;
            capture_en_s2    <= 1'b0;
            consume_s1       <= 1'b0;
            consume_s2       <= 1'b0;
            consume_seen     <= 1'b0;
            prev_y           <= 12'd0;
            prev_pixel_valid <= 1'b0;
            near_origin_d    <= 1'b0;
        end else begin
            capture_en_s1 <= capture_enable_async;
            capture_en_s2 <= capture_en_s1;
            consume_s1    <= consume_toggle_async;
            consume_s2    <= consume_s1;
            near_origin_d <= near_origin;

            if (pixel_valid) begin
                prev_y           <= y;
                prev_pixel_valid <= 1'b1;
            end

            if (consume_event) begin
                consume_seen   <= consume_s2;
                frame_locked   <= 1'b0;
                capture_active <= 1'b0;
            end

            if (start_capture) begin
                wr_addr        <= {ADDR_WIDTH{1'b0}};
                sample_count   <= {ADDR_WIDTH{1'b0}};
                sample_x_index <= {SX_WIDTH{1'b0}};
                next_x         <= X_OFF_12;
                next_y         <= Y_OFF_12;
                capture_active <= 1'b1;
            end

            if (hit_target) begin
                frame_mem[wr_addr] <= pixel_in;

                if (sample_count == LAST_ADDR) begin
                    ready_toggle   <= ~ready_toggle;
                    frame_locked   <= 1'b1;
                    capture_active <= 1'b0;
                end else begin
                    wr_addr      <= wr_addr + 1'b1;
                    sample_count <= sample_count + 1'b1;

                    if (sample_x_index == IMG_W - 1) begin
                        sample_x_index <= {SX_WIDTH{1'b0}};
                        next_x         <= X_OFF_12;
                        next_y         <= next_y + Y_STEP;
                    end else begin
                        sample_x_index <= sample_x_index + 1'b1;
                        next_x         <= next_x + X_STEP;
                    end
                end
            end
        end
    end

    reg ready_s1;
    reg ready_s2;
    reg ready_seen;

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            ready_s1          <= 1'b0;
            ready_s2          <= 1'b0;
            ready_seen        <= 1'b0;
            frame_ready_pulse <= 1'b0;
            rd_data           <= 1'b0;
        end else begin
            ready_s1          <= ready_toggle;
            ready_s2          <= ready_s1;
            frame_ready_pulse <= ready_s2 ^ ready_seen;
            ready_seen        <= ready_s2;
            rd_data           <= frame_mem[rd_addr];
        end
    end

endmodule


// ============================================================================
// Read a 160x120 one-bit edge image and write a YOLO input tensor.
//
// Destination mapping:
//   source 160x120 -> nearest-neighbour 256x192 (scale 8/5)
//   destination y=0..31   : black padding
//   destination y=32..223 : resized edge image
//   destination y=224..255: black padding
//
// CHW addresses for three-channel compatibility:
//   plane 0: INPUT_BASE +      0 + pixel_index
//   plane 1: INPUT_BASE +  65536 + pixel_index
//   plane 2: INPUT_BASE + 131072 + pixel_index
// ============================================================================
module edge_160x120_to_chw_ddr #(
    parameter integer ADDR_W = 32,
    parameter [ADDR_W-1:0] INPUT_BASE = {ADDR_W{1'b0}},
    parameter integer YOLO_INPUT_CHANNELS = 3,
    parameter [7:0] EDGE_ON_VALUE = 8'd127
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 clear,
    input  wire                 frame_ready_pulse,

    output reg  [14:0]          edge_rd_addr,
    input  wire                 edge_rd_data,
    output reg                  consume_toggle,

    output wire                 mem_wr_req,
    output wire [ADDR_W-1:0]    mem_wr_addr,
    output wire [7:0]           mem_wr_data,
    input  wire                 mem_wr_ready,

    output reg                  frame_done_pulse,
    output reg                  frame_loaded,
    output reg                  busy,
    output reg                  error,
    output wire                 overrun_error,
    output reg  [31:0]          last_frame_id,
    output reg  [31:0]          payload_count,
    output reg  [31:0]          frames_loaded_count,
    output wire [7:0]           debug_state
);

    localparam [3:0]
        ST_IDLE      = 4'd0,
        ST_PREP      = 4'd1,
        ST_RD_ADDR   = 4'd2,
        ST_RD_WAIT   = 4'd3,
        ST_RD_LATCH  = 4'd4,
        ST_WRITE_0   = 4'd5,
        ST_WRITE_1   = 4'd6,
        ST_WRITE_2   = 4'd7,
        ST_NEXT      = 4'd8,
        ST_HOLD      = 4'd9;

    localparam [ADDR_W-1:0] PLANE_BYTES_ADDR = 32'd65536;
    localparam [31:0] FRAME_BYTES = (YOLO_INPUT_CHANNELS == 1) ?
                                     32'd65536 : 32'd196608;

    reg [3:0]  state;
    reg [15:0] dst_index;
    // Only one bit is required: the source image is binary edge data.
    // Expand it to EDGE_ON_VALUE only at the DDR interface.
    reg        edge_latched;
    reg        frame_pending;
    reg        buffer_owned;

    wire [7:0] dst_x = dst_index[7:0];
    wire [7:0] dst_y = dst_index[15:8];
    wire padding = (dst_y < 8'd32) || (dst_y >= 8'd224);

    wire [7:0] active_y = dst_y - 8'd32;
    wire [10:0] dst_x_ext = {3'd0, dst_x};
    wire [10:0] active_y_ext = {3'd0, active_y};
    wire [10:0] src_x_mul5 = (dst_x_ext << 2) + dst_x_ext;
    wire [10:0] src_y_mul5 = (active_y_ext << 2) + active_y_ext;
    wire [7:0] src_x = src_x_mul5[10:3];
    wire [7:0] src_y = src_y_mul5[10:3];

    wire [14:0] src_y_15 = {7'd0, src_y};
    wire [14:0] src_x_15 = {7'd0, src_x};
    wire [14:0] src_addr_calc = (src_y_15 << 7) +
                                      (src_y_15 << 5) +
                                      src_x_15;

    wire write_state = (state == ST_WRITE_0) ||
                       (state == ST_WRITE_1) ||
                       (state == ST_WRITE_2);

    wire [ADDR_W-1:0] pixel_addr = {{(ADDR_W-16){1'b0}}, dst_index};
    wire [ADDR_W-1:0] plane_offset =
        (state == ST_WRITE_1) ? PLANE_BYTES_ADDR :
        (state == ST_WRITE_2) ? (PLANE_BYTES_ADDR << 1) :
                                {ADDR_W{1'b0}};

    assign mem_wr_req  = write_state;
    assign mem_wr_addr = INPUT_BASE + plane_offset + pixel_addr;
    assign mem_wr_data = edge_latched ? EDGE_ON_VALUE : 8'd0;
    assign debug_state = {4'd0, state};
    // Every writer error is currently an overrun; expose one state bit instead
    // of keeping two equivalent registers.
    assign overrun_error = error;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= ST_IDLE;
            dst_index           <= 16'd0;
            edge_latched        <= 1'b0;
            edge_rd_addr        <= 15'd0;
            consume_toggle      <= 1'b0;
            frame_done_pulse    <= 1'b0;
            frame_loaded        <= 1'b0;
            busy                <= 1'b0;
            error               <= 1'b0;
            last_frame_id       <= 32'd0;
            payload_count       <= 32'd0;
            frames_loaded_count <= 32'd0;
            frame_pending       <= 1'b0;
            buffer_owned        <= 1'b0;
        end else begin
            frame_done_pulse <= 1'b0;

            // Latch the short CDC pulse until the writer can start.
            if (frame_ready_pulse) begin
                if (buffer_owned || frame_pending || busy || frame_loaded) begin
                    error         <= 1'b1;
                end else begin
                    frame_pending <= 1'b1;
                    buffer_owned  <= 1'b1;
                end
            end

            if (clear) begin
                // If software aborts while a captured frame is owned, release
                // the camera buffer so the next request cannot deadlock.
                if (buffer_owned)
                    consume_toggle <= ~consume_toggle;

                state               <= ST_IDLE;
                dst_index           <= 16'd0;
                edge_latched        <= 1'b0;
                edge_rd_addr        <= 15'd0;
                frame_done_pulse    <= 1'b0;
                frame_loaded        <= 1'b0;
                busy                <= 1'b0;
                error               <= 1'b0;
                last_frame_id       <= 32'd0;
                payload_count       <= 32'd0;
                frames_loaded_count <= 32'd0;
                frame_pending       <= 1'b0;
                buffer_owned        <= 1'b0;
            end else begin
                case (state)
                    ST_IDLE: begin
                        busy <= 1'b0;

                        if (frame_pending && !frame_loaded) begin
                            frame_pending <= 1'b0;
                            dst_index     <= 16'd0;
                            payload_count <= 32'd0;
                            busy          <= 1'b1;
                            state         <= ST_PREP;
                        end
                    end

                    ST_PREP: begin
                        busy <= 1'b1;

                        if (padding) begin
                            edge_latched <= 1'b0;
                            state         <= ST_WRITE_0;
                        end else begin
                            state <= ST_RD_ADDR;
                        end
                    end

                    ST_RD_ADDR: begin
                        edge_rd_addr <= src_addr_calc;
                        state        <= ST_RD_WAIT;
                    end

                    // The inferred/PF_TPSRAM read port is synchronous.  One
                    // address cycle and one wait cycle are deliberately kept.
                    ST_RD_WAIT: begin
                        state <= ST_RD_LATCH;
                    end

                    ST_RD_LATCH: begin
                        edge_latched <= edge_rd_data;
                        state         <= ST_WRITE_0;
                    end

                    ST_WRITE_0: begin
                        if (mem_wr_ready) begin
                            payload_count <= payload_count + 32'd1;
                            if (YOLO_INPUT_CHANNELS == 1)
                                state <= ST_NEXT;
                            else
                                state <= ST_WRITE_1;
                        end
                    end

                    ST_WRITE_1: begin
                        if (mem_wr_ready) begin
                            payload_count <= payload_count + 32'd1;
                            state         <= ST_WRITE_2;
                        end
                    end

                    ST_WRITE_2: begin
                        if (mem_wr_ready) begin
                            payload_count <= payload_count + 32'd1;
                            state         <= ST_NEXT;
                        end
                    end

                    ST_NEXT: begin
                        if (dst_index == 16'hffff) begin
                            frame_loaded        <= 1'b1;
                            frame_done_pulse    <= 1'b1;
                            busy                <= 1'b0;
                            last_frame_id       <= last_frame_id + 32'd1;
                            frames_loaded_count <= frames_loaded_count + 32'd1;
                            payload_count       <= FRAME_BYTES;

                            if (buffer_owned) begin
                                consume_toggle <= ~consume_toggle;
                                buffer_owned   <= 1'b0;
                            end

                            state <= ST_HOLD;
                        end else begin
                            dst_index <= dst_index + 1'b1;
                            state     <= ST_PREP;
                        end
                    end

                    ST_HOLD: begin
                        busy <= 1'b0;
                        // frame_loaded remains asserted until clear.
                    end

                    default: begin
                        state <= ST_IDLE;
                        busy  <= 1'b0;
                        error <= 1'b1;
                    end
                endcase
            end
        end
    end

endmodule
