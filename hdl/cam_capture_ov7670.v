`timescale 1ns / 1ps



module cam_capture_ov7670 #(
    parameter integer DATA_WIDTH        = 8,
    parameter integer X_WIDTH           = 12,
    parameter integer Y_WIDTH           = 12,
    parameter integer YUV422_Y_ONLY     = 1,
    parameter integer Y_ON_ODD_BYTE     = 0,
    parameter integer VSYNC_ACTIVE_HIGH = 1,
    parameter integer HREF_ACTIVE_HIGH  = 1,

    // Number of Y pixels expected per active line.
    parameter integer ACTIVE_PIXELS     = 640,

    // Number of active lines expected per frame.
    parameter integer ACTIVE_LINES      = 480
) (
    input  wire                  rst_n,
    input  wire                  cam_pclk,
    input  wire                  cam_vsync,
    input  wire                  cam_href,
    input  wire [DATA_WIDTH-1:0] cam_d,

    output reg  [DATA_WIDTH-1:0] pixel_data,
    output reg                   pixel_valid,
    output reg                   frame_start,
    output reg                   line_start,
    output reg  [X_WIDTH-1:0]    x,
    output reg  [Y_WIDTH-1:0]    y,
    output reg  [31:0]           frame_count,
    output reg  [31:0]           pixel_count
);

    (* syn_preserve = 1 *) reg [X_WIDTH-1:0] x_cnt;
    (* syn_preserve = 1 *) reg [Y_WIDTH-1:0] y_cnt;
    (* syn_preserve = 1 *) reg               byte_phase;

    reg frame_seen_pixel;
    reg line_seen_pixel;

    // Sync VSYNC/HREF into PCLK domain.
    reg vsync_d1, vsync_d2, vsync_d3;
    reg href_d1,  href_d2,  href_d3;

    wire vsync_active;
    wire vsync_active_prev;
    wire vsync_edge_active;

    wire href_active;
    wire href_active_prev;
    wire href_rise;
    wire href_fall;

    wire y_byte_selected;
    wire take_pixel;

    assign vsync_active      = (VSYNC_ACTIVE_HIGH != 0) ? vsync_d2 : ~vsync_d2;
    assign vsync_active_prev = (VSYNC_ACTIVE_HIGH != 0) ? vsync_d3 : ~vsync_d3;
    assign vsync_edge_active = vsync_active & ~vsync_active_prev;

    assign href_active      = (HREF_ACTIVE_HIGH != 0) ? href_d2 : ~href_d2;
    assign href_active_prev = (HREF_ACTIVE_HIGH != 0) ? href_d3 : ~href_d3;
    assign href_rise        = href_active & ~href_active_prev;
    assign href_fall        = ~href_active & href_active_prev;

    assign y_byte_selected = (Y_ON_ODD_BYTE != 0) ? byte_phase : ~byte_phase;

    assign take_pixel = href_active &&
                        ((YUV422_Y_ONLY == 0) || y_byte_selected);

    wire [X_WIDTH-1:0] LAST_X;
    wire [Y_WIDTH-1:0] LAST_Y;

    assign LAST_X = ACTIVE_PIXELS[X_WIDTH-1:0] - 1'b1;
    assign LAST_Y = ACTIVE_LINES [Y_WIDTH-1:0] - 1'b1;

    task reset_frame_counters;
        begin
            x_cnt            <= {X_WIDTH{1'b0}};
            y_cnt            <= {Y_WIDTH{1'b0}};
            byte_phase       <= 1'b0;
            frame_seen_pixel <= 1'b0;
            line_seen_pixel  <= 1'b0;
            pixel_count      <= 32'd0;
        end
    endtask

    task advance_line;
        begin
            x_cnt           <= {X_WIDTH{1'b0}};
            byte_phase      <= 1'b0;
            line_seen_pixel <= 1'b0;

            if (y_cnt >= LAST_Y) begin
                y_cnt            <= {Y_WIDTH{1'b0}};
                frame_seen_pixel <= 1'b0;
                pixel_count      <= 32'd0;
            end else begin
                y_cnt <= y_cnt + 1'b1;
            end
        end
    endtask

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_data       <= {DATA_WIDTH{1'b0}};
            pixel_valid      <= 1'b0;
            frame_start      <= 1'b0;
            line_start       <= 1'b0;

            x                <= {X_WIDTH{1'b0}};
            y                <= {Y_WIDTH{1'b0}};
            x_cnt            <= {X_WIDTH{1'b0}};
            y_cnt            <= {Y_WIDTH{1'b0}};

            byte_phase       <= 1'b0;
            frame_seen_pixel <= 1'b0;
            line_seen_pixel  <= 1'b0;

            frame_count      <= 32'd0;
            pixel_count      <= 32'd0;

            vsync_d1         <= 1'b0;
            vsync_d2         <= 1'b0;
            vsync_d3         <= 1'b0;
            href_d1          <= 1'b0;
            href_d2          <= 1'b0;
            href_d3          <= 1'b0;
        end else begin
            vsync_d1 <= cam_vsync;
            vsync_d2 <= vsync_d1;
            vsync_d3 <= vsync_d2;

            href_d1  <= cam_href;
            href_d2  <= href_d1;
            href_d3  <= href_d2;

            pixel_valid <= 1'b0;
            frame_start <= 1'b0;
            line_start  <= 1'b0;

            // Use VSYNC only as a boundary edge, not as a level gate.
            // This prevents permanent reset if VSYNC polarity is wrong or noisy.
            if (vsync_edge_active) begin
                reset_frame_counters();
            end else begin
                if (href_rise) begin
                    x_cnt           <= {X_WIDTH{1'b0}};
                    byte_phase      <= 1'b0;
                    line_seen_pixel <= 1'b0;
                end

                if (href_fall) begin
                    // Real HREF may end before x_cnt reaches ACTIVE_PIXELS.
                    // Still advance Y so the next HREF line has a new coordinate.
                    if (line_seen_pixel)
                        advance_line();
                    else begin
                        x_cnt      <= {X_WIDTH{1'b0}};
                        byte_phase <= 1'b0;
                    end
                end

                if (take_pixel) begin
                    pixel_data  <= cam_d;
                    pixel_valid <= 1'b1;
                    x           <= x_cnt;
                    y           <= y_cnt;

                    if (!frame_seen_pixel) begin
                        frame_start      <= 1'b1;
                        frame_seen_pixel <= 1'b1;
                        frame_count      <= frame_count + 1'b1;
                        pixel_count      <= 32'd1;
                    end else begin
                        pixel_count <= pixel_count + 1'b1;
                    end

                    if (!line_seen_pixel) begin
                        line_start      <= 1'b1;
                        line_seen_pixel <= 1'b1;
                    end

                    if (x_cnt >= LAST_X) begin
                        advance_line();
                    end else begin
                        x_cnt <= x_cnt + 1'b1;
                    end
                end

                if (href_active && (YUV422_Y_ONLY != 0))
                    byte_phase <= ~byte_phase;
            end
        end
    end

endmodule
