`timescale 1ns/1ps
// CSI_FRAME_EVENTS_V3: consume explicit CSI-2 frame packets. Legacy mode
// remains available for integrations that only have FRAME_VALID.
module mipi_raw10_raster #(
    parameter integer ACTIVE_W = 640,
    parameter integer ACTIVE_H = 480,
    parameter integer USE_PACKET_EVENTS = 0
) (
    input wire pixel_clk, rst_n,
    input wire [9:0] raw10_in,
    input wire line_valid_in, line_start_in, line_end_in, frame_valid_in,
    input wire frame_start_in, frame_end_in,
    input wire ecc_error_in, crc_error_in,
    output reg [7:0] gray8_out,
    output reg pixel_valid_out, frame_start_out, line_start_out,
    output reg [11:0] x_out, y_out,
    output reg [31:0] frame_count, pixel_count,
    output reg size_error,
    output reg frame_end_out, last_frame_good,
    output reg [31:0] last_frame_pixels,
    output reg [11:0] last_frame_lines, last_line_pixels
);
    localparam [31:0] EXPECTED_PIXELS = ACTIVE_W * ACTIVE_H;
    reg [11:0] x_count, y_count, pixels_this_line, lines_this_frame;
    reg frame_valid_d, fs_d, fe_d, frame_active;
    reg bad_geometry, bad_packet;
    wire frame_rise = (USE_PACKET_EVENTS != 0) ?
        (frame_start_in && !fs_d) : (frame_valid_in && !frame_valid_d);
    wire frame_fall = (USE_PACKET_EVENTS != 0) ?
        (frame_end_in && !fe_d && frame_active && !frame_rise) :
        (!frame_valid_in && frame_valid_d);
    wire in_frame = (USE_PACKET_EVENTS != 0) ?
        (frame_active || frame_rise) : frame_valid_in;
    wire pixel_here = line_valid_in && in_frame;
    wire line_end_here = line_end_in &&
        ((USE_PACKET_EVENTS != 0) ? in_frame :
         (frame_valid_in || frame_valid_d));
    wire [12:0] line_pixels_next =
        ((line_start_in || frame_rise) ? 13'd0 : {1'b0,pixels_this_line}) +
        (pixel_here ? 13'd1 : 13'd0);
    wire [12:0] frame_lines_next =
        (frame_rise ? 13'd0 : {1'b0,lines_this_frame}) +
        (line_end_here ? 13'd1 : 13'd0);
    wire [31:0] frame_pixels_next = (frame_rise ? 32'd0 : pixel_count) +
        (pixel_here ? 32'd1 : 32'd0);
    wire line_bad = line_end_here && (line_pixels_next != ACTIVE_W);
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            x_count <= 0; y_count <= 0; pixels_this_line <= 0;
            lines_this_frame <= 0; frame_valid_d <= 0;
            fs_d <= 0; fe_d <= 0; frame_active <= 0;
            gray8_out <= 0; pixel_valid_out <= 0;
            frame_start_out <= 0; line_start_out <= 0;
            x_out <= 0; y_out <= 0; frame_count <= 0; pixel_count <= 0;
            size_error <= 0; bad_geometry <= 0; bad_packet <= 0;
            frame_end_out <= 0; last_frame_good <= 0;
            last_frame_pixels <= 0; last_frame_lines <= 0; last_line_pixels <= 0;
        end else begin
            frame_valid_d <= frame_valid_in;
            fs_d <= frame_start_in; fe_d <= frame_end_in;
            pixel_valid_out <= 0; frame_start_out <= 0;
            line_start_out <= 0; frame_end_out <= 0;
            if (frame_rise) begin
                // A new FS also resynchronizes after a missing FE. Never
                // publish that abandoned frame as good.
                frame_active <= 1;
                if ((USE_PACKET_EVENTS != 0) && frame_active)
                    size_error <= 1;
                x_count <= 0; y_count <= 0; pixels_this_line <= 0;
                lines_this_frame <= 0; pixel_count <= 0;
                bad_geometry <= 0; bad_packet <= ecc_error_in | crc_error_in;
                frame_start_out <= 1;
                // The downstream buffer samples the previous good flag at
                // this registered frame_start, before the next frame ends.
                if ((USE_PACKET_EVENTS != 0) && frame_active)
                    last_frame_good <= 0;
            end else if ((in_frame || frame_fall) &&
                         (ecc_error_in || crc_error_in)) bad_packet <= 1;
            if (line_start_in && in_frame) begin
                x_count <= 0; pixels_this_line <= 0; line_start_out <= 1;
            end
            if (pixel_here) begin
                gray8_out <= raw10_in[9:2]; pixel_valid_out <= 1;
                x_out <= (line_start_in || frame_rise) ? 12'd0 : x_count;
                y_out <= frame_rise ? 12'd0 : y_count;
                x_count <= (line_start_in || frame_rise) ? 12'd1 : x_count + 1'b1;
                pixels_this_line <= line_pixels_next[11:0];
                pixel_count <= frame_pixels_next;
            end
            if (line_end_here) begin
                if (line_bad) begin size_error <= 1; bad_geometry <= 1; end
                y_count <= (frame_rise ? 12'd0 : y_count) + 1'b1;
                lines_this_frame <= frame_lines_next[11:0];
                x_count <= 0; last_line_pixels <= line_pixels_next[11:0];
            end
            if (frame_fall) begin
                frame_active <= 0;
                frame_count <= frame_count + 1'b1;
                frame_end_out <= 1;
                last_frame_pixels <= frame_pixels_next;
                last_frame_lines <= frame_lines_next[11:0];
                last_frame_good <= !bad_geometry && !line_bad && !bad_packet &&
                    !ecc_error_in && !crc_error_in &&
                    (frame_lines_next == ACTIVE_H) &&
                    (frame_pixels_next == EXPECTED_PIXELS);
                if ((frame_lines_next != ACTIVE_H) ||
                    (frame_pixels_next != EXPECTED_PIXELS)) size_error <= 1;
            end
        end
    end
endmodule
