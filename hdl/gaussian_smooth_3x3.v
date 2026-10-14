`timescale 1ns / 1ps

// gaussian_smooth_3x3.v
// 3x3 Gaussian smoothing filter for 8-bit grayscale video stream.
// Kernel:
//   1 2 1
//   2 4 2   / 16
//   1 2 1
// This block is placed BEFORE Sobel to reduce small texture/noise edges.

module gaussian_smooth_3x3 #(
    parameter integer PIXEL_WIDTH = 8,
    parameter integer X_WIDTH     = 12,
    parameter integer Y_WIDTH     = 12
) (
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     window_valid,

    input  wire [PIXEL_WIDTH-1:0]   w00, w01, w02,
    input  wire [PIXEL_WIDTH-1:0]   w10, w11, w12,
    input  wire [PIXEL_WIDTH-1:0]   w20, w21, w22,

    input  wire [X_WIDTH-1:0]       x_in,
    input  wire [Y_WIDTH-1:0]       y_in,

    output reg                      smooth_valid,
    output reg  [PIXEL_WIDTH-1:0]   smooth_pixel,
    output reg  [X_WIDTH-1:0]       x_out,
    output reg  [Y_WIDTH-1:0]       y_out
);

    // Max sum = 255 * 16 = 4080, so 12 bits are enough.
    wire [11:0] sum_gauss;

    assign sum_gauss =
        {4'd0, w00} +
        ({4'd0, w01} << 1) +
        {4'd0, w02} +

        ({4'd0, w10} << 1) +
        ({4'd0, w11} << 2) +
        ({4'd0, w12} << 1) +

        {4'd0, w20} +
        ({4'd0, w21} << 1) +
        {4'd0, w22};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            smooth_valid <= 1'b0;
            smooth_pixel <= {PIXEL_WIDTH{1'b0}};
            x_out        <= {X_WIDTH{1'b0}};
            y_out        <= {Y_WIDTH{1'b0}};
        end else begin
            smooth_valid <= window_valid;

            if (window_valid) begin
                // +8 for rounding before divide by 16.
                smooth_pixel <= (sum_gauss + 12'd8) >> 4;
                x_out        <= x_in;
                y_out        <= y_in;
            end
        end
    end

endmodule
