`timescale 1ns / 1ps

// edge_refine_3x3.v
// Binary edge post-filter placed AFTER Sobel threshold and BEFORE frame sampler.
// Purpose:
//   - remove isolated salt-pepper edge dots
//   - connect small gaps in broken edge lines
//   - make the 160x120 UART image look smoother after downsampling
//
// Rule:
//   If center pixel is white: keep it only when the 3x3 white count >= KEEP_MIN.
//   If center pixel is black : turn it white only when the 3x3 white count >= FILL_MIN.
//
// Suggested parameters:
//   KEEP_MIN = 2 : removes isolated single pixels
//   FILL_MIN = 4 : fills small gaps but avoids expanding too much noise

module edge_refine_3x3 #(
    parameter integer X_WIDTH  = 12,
    parameter integer Y_WIDTH  = 12,
    parameter integer KEEP_MIN = 2,
    parameter integer FILL_MIN = 4
) (
    input  wire               clk,
    input  wire               rst_n,

    input  wire               window_valid,

    input  wire               p00, p01, p02,
    input  wire               p10, p11, p12,
    input  wire               p20, p21, p22,

    input  wire [X_WIDTH-1:0] x_in,
    input  wire [Y_WIDTH-1:0] y_in,

    output reg                edge_valid,
    output reg                edge_out,
    output reg  [X_WIDTH-1:0] x_out,
    output reg  [Y_WIDTH-1:0] y_out
);

    localparam [3:0] KEEP_MIN_4 = KEEP_MIN;
    localparam [3:0] FILL_MIN_4 = FILL_MIN;

    wire [3:0] white_count;

    assign white_count =
        p00 + p01 + p02 +
        p10 + p11 + p12 +
        p20 + p21 + p22;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            edge_valid <= 1'b0;
            edge_out   <= 1'b0;
            x_out      <= {X_WIDTH{1'b0}};
            y_out      <= {Y_WIDTH{1'b0}};
        end else begin
            edge_valid <= window_valid;

            if (window_valid) begin
                // Center white: remove isolated noise.
                // Center black : fill small holes/gaps only when enough neighbors exist.
                if (p11)
                    edge_out <= (white_count >= KEEP_MIN_4);
                else
                    edge_out <= (white_count >= FILL_MIN_4);

                x_out <= x_in;
                y_out <= y_in;
            end
        end
    end

endmodule
