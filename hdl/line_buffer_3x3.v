`timescale 1ns / 1ps

module line_buffer_3x3 #(
    parameter integer PIXEL_WIDTH = 8,
    parameter integer IMG_WIDTH   = 640, 
    parameter integer X_WIDTH     = 12,
    parameter integer Y_WIDTH     = 12
) (
    input  wire                      clk,
    input  wire                      rst_n,

    input  wire [PIXEL_WIDTH-1:0]    pixel_in,
    input  wire                      pixel_valid,
    input  wire [X_WIDTH-1:0]        x_in,
    input  wire [Y_WIDTH-1:0]        y_in,

    output wire [PIXEL_WIDTH-1:0]    p00, p01, p02,
    output wire [PIXEL_WIDTH-1:0]    p10, p11, p12,
    output wire [PIXEL_WIDTH-1:0]    p20, p21, p22,

    output reg                       window_valid,
    output reg  [X_WIDTH-1:0]        x_out,
    output reg  [Y_WIDTH-1:0]        y_out
);

    // T? ð?ng tính toán ð? r?ng ð?a ch? ð? bao ph? h?t IMG_WIDTH
    localparam integer ADDR_WIDTH = $clog2(IMG_WIDTH);

    // RAM hàng (S? d?ng LSRAM trên Microchip PolarFire)
    (* syn_ramstyle = "lsram, rw_check" *) reg [PIXEL_WIDTH-1:0] linebuf0 [0:IMG_WIDTH-1];
    (* syn_ramstyle = "lsram, rw_check" *) reg [PIXEL_WIDTH-1:0] linebuf1 [0:IMG_WIDTH-1];

    // C?a s? thanh ghi d?ch 3x3
    (* syn_preserve = 1 *) reg [PIXEL_WIDTH-1:0] r0_c0, r0_c1, r0_c2;
    (* syn_preserve = 1 *) reg [PIXEL_WIDTH-1:0] r1_c0, r1_c1, r1_c2;
    (* syn_preserve = 1 *) reg [PIXEL_WIDTH-1:0] r2_c0, r2_c1, r2_c2;

    // Ð?a ch? an toàn ð? ðý?c s?a l?i ð? r?ng bit
    wire [ADDR_WIDTH-1:0] safe_addr = x_in[ADDR_WIDTH-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {r0_c0, r0_c1, r0_c2, r1_c0, r1_c1, r1_c2, r2_c0, r2_c1, r2_c2} <= 0;
            window_valid <= 1'b0;
            x_out <= 0;
            y_out <= 0;
        end else begin
            if (pixel_valid) begin
                // Ð?c t? RAM (D? li?u này s? có ð? tr? 1 clock so v?i x_in)
                // Lýu ?: V?i LSRAM, k?t qu? ð?c s? xu?t hi?n ? chu k? sau
                r1_c2 <= linebuf0[safe_addr]; // L?y pixel d?ng trý?c
                r0_c2 <= linebuf1[safe_addr]; // L?y pixel 2 d?ng trý?c

                // Ghi vào RAM ð? ð?y d?ng lên
                linebuf0[safe_addr] <= pixel_in;
                linebuf1[safe_addr] <= linebuf0[safe_addr]; 

                // Hàng hi?n t?i l?y tr?c ti?p t? ð?u vào
                r2_c2 <= pixel_in;

                // D?ch chuy?n c?a s? ngang
                r0_c1 <= r0_c2; r0_c0 <= r0_c1;
                r1_c1 <= r1_c2; r1_c0 <= r1_c1;
                r2_c1 <= r2_c2; r2_c0 <= r2_c1;

                // Ð?ng b? t?a ð? ð?u ra
                x_out <= x_in;
                y_out <= y_in;

                // Ð?i ð? 2 d?ng và 2 pixel ð?u ð? c?a s? có d? li?u h?p l?
                if (x_in >= 2 && y_in >= 2)
                    window_valid <= 1'b1;
                else
                    window_valid <= 1'b0;
            end else begin
                window_valid <= 1'b0;
            end
        end
    end

    // Gán tr?c ti?p ra ng? ra ð? gi?m latency
    assign p00 = r0_c0; assign p01 = r0_c1; assign p02 = r0_c2;
    assign p10 = r1_c0; assign p11 = r1_c1; assign p12 = r1_c2;
    assign p20 = r2_c0; assign p21 = r2_c1; assign p22 = r2_c2;

endmodule