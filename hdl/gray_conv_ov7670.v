module gray_conv_ov7670 #(
    parameter integer BYPASS_Y = 1
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       pixel_valid_in,
    input  wire [7:0] pixel_in,
    output reg        pixel_valid_out,
    output reg  [7:0] gray_out
);
    wire [7:0] rgb332_gray;
    
    // N?u v? t?nh camera x?i RGB332 th? d?ng c?ng th?c tr?n c?ng ?? t?nh ?? s?ng
    assign rgb332_gray = {2'b00, pixel_in[7:5], 3'b000} +
                         {3'b000, pixel_in[4:2], 2'b00} +
                         {4'b0000, pixel_in[1:0], 2'b00};

    // M?ch t? h?p thu?n t?y (Combinational logic)
    always @(*) begin
        pixel_valid_out = pixel_valid_in;
        if (pixel_valid_in)
            // N?u l? YUV (Bypass), pixel ?? l? gray r?i, chuy?n th?ng qua
            gray_out = (BYPASS_Y != 0) ? pixel_in : rgb332_gray;
        else
            gray_out = 8'd0;
    end
endmodule