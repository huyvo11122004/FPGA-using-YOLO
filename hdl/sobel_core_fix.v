module sobel_core_fix (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        window_valid,
    input  wire [7:0]  w00, w01, w02,
    input  wire [7:0]  w10, w11, w12,
    input  wire [7:0]  w20, w21, w22,

    output reg         grad_valid,
    output reg  [10:0] grad_mag
);
    reg signed [10:0] gx_s1, gy_s1;
    reg [10:0]        abs_gx_s2, abs_gy_s2;
    reg               v1, v2, v3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gx_s1      <= 11'sd0;
            gy_s1      <= 11'sd0;
            abs_gx_s2  <= 11'd0;
            abs_gy_s2  <= 11'd0;
            grad_mag   <= 11'd0;
            v1         <= 1'b0;
            v2         <= 1'b0;
            v3         <= 1'b0;
            grad_valid <= 1'b0;
        end else begin
            v1 <= window_valid;
            v2 <= v1;
            v3 <= v2;
            grad_valid <= v3;

            if (window_valid) begin
                gx_s1 <= ($signed({3'b0, w02}) + $signed({2'b0, w12, 1'b0}) + $signed({3'b0, w22})) -
                         ($signed({3'b0, w00}) + $signed({2'b0, w10, 1'b0}) + $signed({3'b0, w20}));

                gy_s1 <= ($signed({3'b0, w20}) + $signed({2'b0, w21, 1'b0}) + $signed({3'b0, w22})) -
                         ($signed({3'b0, w00}) + $signed({2'b0, w01, 1'b0}) + $signed({3'b0, w02}));
            end

            if (v1) begin
                abs_gx_s2 <= (gx_s1 < 0) ? -gx_s1 : gx_s1;
                abs_gy_s2 <= (gy_s1 < 0) ? -gy_s1 : gy_s1;
            end

            if (v2) begin
                grad_mag <= abs_gx_s2 + abs_gy_s2;
            end
        end
    end
endmodule
