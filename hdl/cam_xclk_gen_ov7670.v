module cam_xclk_gen_ov7670 (
    input  wire clk_in,
    input  wire rst_n,
    output wire xclk,
    output wire clk_50m,
    output wire locked
);
    // Libero SmartDesign component exported in cam_xclk_24m_ccc_work.ixf
    // Exported top-level ports are: clk_in, rst_n, xclk, locked
  cam_xclk_24m_ccc u_cam_xclk_24m_ccc (
    .REF_CLK_0         (clk_in),
    .PLL_POWERDOWN_N_0 (rst_n),

    // OUT0 ph?i ðý?c c?u h?nh 24 MHz
    .OUT0_FABCLK_0     (xclk),

    // OUT1 ph?i ðý?c c?u h?nh 50 MHz
    .OUT1_FABCLK_0     (clk_50m),

    .PLL_LOCK_0        (locked)
);
endmodule
