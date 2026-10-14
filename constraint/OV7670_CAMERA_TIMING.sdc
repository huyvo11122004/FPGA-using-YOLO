# OV7670 timing constraints.
# The old working camera project used 42 ns for camera PCLK.
create_clock -name {CAM_PCLK} -period 42.000 [get_ports {CAM_PCLK}]

# CAM_PCLK is returned by the external sensor and has no guaranteed phase
# relationship to the FIC3/YOLO clock.  The HDL crosses these domains only
# through a dual-clock RAM, toggle handshakes and two-flop synchronizers.
set_clock_groups -asynchronous \
    -group [get_clocks {CAM_PCLK}] \
    -group [get_clocks {CLOCKS_AND_RESETS_0/CCC_FIC_x_CLK/PF_CCC_C0_0/pll_inst_0/OUT3}]

# CAM_XCLK is an output clock to the sensor, not a timed data output.
set_false_path -to [get_ports {CAM_XCLK}]
