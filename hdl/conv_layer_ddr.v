// conv_layer_ddr.v
// DDR-weight version: no hard-coded .mem ROMs in FPGA fabric.
// Weight/bias bytes are fetched by conv2d_ddr_core from DDR offsets.

module conv_layer_ddr #(
    parameter DATA_W    = 8,
    parameter ADDR_W    = 32,
    parameter ACC_W  = 32,

    parameter IN_CH     = 64,
    parameter OUT_CH    = 128,
    parameter IN_W      = 4,
    parameter IN_H      = 4,
    parameter OUT_W     = 2,
    parameter OUT_H     = 2,
    parameter K_SIZE    = 3,
    parameter STRIDE    = 2,
    parameter PAD       = 1,

    parameter W_NUM     = OUT_CH * IN_CH * K_SIZE * K_SIZE,
    parameter B_NUM     = OUT_CH,
    parameter W_BASE    = 32'h0390_0000,
    parameter B_BASE    = 32'h0390_0000,

    // Temporary global output shift for signed-int8 feature maps.
    // Real YOLO quantization should replace this with per-layer scale/shift values.
    parameter REQUANT_SHIFT = 10,

    parameter MAC_LANES = 4
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,

    input  wire [ADDR_W-1:0]       input_base,
    input  wire [ADDR_W-1:0]       output_base,

    output wire                    mem_rd_req,
    output wire [ADDR_W-1:0]       mem_rd_addr,
    input  wire                    mem_rd_ready,
    input  wire                    mem_rd_valid,
    input  wire [DATA_W-1:0]       mem_rd_data,

    output wire                    mem_wr_req,
    output wire [ADDR_W-1:0]       mem_wr_addr,
    output wire [DATA_W-1:0]       mem_wr_data,
    input  wire                    mem_wr_ready,

    output wire                    busy,
    output wire                    done,

    output wire                    out_valid,
    output wire signed [ACC_W-1:0] out_data,

    output wire                    final_wr_en,
    output wire [ADDR_W-1:0]       final_wr_addr,
    output wire [DATA_W-1:0]       final_wr_data
);

    localparam [4:0] REQUANT_SHIFT_W = REQUANT_SHIFT;

    conv2d_ddr_core #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .ACC_W(ACC_W),
        .MAX_W_NUM(W_NUM),
        .MAX_B_NUM(B_NUM),
        .MAC_LANES(MAC_LANES)
    ) u_core (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),

        .in_ch(IN_CH[15:0]),
        .out_ch(OUT_CH[15:0]),
        .in_w(IN_W[15:0]),
        .in_h(IN_H[15:0]),
        .out_w(OUT_W[15:0]),
        .out_h(OUT_H[15:0]),
        .k_size(K_SIZE[3:0]),
        .stride(STRIDE[3:0]),
        .pad(PAD[3:0]),

        .input_base(input_base),
        .output_base(output_base),
        .weight_base(W_BASE[ADDR_W-1:0]),
        .bias_base(B_BASE[ADDR_W-1:0]),
        .out_shift(REQUANT_SHIFT_W),

        .mem_rd_req(mem_rd_req),
        .mem_rd_addr(mem_rd_addr),
        .mem_rd_ready(mem_rd_ready),
        .mem_rd_valid(mem_rd_valid),
        .mem_rd_data(mem_rd_data),

        .mem_wr_req(mem_wr_req),
        .mem_wr_addr(mem_wr_addr),
        .mem_wr_data(mem_wr_data),
        .mem_wr_ready(mem_wr_ready),

        .busy(busy),
        .done(done),

        .out_valid(out_valid),
        .out_data(out_data),

        .final_wr_en(final_wr_en),
        .final_wr_addr(final_wr_addr),
        .final_wr_data(final_wr_data)
    );

endmodule
