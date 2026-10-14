module mss_irq_pack (
    input  wire        ihc_e51_irq,
    input  wire        ihc_u54_1_irq,
    input  wire        ihc_u54_2_irq,
    input  wire        ihc_u54_3_irq,
    input  wire        ihc_u54_4_irq,
    input  wire        dma_irq,

    output wire [63:0] mss_int_f2m
);

assign mss_int_f2m = {
    ihc_e51_irq,       // bit 63
    ihc_u54_1_irq,     // bit 62
    ihc_u54_2_irq,     // bit 61
    ihc_u54_3_irq,     // bit 60
    ihc_u54_4_irq,     // bit 59
    56'd0,             // bit 58:3
    dma_irq,           // bit 2
    2'b00              // bit 1:0
};

endmodule