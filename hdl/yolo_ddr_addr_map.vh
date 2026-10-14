// yolo_ddr_addr_map.vh
// DDR local byte-offset map for top_yolov5n_ddr.v (256x256 signed-int8 CHW).
//
// Address regions used by this design:
//   0x00000000 - 0x00217fff : input, persistent feature maps and scratch
//   0x01000000 - 0x011c2c97 : quantized weights/biases (weight map supplied)
//   0x01c00000 - ...        : raw detection records (wrapper default)
//
// All feature-map bases are 4 KiB aligned unless a smaller naturally aligned
// boundary is sufficient. Each feature value occupies one byte in CHW order.

// Input tensor: 3 x 256 x 256 = 0x00030000 bytes.
localparam [31:0] Y_BASE_INPUT = 32'h0000_0000;

// Persistent backbone/head feature maps.
localparam [31:0] Y_BASE_L0    = 32'h0003_0000; // 16  x 128 x 128 = 0x40000
localparam [31:0] Y_BASE_L1    = 32'h0007_0000; // 32  x 64  x 64  = 0x20000
localparam [31:0] Y_BASE_L2    = 32'h0009_0000; // 32  x 64  x 64  = 0x20000
localparam [31:0] Y_BASE_L3    = 32'h000b_0000; // 64  x 32  x 32  = 0x10000

// L4 and L6 must remain alive for the later PAN/FPN concatenations.
localparam [31:0] Y_L4_SLOT1   = 32'h000c_0000; // 64  x 32  x 32  = 0x10000
localparam [31:0] Y_BASE_L4    = Y_L4_SLOT1;    // compatibility alias
localparam [31:0] Y_BASE_L5    = 32'h000d_0000; // 128 x 16  x 16  = 0x08000
localparam [31:0] Y_L6_SLOT1   = 32'h000d_8000; // 128 x 16  x 16  = 0x08000
localparam [31:0] Y_BASE_L6    = Y_L6_SLOT1;    // compatibility alias

localparam [31:0] Y_BASE_L7    = 32'h000e_0000; // 256 x 8   x 8   = 0x04000
localparam [31:0] Y_BASE_L8    = 32'h000e_4000; // 256 x 8   x 8   = 0x04000
localparam [31:0] Y_BASE_L9    = 32'h000e_8000; // 256 x 8   x 8   = 0x04000
localparam [31:0] Y_BASE_L10   = 32'h000e_c000; // 128 x 8   x 8   = 0x02000
localparam [31:0] Y_BASE_L11   = 32'h000e_e000; // 128 x 16  x 16  = 0x08000
localparam [31:0] Y_BASE_L12   = 32'h000f_6000; // 256 x 16  x 16  = 0x10000
localparam [31:0] Y_BASE_L13   = 32'h0010_6000; // 128 x 16  x 16  = 0x08000
localparam [31:0] Y_BASE_L14   = 32'h0010_e000; // 64  x 16  x 16  = 0x04000
localparam [31:0] Y_BASE_L15   = 32'h0011_2000; // 64  x 32  x 32  = 0x10000
localparam [31:0] Y_BASE_L16   = 32'h0012_2000; // 128 x 32  x 32  = 0x20000
localparam [31:0] Y_BASE_L17   = 32'h0014_2000; // 64  x 32  x 32  = 0x10000
localparam [31:0] Y_BASE_L18   = 32'h0015_2000; // 64  x 16  x 16  = 0x04000
localparam [31:0] Y_BASE_L19   = 32'h0015_6000; // 128 x 16  x 16  = 0x08000
localparam [31:0] Y_BASE_L20   = 32'h0015_e000; // 128 x 16  x 16  = 0x08000
localparam [31:0] Y_BASE_L21   = 32'h0016_6000; // 128 x 8   x 8   = 0x02000
localparam [31:0] Y_BASE_L22   = 32'h0016_8000; // 256 x 8   x 8   = 0x04000
localparam [31:0] Y_BASE_L23   = 32'h0016_c000; // 256 x 8   x 8   = 0x04000

// Stage-A scratch banks. Sizes cover their largest use in ops 0..33.
localparam [31:0] Y_TMP0       = 32'h0017_0000; // 0x10000 bytes
localparam [31:0] Y_TMP1       = 32'h0018_0000; // 0x10000 bytes
localparam [31:0] Y_TMP2       = 32'h0019_0000; // 0x10000 bytes
localparam [31:0] Y_TMP3       = 32'h001a_0000; // 0x10000 bytes
localparam [31:0] Y_TMP4       = 32'h001b_0000; // 0x10000 bytes
localparam [31:0] Y_TMP5       = 32'h001c_0000; // 0x20000 bytes

// Stage-B scratch banks. Kept separate from Stage A and persistent skip maps.
localparam [31:0] Y_B_TMP0     = 32'h001e_0000; // 0x08000 bytes
localparam [31:0] Y_B_TMP1     = 32'h001e_8000; // 0x08000 bytes
localparam [31:0] Y_B_TMP2     = 32'h001f_0000; // 0x08000 bytes
localparam [31:0] Y_B_TMP3     = 32'h001f_8000; // 0x08000 bytes
localparam [31:0] Y_B_TMP4     = 32'h0020_0000; // 0x08000 bytes
localparam [31:0] Y_B_TMP5     = 32'h0020_8000; // 0x10000 bytes

// First byte after all activation/scratch allocations.
localparam [31:0] Y_ACTIVATION_END = 32'h0021_8000;
