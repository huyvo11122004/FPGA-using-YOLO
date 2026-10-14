// apb_axi_ddr_smoke.v
// Real DDR smoke-test block for MPFS Discovery Kit.
// APB control plane + AXI4 master data plane.
//
// Purpose:
//   Verify that FPGA fabric can read/write the real MSS DDR window before
//   wiring YOLOv5n to DDR. This is not a fake DDR model and does not ask CPU
//   to move each byte. CPU only fills /dev/udmabuf, sets registers, then polls DONE.
//
// Default addresses from the user's live.dts:
//   CPU /dev/udmabuf1: 0x00000000c8000000, 32 MiB
//   Fabric AXI DMA address: 0x0000001408000000
//
// Register map, APB byte offsets:
//   0x00 CTRL        W bit0=start, W bit1=clear_done
//   0x04 STATUS      R bit0=busy, bit1=done, bit2=error
//   0x08 LEN_BYTES   RW bytes, must be multiple of DATA_BYTES
//   0x0c XOR_LO      RW low 32 bits of XOR pattern applied to output
//   0x10 SRC_LO      RW source fabric address low
//   0x14 SRC_HI      RW source fabric address high
//   0x18 DST_LO      RW destination fabric address low
//   0x1c DST_HI      RW destination fabric address high
//   0x20 CHECK_LO    R checksum low
//   0x24 CHECK_HI    R checksum high
//   0x28 ERR_COUNT   R protocol error count
//   0x2c BEATS_DONE  R completed AXI data beats

`timescale 1ns/1ps

module apb_axi_ddr_smoke #(
    parameter integer AXI_ADDR_WIDTH = 40,
    parameter integer AXI_DATA_WIDTH = 64
)(
    input  wire                         PCLK,
    input  wire                         PRESETN,

    input  wire                         PSEL,
    input  wire                         PENABLE,
    input  wire                         PWRITE,
    input  wire [11:0]                  PADDR,
    input  wire [31:0]                  PWDATA,
    output reg  [31:0]                  PRDATA,
    output wire                         PREADY,
    output wire                         PSLVERR,

    output reg  [AXI_ADDR_WIDTH-1:0]    M_AXI_AWADDR,
    output wire [7:0]                   M_AXI_AWLEN,
    output wire [2:0]                   M_AXI_AWSIZE,
    output wire [1:0]                   M_AXI_AWBURST,
    output wire                         M_AXI_AWLOCK,
    output wire [3:0]                   M_AXI_AWCACHE,
    output wire [2:0]                   M_AXI_AWPROT,
    output wire [3:0]                   M_AXI_AWQOS,
    output reg                          M_AXI_AWVALID,
    input  wire                         M_AXI_AWREADY,

    output reg  [AXI_DATA_WIDTH-1:0]    M_AXI_WDATA,
    output reg  [(AXI_DATA_WIDTH/8)-1:0] M_AXI_WSTRB,
    output reg                          M_AXI_WLAST,
    output reg                          M_AXI_WVALID,
    input  wire                         M_AXI_WREADY,

    input  wire [1:0]                   M_AXI_BRESP,
    input  wire                         M_AXI_BVALID,
    output reg                          M_AXI_BREADY,

    output reg  [AXI_ADDR_WIDTH-1:0]    M_AXI_ARADDR,
    output wire [7:0]                   M_AXI_ARLEN,
    output wire [2:0]                   M_AXI_ARSIZE,
    output wire [1:0]                   M_AXI_ARBURST,
    output wire                         M_AXI_ARLOCK,
    output wire [3:0]                   M_AXI_ARCACHE,
    output wire [2:0]                   M_AXI_ARPROT,
    output wire [3:0]                   M_AXI_ARQOS,
    output reg                          M_AXI_ARVALID,
    input  wire                         M_AXI_ARREADY,

    input  wire [AXI_DATA_WIDTH-1:0]    M_AXI_RDATA,
    input  wire [1:0]                   M_AXI_RRESP,
    input  wire                         M_AXI_RLAST,
    input  wire                         M_AXI_RVALID,
    output reg                          M_AXI_RREADY
);

localparam integer DATA_BYTES = AXI_DATA_WIDTH / 8;
localparam [31:0] DATA_BYTES_32 = DATA_BYTES;
localparam [2:0] AXI_SIZE = (DATA_BYTES == 8)  ? 3'd3 :
                            (DATA_BYTES == 4)  ? 3'd2 :
                            (DATA_BYTES == 2)  ? 3'd1 : 3'd0;

assign PREADY  = 1'b1;
assign PSLVERR = 1'b0;

assign M_AXI_AWLEN   = 8'd0;      // one beat
assign M_AXI_AWSIZE  = AXI_SIZE;
assign M_AXI_AWBURST = 2'b01;     // INCR
assign M_AXI_AWLOCK  = 1'b0;
assign M_AXI_AWCACHE = 4'b0011;   // normal, bufferable/modifiable hint
assign M_AXI_AWPROT  = 3'b000;
assign M_AXI_AWQOS   = 4'b0000;

assign M_AXI_ARLEN   = 8'd0;      // one beat
assign M_AXI_ARSIZE  = AXI_SIZE;
assign M_AXI_ARBURST = 2'b01;     // INCR
assign M_AXI_ARLOCK  = 1'b0;
assign M_AXI_ARCACHE = 4'b0011;
assign M_AXI_ARPROT  = 3'b000;
assign M_AXI_ARQOS   = 4'b0000;

reg [31:0] len_bytes;
reg [AXI_ADDR_WIDTH-1:0] src_addr;
reg [AXI_ADDR_WIDTH-1:0] dst_addr;
reg [31:0] xor_lo;
reg [63:0] checksum;
reg [31:0] err_count;
reg [31:0] beats_done;
reg [31:0] offset;
reg [AXI_DATA_WIDTH-1:0] data_hold;
reg busy;
reg done;
reg error;
reg start_pulse;

localparam [3:0]
    S_IDLE       = 4'd0,
    S_AR         = 4'd1,
    S_R          = 4'd2,
    S_AW         = 4'd3,
    S_W          = 4'd4,
    S_B          = 4'd5,
    S_FINISH     = 4'd6;

reg [3:0] state;

wire apb_access = PSEL & PENABLE;
wire apb_write  = apb_access & PWRITE;

wire [AXI_DATA_WIDTH-1:0] xor_pattern = {AXI_DATA_WIDTH/32{xor_lo}};

always @(*) begin
    case (PADDR[7:0])
        8'h00: PRDATA = 32'h0000_0000;
        8'h04: PRDATA = {29'd0, error, done, busy};
        8'h08: PRDATA = len_bytes;
        8'h0c: PRDATA = xor_lo;
        8'h10: PRDATA = src_addr[31:0];
        8'h14: PRDATA = {{(32-(AXI_ADDR_WIDTH-32)){1'b0}}, src_addr[AXI_ADDR_WIDTH-1:32]};
        8'h18: PRDATA = dst_addr[31:0];
        8'h1c: PRDATA = {{(32-(AXI_ADDR_WIDTH-32)){1'b0}}, dst_addr[AXI_ADDR_WIDTH-1:32]};
        8'h20: PRDATA = checksum[31:0];
        8'h24: PRDATA = checksum[63:32];
        8'h28: PRDATA = err_count;
        8'h2c: PRDATA = beats_done;
        default: PRDATA = 32'h0bad_f00d;
    endcase
end

always @(posedge PCLK or negedge PRESETN) begin
    if (!PRESETN) begin
        len_bytes     <= 32'd4096;
        src_addr      <= 40'h14_0800_0000;
        dst_addr      <= 40'h14_0810_0000;
        xor_lo        <= 32'h5a5a_a5a5;
        checksum      <= 64'd0;
        err_count     <= 32'd0;
        beats_done    <= 32'd0;
        offset        <= 32'd0;
        busy          <= 1'b0;
        done          <= 1'b0;
        error         <= 1'b0;
        start_pulse   <= 1'b0;
        state         <= S_IDLE;
        M_AXI_AWADDR  <= {AXI_ADDR_WIDTH{1'b0}};
        M_AXI_AWVALID <= 1'b0;
        M_AXI_WDATA   <= {AXI_DATA_WIDTH{1'b0}};
        M_AXI_WSTRB   <= {DATA_BYTES{1'b0}};
        M_AXI_WLAST   <= 1'b0;
        M_AXI_WVALID  <= 1'b0;
        M_AXI_BREADY  <= 1'b0;
        M_AXI_ARADDR  <= {AXI_ADDR_WIDTH{1'b0}};
        M_AXI_ARVALID <= 1'b0;
        M_AXI_RREADY  <= 1'b0;
        data_hold     <= {AXI_DATA_WIDTH{1'b0}};
    end else begin
        start_pulse <= 1'b0;

        if (apb_write) begin
            case (PADDR[7:0])
                8'h00: begin
                    if (PWDATA[0] && !busy) start_pulse <= 1'b1;
                    if (PWDATA[1]) begin
                        done  <= 1'b0;
                        error <= 1'b0;
                    end
                end
                8'h08: len_bytes <= PWDATA;
                8'h0c: xor_lo    <= PWDATA;
                8'h10: src_addr[31:0] <= PWDATA;
                8'h14: src_addr[AXI_ADDR_WIDTH-1:32] <= PWDATA[AXI_ADDR_WIDTH-33:0];
                8'h18: dst_addr[31:0] <= PWDATA;
                8'h1c: dst_addr[AXI_ADDR_WIDTH-1:32] <= PWDATA[AXI_ADDR_WIDTH-33:0];
                default: ;
            endcase
        end

        case (state)
            S_IDLE: begin
                M_AXI_AWVALID <= 1'b0;
                M_AXI_WVALID  <= 1'b0;
                M_AXI_WLAST   <= 1'b0;
                M_AXI_BREADY  <= 1'b0;
                M_AXI_ARVALID <= 1'b0;
                M_AXI_RREADY  <= 1'b0;
                if (start_pulse) begin
                    busy       <= 1'b1;
                    done       <= 1'b0;
                    error      <= 1'b0;
                    checksum   <= 64'd0;
                    err_count  <= 32'd0;
                    beats_done <= 32'd0;
                    offset     <= 32'd0;
                    if (len_bytes == 32'd0) begin
                        state <= S_FINISH;
                    end else begin
                        M_AXI_ARADDR  <= src_addr;
                        M_AXI_ARVALID <= 1'b1;
                        state         <= S_AR;
                    end
                end
            end

            S_AR: begin
                if (M_AXI_ARVALID && M_AXI_ARREADY) begin
                    M_AXI_ARVALID <= 1'b0;
                    M_AXI_RREADY  <= 1'b1;
                    state         <= S_R;
                end
            end

            S_R: begin
                if (M_AXI_RVALID && M_AXI_RREADY) begin
                    data_hold    <= M_AXI_RDATA;
                    checksum     <= checksum + M_AXI_RDATA[31:0];
                    if (M_AXI_RRESP != 2'b00) begin
                        error     <= 1'b1;
                        err_count <= err_count + 1'b1;
                    end
                    M_AXI_RREADY <= 1'b0;
                    
                    // Ph t d?a ch? AWADDR v  AWVALID
                    M_AXI_AWADDR  <= dst_addr + {{(AXI_ADDR_WIDTH-32){1'b0}}, offset};
                    M_AXI_AWVALID <= 1'b1;
                    
                    // Ph t WDATA v  WVALID c ng l c (Kh ng ch? AWREADY)
                    M_AXI_WDATA   <= M_AXI_RDATA ^ xor_pattern;
                    M_AXI_WSTRB   <= {DATA_BYTES{1'b1}};
                    M_AXI_WLAST   <= 1'b1;
                    M_AXI_WVALID  <= 1'b1;
                    
                    state <= S_AW;
                end
            end

            S_AW: begin
                // N?u k nh d?a ch? d  du?c Master nh?n (AWREADY = 1), th  h? c? AWVALID
                if (M_AXI_AWREADY) begin
                    M_AXI_AWVALID <= 1'b0;
                end
                
                // N?u k nh d? li?u d  du?c Master nh?n (WREADY = 1), th  h? c? WVALID
                if (M_AXI_WREADY) begin
                    M_AXI_WVALID <= 1'b0;
                    M_AXI_WLAST  <= 1'b0;
                end

                // Chuy?n sang S_B khi C? 2 k nh d?u d  ho n t?t b?t tay
                if ((!M_AXI_AWVALID || M_AXI_AWREADY) && (!M_AXI_WVALID || M_AXI_WREADY)) begin
                    M_AXI_BREADY <= 1'b1;
                    state        <= S_B;
                end
            end

            S_B: begin
                if (M_AXI_BVALID && M_AXI_BREADY) begin
                    M_AXI_BREADY <= 1'b0;
                    if (M_AXI_BRESP != 2'b00) begin
                        error     <= 1'b1;
                        err_count <= err_count + 1'b1;
                    end
                    beats_done <= beats_done + 1'b1;
                    offset <= offset + DATA_BYTES_32;
                    if ((offset + DATA_BYTES_32) >= len_bytes) begin
                        state <= S_FINISH;
                    end else begin
                        M_AXI_ARADDR  <= src_addr + {{(AXI_ADDR_WIDTH-32){1'b0}}, (offset + DATA_BYTES_32)};
                        M_AXI_ARVALID <= 1'b1;
                        state         <= S_AR;
                    end
                end
            end

            S_FINISH: begin
                busy  <= 1'b0;
                done  <= 1'b1;
                state <= S_IDLE;
            end

            default: begin
                state <= S_IDLE;
                busy  <= 1'b0;
                error <= 1'b1;
            end
        endcase
    end
end

endmodule
