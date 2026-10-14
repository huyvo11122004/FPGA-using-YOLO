// FIX221B_RESTORE_WR64_FASTPATH_FOR_DET: keep mem_wr64 fast to avoid det_overflow.
// FIX221_WR_ADDR_DEEP_PIPE_75: add internal write address pipeline for 75MHz smoke.
// yolo_mem_axi_bridge.v
// Byte-oriented YOLO memory interface -> AXI4 single-beat 64-bit master.
// STEP2C optimization:
//   - Keeps external YOLO byte protocol unchanged.
//   - Keeps STEP2A registered request stage.
//   - Expands direct-mapped read cache from 4 entries to 16 entries.
//   - Cache line remains 64-bit/8-byte, matching AXI data width.
//   - Invalidates matching cache line on byte write.
//   - Does NOT modify conv/datapath and does NOT coalesce writes yet.

`timescale 1ns/1ps

module yolo_mem_axi_bridge #(
    parameter integer AXI_ADDR_WIDTH = 40,
    parameter integer AXI_DATA_WIDTH = 64,
    parameter integer ADDR_W = 32,
    parameter integer DATA_W = 8
)(
    input  wire                         clk,
    input  wire                         rst_n,

    input  wire [AXI_ADDR_WIDTH-1:0]    ddr_base,

    input  wire                         mem_rd_req,
    input  wire [ADDR_W-1:0]            mem_rd_addr,
    output reg                          mem_rd_ready,
    output reg                          mem_rd_valid,
    output reg  [DATA_W-1:0]            mem_rd_data,

    // FIX164_MAC4_WEIGHT_PACK32: optional 32-bit read sideband sharing the same cache/AXI datapath.
    input  wire                         mem_rd32_req,
    input  wire [ADDR_W-1:0]            mem_rd32_addr,
    output reg                          mem_rd32_ready,
    output reg                          mem_rd32_valid,
    output reg  [31:0]                  mem_rd32_data,

    // FIX164_MAC8_VECTOR_PACK64: optional 64-bit read sideband for packed MAC8 weights.
    input  wire                         mem_rd64_req,
    input  wire [ADDR_W-1:0]            mem_rd64_addr,
    output reg                          mem_rd64_ready,
    output reg                          mem_rd64_valid,
    output reg  [63:0]                  mem_rd64_data,

    input  wire                         mem_wr_req,
    input  wire [ADDR_W-1:0]            mem_wr_addr,
    input  wire [DATA_W-1:0]            mem_wr_data,
    output reg                          mem_wr_ready,

    
      // FIX187E_DET_RECORD_WR64_COMPLETE: optional aligned 64-bit write sideband.
      input  wire                         mem_wr64_req,
      input  wire [ADDR_W-1:0]            mem_wr64_addr,
      input  wire [63:0]                  mem_wr64_data,
      output reg                          mem_wr64_ready,
output reg                          busy,
    output reg                          error,
    output reg  [31:0]                  rd_count,
    output reg  [31:0]                  wr_count,
    // FRAME_AXI_FIX_V2: diagnostic-only; wrapper external ports stay unchanged.
    output wire [31:0]                  debug_status,
    output reg [AXI_ADDR_WIDTH-1:0]     first_error_addr,
    output wire [AXI_ADDR_WIDTH-1:0]    last_write_addr,

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
localparam [2:0] AXI_SIZE = (DATA_BYTES == 8)  ? 3'd3 :
                            (DATA_BYTES == 4)  ? 3'd2 :
                            (DATA_BYTES == 2)  ? 3'd1 : 3'd0;

assign M_AXI_AWLEN   = 8'd0;
assign M_AXI_AWSIZE  = AXI_SIZE;
assign M_AXI_AWBURST = 2'b01;
assign M_AXI_AWLOCK  = 1'b0;
assign M_AXI_AWCACHE = 4'b0000;
assign M_AXI_AWPROT  = 3'b000;
assign M_AXI_AWQOS   = 4'b0000;

reg [7:0] arlen_r; // FIX164_MAC16_RDADDR_PIPE_BURST2: ARLEN=1 for rd64 beat-pair prefetch.
assign M_AXI_ARLEN   = arlen_r; // FIX164_MAC16_RDADDR_PIPE_BURST2
assign M_AXI_ARSIZE  = AXI_SIZE;
assign M_AXI_ARBURST = 2'b01;
assign M_AXI_ARLOCK  = 1'b0;
assign M_AXI_ARCACHE = 4'b0000;
assign M_AXI_ARPROT  = 3'b000;
assign M_AXI_ARQOS   = 4'b0000;

localparam [3:0]
    S_IDLE       = 4'd0,
    S_RD_ADDR    = 4'd1,
    S_RD_LOOKUP  = 4'd2,
    S_AR         = 4'd3,
    S_R          = 4'd4,
    S_WR_ISSUE   = 4'd5,
    S_AW_W       = 4'd6,
    S_B          = 4'd7,
    S_CACHE_RESP = 4'd8,
    S_WR_ADDR    = 4'd9,
    S_WR_ACK     = 4'd10;

reg [3:0] state;
reg [2:0] rd_lane;
reg       rd32_r; // FIX164_MAC4_WEIGHT_PACK32: selected request is 32-bit
reg       rd64_r; // FIX164_MAC8_VECTOR_PACK64: selected request is 64-bit
reg [ADDR_W-1:0] rd_local_addr_r; // FIX164_MAC16_RDADDR_PIPE_BURST2: cuts conv address math -> bridge AXI addr timing path
reg [3:0] rd_burst_cnt;           // FIX164_MAC16_RDADDR_PIPE_BURST2
reg [3:0] rd_burst_len;           // FIX164_MAC16_RDADDR_PIPE_BURST2
reg [2:0] wr_lane_r;
reg [7:0] wr_data_r;
  reg wr64_r; // FIX187E_DET_RECORD_WR64_COMPLETE
  reg [63:0] wr64_data_r; // FIX187E_DET_RECORD_WR64_COMPLETE
  (* syn_preserve = 1, syn_keep = 1 *) reg [ADDR_W-1:0] wr_local_addr_r; // FIX221_WR_ADDR_DEEP_PIPE_75
reg aw_seen;
reg w_seen;
reg [1:0] last_bresp, last_rresp, first_resp;
reg b_seen, r_seen, first_error_valid, first_error_write;
assign last_write_addr = M_AXI_AWADDR;
assign debug_status = {1'b1, 2'b00, first_resp, wr64_r, busy, error,
                       w_seen, aw_seen, M_AXI_RREADY, M_AXI_RVALID,
                       M_AXI_ARREADY, M_AXI_ARVALID, M_AXI_BREADY,
                       M_AXI_BVALID, M_AXI_WREADY, M_AXI_WVALID,
                       M_AXI_AWREADY, M_AXI_AWVALID, r_seen, b_seen,
                       first_error_write, first_error_valid,
                       last_rresp, last_bresp, state};
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        last_bresp <= 0; last_rresp <= 0; first_resp <= 0;
        b_seen <= 0; r_seen <= 0; first_error_valid <= 0;
        first_error_write <= 0; first_error_addr <= 0;
    end else begin
        if (M_AXI_BVALID && M_AXI_BREADY) begin
            b_seen <= 1; last_bresp <= M_AXI_BRESP;
            if ((M_AXI_BRESP != 0) && !first_error_valid) begin
                first_error_valid <= 1; first_error_write <= 1;
                first_resp <= M_AXI_BRESP; first_error_addr <= M_AXI_AWADDR;
            end
        end
        if (M_AXI_RVALID && M_AXI_RREADY) begin
            r_seen <= 1; last_rresp <= M_AXI_RRESP;
            if ((M_AXI_RRESP != 0) && !first_error_valid) begin
                first_error_valid <= 1; first_error_write <= 0;
                first_resp <= M_AXI_RRESP; first_error_addr <= rd_burst_axi_addr;
            end
        end
    end
end

wire [AXI_ADDR_WIDTH-1:0] rd_byte_addr = ddr_base + {{(AXI_ADDR_WIDTH-ADDR_W){1'b0}}, mem_rd_addr};
wire [AXI_ADDR_WIDTH-1:0] wr_byte_addr = ddr_base + {{(AXI_ADDR_WIDTH-ADDR_W){1'b0}}, mem_wr_addr};
  wire [AXI_ADDR_WIDTH-1:0] wr64_byte_addr = ddr_base + {{(AXI_ADDR_WIDTH-ADDR_W){1'b0}}, mem_wr64_addr}; // FIX187E_DET_RECORD_WR64_COMPLETE
wire [AXI_ADDR_WIDTH-1:0] rd32_byte_addr = ddr_base + {{(AXI_ADDR_WIDTH-ADDR_W){1'b0}}, mem_rd32_addr}; // FIX164_MAC4_WEIGHT_PACK32
wire [AXI_ADDR_WIDTH-1:0] rd64_byte_addr = ddr_base + {{(AXI_ADDR_WIDTH-ADDR_W){1'b0}}, mem_rd64_addr}; // FIX164_MAC8_VECTOR_PACK64
wire [AXI_ADDR_WIDTH-1:0] rd_axi_addr  = {rd_byte_addr[AXI_ADDR_WIDTH-1:3], 3'b000};
wire [AXI_ADDR_WIDTH-1:0] rd32_axi_addr = {rd32_byte_addr[AXI_ADDR_WIDTH-1:3], 3'b000}; // FIX164_MAC4_WEIGHT_PACK32
wire [AXI_ADDR_WIDTH-1:0] rd64_axi_addr = {rd64_byte_addr[AXI_ADDR_WIDTH-1:3], 3'b000}; // FIX164_MAC8_VECTOR_PACK64
wire [AXI_ADDR_WIDTH-1:0] rd_pipe_byte_addr = ddr_base + {{(AXI_ADDR_WIDTH-ADDR_W){1'b0}}, rd_local_addr_r}; // FIX164_MAC16_RDADDR_PIPE_BURST2
wire [AXI_ADDR_WIDTH-1:0] rd_pipe_axi_addr  = {rd_pipe_byte_addr[AXI_ADDR_WIDTH-1:3], 3'b000}; // FIX164_MAC16_RDADDR_PIPE_BURST2
wire [3:0] rd_burst_cache_idx = rd_axi_addr_r[6:3] + rd_burst_cnt[3:0]; // FIX164_MAC16_RDADDR_PIPE_BURST2
wire [AXI_ADDR_WIDTH-1:0] rd_burst_axi_addr = rd_axi_addr_r + {{(AXI_ADDR_WIDTH-7){1'b0}}, rd_burst_cnt[3:0], 3'b000}; // FIX164_MAC16_RDADDR_PIPE_BURST2
wire [AXI_ADDR_WIDTH-1:0] wr_axi_addr  = {wr_byte_addr[AXI_ADDR_WIDTH-1:3], 3'b000};
  wire [AXI_ADDR_WIDTH-1:0] wr64_axi_addr = {wr64_byte_addr[AXI_ADDR_WIDTH-1:3], 3'b000}; // FIX187E_DET_RECORD_WR64_COMPLETE
wire [AXI_ADDR_WIDTH-1:0] wr_pipe_byte_addr = ddr_base + {{(AXI_ADDR_WIDTH-ADDR_W){1'b0}}, wr_local_addr_r}; // FIX221_WR_ADDR_DEEP_PIPE_75
wire [AXI_ADDR_WIDTH-1:0] wr_pipe_axi_addr  = {wr_pipe_byte_addr[AXI_ADDR_WIDTH-1:3], 3'b000}; // FIX221_WR_ADDR_DEEP_PIPE_75

// Registered request boundary. These registers intentionally break long paths
// from the YOLO datapath/op_id/address logic into AXI address/data/cache logic.
reg [AXI_ADDR_WIDTH-1:0] rd_axi_addr_r;
reg [AXI_ADDR_WIDTH-1:0] wr_axi_addr_r;
reg [3:0] rd_cache_idx_r;
reg [3:0] wr_cache_idx_r;

// 16 direct-mapped 64-bit cache lines, indexed by aligned address bits [6:3].
// This is still a tiny register-file cache, but it reduces conflict misses when
// the YOLO core interleaves activation/weight reads across several nearby lines.
reg [15:0] cache_valid;
reg [AXI_ADDR_WIDTH-1:0] cache_addr [0:15];
reg [AXI_DATA_WIDTH-1:0] cache_data [0:15];
reg [AXI_DATA_WIDTH-1:0] cache_hit_data_r;
integer cache_i;

function [31:0] pick_word32;
    input [AXI_DATA_WIDTH-1:0] din;
    input [2:0] lane;
    begin
        case (lane[2])
            1'b0: pick_word32 = din[31:0];
            default: pick_word32 = din[63:32];
        endcase
    end
endfunction

function [7:0] pick_byte;
    input [AXI_DATA_WIDTH-1:0] din;
    input [2:0] lane;
    begin
        case (lane)
            3'd0: pick_byte = din[7:0];
            3'd1: pick_byte = din[15:8];
            3'd2: pick_byte = din[23:16];
            3'd3: pick_byte = din[31:24];
            3'd4: pick_byte = din[39:32];
            3'd5: pick_byte = din[47:40];
            3'd6: pick_byte = din[55:48];
            default: pick_byte = din[63:56];
        endcase
    end
endfunction

function [AXI_DATA_WIDTH-1:0] put_byte_data;
    input [7:0] b;
    input [2:0] lane;
    reg [AXI_DATA_WIDTH-1:0] tmp;
    begin
        tmp = {AXI_DATA_WIDTH{1'b0}};
        case (lane)
            3'd0: tmp[7:0]   = b;
            3'd1: tmp[15:8]  = b;
            3'd2: tmp[23:16] = b;
            3'd3: tmp[31:24] = b;
            3'd4: tmp[39:32] = b;
            3'd5: tmp[47:40] = b;
            3'd6: tmp[55:48] = b;
            default: tmp[63:56] = b;
        endcase
        put_byte_data = tmp;
    end
endfunction

function [(AXI_DATA_WIDTH/8)-1:0] put_byte_strb;
    input [2:0] lane;
    reg [(AXI_DATA_WIDTH/8)-1:0] tmp;
    begin
        tmp = {(AXI_DATA_WIDTH/8){1'b0}};
        tmp[lane] = 1'b1;
        put_byte_strb = tmp;
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= S_IDLE;
        mem_rd_ready <= 1'b0;
        mem_rd_valid <= 1'b0;
        mem_rd_data <= {DATA_W{1'b0}};
        mem_rd32_ready <= 1'b0; // FIX164_MAC4_WEIGHT_PACK32
        mem_rd32_valid <= 1'b0; // FIX164_MAC4_WEIGHT_PACK32
        mem_rd32_data <= 32'd0; // FIX164_MAC4_WEIGHT_PACK32
        mem_rd64_ready <= 1'b0; // FIX164_MAC8_VECTOR_PACK64
        mem_rd64_valid <= 1'b0; // FIX164_MAC8_VECTOR_PACK64
        mem_rd64_data <= 64'd0; // FIX164_MAC8_VECTOR_PACK64
        mem_wr_ready <= 1'b0;
          mem_wr64_ready <= 1'b0; // FIX187E_DET_RECORD_WR64_COMPLETE
        busy <= 1'b0;
        error <= 1'b0;
        rd_count <= 32'd0;
        wr_count <= 32'd0;
        M_AXI_AWADDR <= {AXI_ADDR_WIDTH{1'b0}};
        M_AXI_AWVALID <= 1'b0;
        M_AXI_WDATA <= {AXI_DATA_WIDTH{1'b0}};
        M_AXI_WSTRB <= {(AXI_DATA_WIDTH/8){1'b0}};
        M_AXI_WLAST <= 1'b0;
        M_AXI_WVALID <= 1'b0;
        M_AXI_BREADY <= 1'b0;
        M_AXI_ARADDR <= {AXI_ADDR_WIDTH{1'b0}};
        M_AXI_ARVALID <= 1'b0;
        M_AXI_RREADY <= 1'b0;
        arlen_r <= 8'd0; // FIX164_MAC16_RDADDR_PIPE_BURST2
        rd_local_addr_r <= {ADDR_W{1'b0}}; // FIX164_MAC16_RDADDR_PIPE_BURST2
        rd_burst_cnt <= 4'd0; // FIX164_MAC16_RDADDR_PIPE_BURST2
        rd_burst_len <= 4'd0; // FIX164_MAC16_RDADDR_PIPE_BURST2
        rd_lane <= 3'd0;
        rd32_r <= 1'b0; // FIX164_MAC4_WEIGHT_PACK32
        rd64_r <= 1'b0; // FIX164_MAC8_VECTOR_PACK64
        wr_lane_r <= 3'd0;
        wr_data_r <= 8'd0;
          wr64_r <= 1'b0; // FIX187E_DET_RECORD_WR64_COMPLETE
          wr64_data_r <= 64'd0; // FIX187E_DET_RECORD_WR64_COMPLETE
        wr_local_addr_r <= {ADDR_W{1'b0}}; // FIX221_WR_ADDR_DEEP_PIPE_75
        aw_seen <= 1'b0;
        w_seen <= 1'b0;
        rd_axi_addr_r <= {AXI_ADDR_WIDTH{1'b0}};
        wr_axi_addr_r <= {AXI_ADDR_WIDTH{1'b0}};
        rd_cache_idx_r <= 4'd0;
        wr_cache_idx_r <= 4'd0;
        cache_valid <= 16'h0000;
        for (cache_i = 0; cache_i < 16; cache_i = cache_i + 1) begin
            cache_addr[cache_i] <= {AXI_ADDR_WIDTH{1'b0}};
            cache_data[cache_i] <= {AXI_DATA_WIDTH{1'b0}};
        end
        cache_hit_data_r <= {AXI_DATA_WIDTH{1'b0}};
    end else begin
        mem_rd_ready <= 1'b0;
        mem_rd_valid <= 1'b0;
        mem_rd32_ready <= 1'b0; // FIX164_MAC4_WEIGHT_PACK32
        mem_rd32_valid <= 1'b0; // FIX164_MAC4_WEIGHT_PACK32
        mem_rd64_ready <= 1'b0; // FIX164_MAC8_VECTOR_PACK64
        mem_rd64_valid <= 1'b0; // FIX164_MAC8_VECTOR_PACK64
        mem_wr_ready <= 1'b0;
          mem_wr64_ready <= 1'b0; // FIX187E_DET_RECORD_WR64_COMPLETE
        busy <= (state != S_IDLE);

        case (state)
            S_IDLE: begin
                M_AXI_AWVALID <= 1'b0;
                M_AXI_WVALID <= 1'b0;
                M_AXI_WLAST <= 1'b0;
                M_AXI_BREADY <= 1'b0;
                M_AXI_ARVALID <= 1'b0;
                M_AXI_RREADY <= 1'b0;
                aw_seen <= 1'b0;
                w_seen <= 1'b0;

                // Write has priority. Capture request first; issue AXI from registers next cycle.
                if (mem_wr64_req) begin // FIX187E_DET_RECORD_WR64_COMPLETE
                      // FIX221B_RESTORE_WR64_FASTPATH_FOR_DET:
                      // DET raw writer uses 64-bit writes as a continuous stream.
                      // Do NOT add S_WR_ADDR bubble here, otherwise det_overflow drops records.
                      wr_axi_addr_r  <= wr64_axi_addr;
                      wr_cache_idx_r <= wr64_axi_addr[6:3];
                      wr_lane_r      <= 3'd0;
                      wr_data_r      <= 8'd0;
                      wr64_r         <= 1'b1;
                      wr64_data_r    <= mem_wr64_data;
                      state <= S_WR_ISSUE;
                  end else if (mem_wr_req) begin
                      // FIX221_WR_ADDR_DEEP_PIPE_75:
                      // Capture local write address first; compute AXI address/cache index in S_WR_ADDR.
                      wr_local_addr_r <= mem_wr_addr;
                      wr_lane_r       <= mem_wr_addr[2:0];
                      wr_data_r       <= mem_wr_data;
                      wr64_r          <= 1'b0; // FIX187E_DET_RECORD_WR64_COMPLETE
                      state <= S_WR_ADDR;
                  end else if (mem_rd64_req) begin // FIX164_MAC16_RDADDR_PIPE_BURST2
                    rd_local_addr_r <= mem_rd64_addr;
                    rd32_r          <= 1'b0;
                    rd64_r          <= 1'b1;
                    state <= S_RD_ADDR;
                end else if (mem_rd32_req) begin // FIX164_MAC16_RDADDR_PIPE_BURST2
                    rd_local_addr_r <= mem_rd32_addr;
                    rd32_r          <= 1'b1;
                    rd64_r          <= 1'b0;
                    state <= S_RD_ADDR;
                end else if (mem_rd_req) begin
                    rd_local_addr_r <= mem_rd_addr;
                    rd32_r          <= 1'b0;
                    rd64_r          <= 1'b0;
                    state <= S_RD_ADDR;
                end
            end

            S_RD_ADDR: begin // FIX164_MAC16_RDADDR_PIPE_BURST2
                // Registered local address from S_IDLE is converted to aligned AXI/cache address here.
                // This breaks the long timing path from YOLO op/config/address math into rd_axi_addr_r.
                rd_axi_addr_r  <= rd_pipe_axi_addr;
                rd_cache_idx_r <= rd_pipe_axi_addr[6:3];
                rd_lane        <= rd_pipe_byte_addr[2:0];
                state <= S_RD_LOOKUP;
            end

            S_RD_LOOKUP: begin
                // Cache hit: keep original protocol shape: ready pulse first, valid next cycle.
                if (cache_valid[rd_cache_idx_r] && (cache_addr[rd_cache_idx_r] == rd_axi_addr_r)) begin
                    cache_hit_data_r <= cache_data[rd_cache_idx_r];
                    if (rd64_r) mem_rd64_ready <= 1'b1; else if (rd32_r) mem_rd32_ready <= 1'b1; else mem_rd_ready <= 1'b1; // FIX164_MAC8_VECTOR_PACK64
                    state <= S_CACHE_RESP;
                end else begin
                    M_AXI_ARADDR  <= rd_axi_addr_r;
                    // FIX164_MAC16_RD8_BURST2_PREFETCH:
                    //   rd64 uses burst2 as before.
                    //   normal byte reads also use burst2: beat0 returns requested byte, beat1 is cache-only prefetch.
                    //   rd32 remains single beat to avoid changing its protocol.
                    arlen_r       <= (rd64_r || (!rd32_r && !rd64_r)) ? 8'd1 : 8'd0; // FIX164_MAC16_RD8_BURST2_PREFETCH
                    rd_burst_len  <= (rd64_r || (!rd32_r && !rd64_r)) ? 4'd1 : 4'd0; // FIX164_MAC16_RD8_BURST2_PREFETCH
                    rd_burst_cnt  <= 4'd0;                 // FIX164_MAC16_RDADDR_PIPE_BURST2
                    M_AXI_ARVALID <= 1'b1;
                    state <= S_AR;
                end
            end

            S_CACHE_RESP: begin
                if (rd64_r) begin // FIX164_MAC8_VECTOR_PACK64
                    mem_rd64_valid <= 1'b1;
                    mem_rd64_data  <= cache_hit_data_r;
                    rd_count <= rd_count + 32'd8;
                end else if (rd32_r) begin // FIX164_MAC4_WEIGHT_PACK32
                    mem_rd32_valid <= 1'b1;
                    mem_rd32_data  <= pick_word32(cache_hit_data_r, rd_lane);
                    rd_count <= rd_count + 32'd4;
                end else begin
                    mem_rd_valid <= 1'b1;
                    mem_rd_data  <= pick_byte(cache_hit_data_r, rd_lane);
                    rd_count <= rd_count + 1'b1;
                end
                state <= S_IDLE;
            end

            S_AR: begin
                if (M_AXI_ARVALID && M_AXI_ARREADY) begin
                    M_AXI_ARVALID <= 1'b0;
                    M_AXI_RREADY  <= 1'b1;
                    if (rd64_r) mem_rd64_ready <= 1'b1; else if (rd32_r) mem_rd32_ready <= 1'b1; else mem_rd_ready <= 1'b1; // FIX164_MAC8_VECTOR_PACK64
                    state <= S_R;
                end
            end

            S_R: begin
                if (M_AXI_RVALID && M_AXI_RREADY) begin
                    if (rd64_r) begin // FIX164_MAC16_RDADDR_PIPE_BURST2
                        // Beat0 is returned to the core. Beat1, fetched by ARLEN=1, is cached only.
                        if (rd_burst_cnt == 4'd0) begin
                            mem_rd64_valid <= 1'b1;
                            mem_rd64_data  <= M_AXI_RDATA;
                            rd_count <= rd_count + 32'd8;
                        end
                    end else if (rd32_r) begin // FIX164_MAC4_WEIGHT_PACK32
                        mem_rd32_valid <= 1'b1;
                        mem_rd32_data  <= pick_word32(M_AXI_RDATA, rd_lane);
                        rd_count <= rd_count + 32'd4;
                    end else begin
                        // FIX164_MAC16_RD8_BURST2_PREFETCH: if this is a byte-read burst,
                        // only beat0 is the requested data. Later beats are cache-only prefetch.
                        if (rd_burst_cnt == 4'd0) begin
                            mem_rd_valid <= 1'b1;
                            mem_rd_data  <= pick_byte(M_AXI_RDATA, rd_lane);
                            rd_count <= rd_count + 1'b1;
                        end
                    end

                    if (M_AXI_RRESP != 2'b00) begin
                        error <= 1'b1;
                    end else begin
                        if (rd64_r || (!rd32_r && (rd_burst_len != 4'd0))) begin // FIX164_MAC16_RD8_BURST2_PREFETCH
                            cache_valid[rd_burst_cache_idx] <= 1'b1;
                            cache_addr[rd_burst_cache_idx] <= rd_burst_axi_addr;
                            cache_data[rd_burst_cache_idx] <= M_AXI_RDATA;
                        end else begin
                            cache_valid[rd_cache_idx_r] <= 1'b1;
                            cache_addr[rd_cache_idx_r] <= rd_axi_addr_r;
                            cache_data[rd_cache_idx_r] <= M_AXI_RDATA;
                        end
                    end

                    if (M_AXI_RLAST || (rd_burst_cnt == rd_burst_len)) begin
                        M_AXI_RREADY <= 1'b0;
                        rd_burst_cnt <= 4'd0;
                        arlen_r <= 8'd0;
                        state <= S_IDLE;
                    end else begin
                        rd_burst_cnt <= rd_burst_cnt + 4'd1;
                        M_AXI_RREADY <= 1'b1;
                        state <= S_R;
                    end
                end
            end

            S_WR_ADDR: begin // FIX221_WR_ADDR_DEEP_PIPE_75
                // Convert registered local write address to aligned AXI/cache address.
                // This cuts op_id/cfg/address logic -> wr_axi_addr_r critical path.
                wr_axi_addr_r  <= wr_pipe_axi_addr;
                wr_cache_idx_r <= wr_pipe_axi_addr[6:3];
                wr_lane_r      <= wr_pipe_byte_addr[2:0];
                state <= S_WR_ISSUE;
            end

            S_WR_ISSUE: begin
                // Invalidate matching read-cache line before committing the byte write.
                if (cache_valid[wr_cache_idx_r] && (cache_addr[wr_cache_idx_r] == wr_axi_addr_r))
                    cache_valid[wr_cache_idx_r] <= 1'b0;

                M_AXI_AWADDR  <= wr_axi_addr_r;
                M_AXI_WDATA   <= wr64_r ? wr64_data_r : put_byte_data(wr_data_r, wr_lane_r); // FIX187E_DET_RECORD_WR64_COMPLETE
                M_AXI_WSTRB   <= wr64_r ? {(AXI_DATA_WIDTH/8){1'b1}} : put_byte_strb(wr_lane_r); // FIX187E_DET_RECORD_WR64_COMPLETE
                M_AXI_WLAST   <= 1'b1;
                M_AXI_AWVALID <= 1'b1;
                M_AXI_WVALID  <= 1'b1;
                // Do not accept a response until the FSM can process it.
                M_AXI_BREADY  <= 1'b0;
                state <= S_AW_W;
            end

            S_AW_W: begin
                if (M_AXI_AWVALID && M_AXI_AWREADY) begin
                    M_AXI_AWVALID <= 1'b0;
                    aw_seen <= 1'b1;
                end
                if (M_AXI_WVALID && M_AXI_WREADY) begin
                    M_AXI_WVALID <= 1'b0;
                    M_AXI_WLAST  <= 1'b0;
                    w_seen <= 1'b1;
                end
                if ((aw_seen || (M_AXI_AWVALID && M_AXI_AWREADY)) &&
                    (w_seen  || (M_AXI_WVALID  && M_AXI_WREADY))) begin
                    M_AXI_BREADY <= 1'b1;
                    state <= S_B;
                end
            end

            S_B: begin
                if (M_AXI_BVALID && M_AXI_BREADY) begin
                    M_AXI_BREADY <= 1'b0;
                    if (wr64_r) mem_wr64_ready <= 1'b1; else mem_wr_ready <= 1'b1; // FIX187E_DET_RECORD_WR64_COMPLETE
                    wr_count <= wr_count + 1'b1;
                    if (M_AXI_BRESP != 2'b00)
                        error <= 1'b1;
                    state <= S_WR_ACK;
                end
            end
            S_WR_ACK: begin
                    state <= S_IDLE;
                end

            default: begin
                state <= S_IDLE;
            end
        endcase
    end
end

endmodule
