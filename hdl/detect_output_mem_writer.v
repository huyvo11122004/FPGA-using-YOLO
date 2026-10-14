// FIX187E_DET_RECORD_WR64_COMPLETE:
// DET raw writer stores each 16-byte record using two aligned 64-bit writes.
// Legacy byte-write outputs are kept for compatibility and driven idle.

`timescale 1ns/1ps

module detect_output_mem_writer #(
    parameter integer ADDR_W = 32,
    parameter integer DATA_W = 8,
    parameter integer ACC_W  = 32,
    parameter [ADDR_W-1:0] DEFAULT_DET_BASE = 32'h01c0_0000,
    parameter [31:0] MAX_DET_RECORDS = 32'd262144
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    clear,

    input  wire [ADDR_W-1:0]       det_base,

    input  wire                    det_valid,
    input  wire [1:0]              det_branch,
    input  wire [31:0]             det_index,
    input  wire signed [ACC_W-1:0] det_data,

    output wire                    mem_wr_req,
    output wire [ADDR_W-1:0]       mem_wr_addr,
    output wire [DATA_W-1:0]       mem_wr_data,
    input  wire                    mem_wr_ready,

    // FIX187E_DET_RECORD_WR64_COMPLETE: 64-bit write sideband.
    output wire                    mem_wr64_req,
    output wire [ADDR_W-1:0]       mem_wr64_addr,
    output wire [63:0]             mem_wr64_data,
    input  wire                    mem_wr64_ready,

    output reg  [31:0]             det_count,
    output reg                     overflow,
    output reg                     busy,
    output wire                    ready
);

localparam [3:0]
    S_IDLE   = 4'd0,
    S_WRITE0 = 4'd1,
    S_WRITE1 = 4'd2;

reg [3:0] state;
reg [127:0] record;
reg [ADDR_W-1:0] record_base;
reg pending;

// FIX177_DET_PIXEL_MAJOR_REUSE:
// Store each record at canonical branch/index slot so det_raw.bin remains compatible.
wire [31:0] det_branch_base = (det_branch == 2'd0) ? 32'd0 :
                              (det_branch == 2'd1) ? 32'd33792 :
                              (det_branch == 2'd2) ? 32'd42240 : 32'd0;
wire [31:0] det_record_slot = det_branch_base + det_index;

wire signed [63:0] det_data_64 = {{(64-ACC_W){det_data[ACC_W-1]}}, det_data};

// Legacy byte interface is unused by DET after FIX187E.
assign mem_wr_req  = 1'b0;
assign mem_wr_addr = {ADDR_W{1'b0}};
assign mem_wr_data = {DATA_W{1'b0}};

// One 16-byte record -> two 64-bit writes. Record base is 16-byte aligned.
assign mem_wr64_req  = (state == S_WRITE0) || (state == S_WRITE1);
assign mem_wr64_addr = record_base + ((state == S_WRITE1) ? {{(ADDR_W-4){1'b0}}, 4'd8} : {ADDR_W{1'b0}});
assign mem_wr64_data = (state == S_WRITE0) ? record[63:0] : record[127:64];

// FIX_DET_READY: combinational accept signal for producer-side back-pressure.
assign ready = (state == S_IDLE) && !pending && !clear && (det_count < MAX_DET_RECORDS);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= S_IDLE;
        record <= 128'd0;
        record_base <= {ADDR_W{1'b0}};
        det_count <= 32'd0;
        overflow <= 1'b0;
        busy <= 1'b0;
        pending <= 1'b0;
    end else begin
        busy <= (state != S_IDLE);

        if (clear) begin
            state <= S_IDLE;
            record <= 128'd0;
            record_base <= {ADDR_W{1'b0}};
            det_count <= 32'd0;
            overflow <= 1'b0;
            pending <= 1'b0;
        end else begin
            if (det_valid) begin
                if (det_count >= MAX_DET_RECORDS) begin
                    overflow <= 1'b1;
                end else if (state != S_IDLE || pending) begin
                    overflow <= 1'b1;
                end else begin
                    record[7:0]     <= {6'd0, det_branch};
                    record[15:8]    <= det_index[7:0];
                    record[23:16]   <= det_index[15:8];
                    record[31:24]   <= det_index[23:16];
                    record[39:32]   <= det_index[31:24];
                    record[47:40]   <= det_data_64[7:0];
                    record[55:48]   <= det_data_64[15:8];
                    record[63:56]   <= det_data_64[23:16];
                    record[71:64]   <= det_data_64[31:24];
                    record[79:72]   <= det_data_64[39:32];
                    record[87:80]   <= det_data_64[47:40];
                    record[95:88]   <= det_data_64[55:48];
                    record[103:96]  <= det_data_64[63:56];
                    record[127:104] <= 24'd0;
                    record_base <= det_base + {det_record_slot[27:0], 4'b0000};
                    pending <= 1'b1;
                end
            end

            case (state)
                S_IDLE: begin
                    if (pending) begin
                        pending <= 1'b0;
                        state <= S_WRITE0;
                    end
                end
                S_WRITE0: begin
                    if (mem_wr64_ready) state <= S_WRITE1;
                end
                S_WRITE1: begin
                    if (mem_wr64_ready) begin
                        det_count <= det_count + 1'b1;
                        state <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
end

endmodule
