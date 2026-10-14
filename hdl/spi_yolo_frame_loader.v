// spi_yolo_frame_loader.v
// RAW BIN mode, anti-stuck version:
//   - FPGA receives exactly FRAME_BYTES bytes from SPI FIFO.
//   - Byte 0 writes to INPUT_BASE + 0.
//   - Byte N writes to INPUT_BASE + N.
//   - No YSP1 header parsing.
//   - When FRAME_BYTES bytes are written, frame_loaded=1 and frames_loaded_count++.
//   - After frame is complete, extra bytes are DROPPED until APB clear.
//     This safely absorbs sender dummy-tail byte: payload + payload[:1].
//   - If a partial frame stalls too long, auto-timeout resets loader to IDLE.

`timescale 1ns/1ps

module spi_yolo_frame_loader #(
    parameter integer ADDR_W = 32,
    parameter [31:0]  INPUT_BASE = 32'h0000_0000,
    parameter integer IMG_W = 256,
    parameter integer IMG_H = 256,
    parameter integer IMG_C = 3,
    parameter integer FRAME_BYTES = 196608,
    parameter integer PLANE_BYTES = 65536,
    parameter         DEFAULT_HWC_U8_TO_CHW_I8 = 1'b0,
    parameter         DEFAULT_BGR_INPUT = 1'b0,
    parameter         USE_HEADER_FLAGS = 1'b0,

    // 25,000,000 cycles ~= 0.5s if PCLK=50MHz.
    // Must be much larger than sender inter-chunk gaps.
    parameter [31:0]  PAYLOAD_TIMEOUT_CYCLES = 32'd25000000
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              clear,

    input  wire              fifo_empty,
    output reg               fifo_rd_en,
    input  wire [7:0]        fifo_rd_data,

    output wire              mem_wr_req,
    output wire [ADDR_W-1:0] mem_wr_addr,
    output wire [7:0]        mem_wr_data,
    input  wire              mem_wr_ready,

    output reg               busy,
    output reg               frame_done_pulse,
    output reg               frame_loaded,
    output reg               error,
    output reg               len_error,
    output reg               overrun_error,
    output reg [31:0]        last_frame_id,
    output reg [31:0]        payload_count,
    output reg [31:0]        frames_loaded_count,
    output reg [7:0]         debug_state
);

    localparam [3:0]
        ST_IDLE    = 4'd0,
        ST_PAYLOAD = 4'd5,
        ST_HOLD    = 4'd6;  // frame complete; drop extra bytes until clear

    reg [3:0] state;

    reg        wr_pending;
    reg        wr_is_last;
    reg [ADDR_W-1:0] wr_addr_r;
    reg [7:0]        wr_data_r;
    reg [31:0]       idle_timeout_count;

    assign mem_wr_req  = wr_pending;
    assign mem_wr_addr = wr_addr_r;
    assign mem_wr_data = wr_data_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            fifo_rd_en <= 1'b0;

            wr_pending <= 1'b0;
            wr_is_last <= 1'b0;
            wr_addr_r <= {ADDR_W{1'b0}};
            wr_data_r <= 8'd0;

            busy <= 1'b0;
            frame_done_pulse <= 1'b0;
            frame_loaded <= 1'b0;
            error <= 1'b0;
            len_error <= 1'b0;
            overrun_error <= 1'b0;
            last_frame_id <= 32'd0;
            payload_count <= 32'd0;
            frames_loaded_count <= 32'd0;
            debug_state <= 8'd0;
            idle_timeout_count <= 32'd0;
        end else begin
            fifo_rd_en <= 1'b0;
            frame_done_pulse <= 1'b0;
            debug_state <= {4'd0, state};

            if (clear) begin
                state <= ST_IDLE;
                fifo_rd_en <= 1'b0;

                wr_pending <= 1'b0;
                wr_is_last <= 1'b0;
                wr_addr_r <= {ADDR_W{1'b0}};
                wr_data_r <= 8'd0;

                busy <= 1'b0;
                frame_done_pulse <= 1'b0;
                frame_loaded <= 1'b0;
                error <= 1'b0;
                len_error <= 1'b0;
                overrun_error <= 1'b0;
                last_frame_id <= 32'd0;
                payload_count <= 32'd0;
                frames_loaded_count <= 32'd0;
                idle_timeout_count <= 32'd0;
            end else begin
                // Complete one pending DDR byte write.
                if (wr_pending) begin
                    busy <= 1'b1;
                    idle_timeout_count <= 32'd0;

                    if (mem_wr_ready) begin
                        wr_pending <= 1'b0;

                        if (wr_is_last) begin
                            // Full frame accepted.
                            frame_loaded <= 1'b1;
                            frame_done_pulse <= 1'b1;
                            frames_loaded_count <= frames_loaded_count + 32'd1;

                            // Stay in HOLD so extra dummy-tail bytes are dropped,
                            // not treated as the start of a new partial frame.
                            state <= ST_HOLD;
                            busy <= 1'b0;
                            payload_count <= FRAME_BYTES[31:0];
                            idle_timeout_count <= 32'd0;
                        end
                    end
                end else begin
                    case (state)
                        ST_IDLE: begin
                            busy <= 1'b0;
                            idle_timeout_count <= 32'd0;

                            if (!frame_loaded && !fifo_empty) begin
                                // Start raw payload frame at DDR INPUT_BASE.
                                fifo_rd_en <= 1'b1;

                                wr_pending <= 1'b1;
                                wr_addr_r <= INPUT_BASE[ADDR_W-1:0];
                                wr_data_r <= fifo_rd_data;
                                wr_is_last <= (FRAME_BYTES[31:0] == 32'd1);

                                payload_count <= 32'd1;
                                last_frame_id <= last_frame_id + 32'd1;

                                state <= ST_PAYLOAD;
                                busy <= 1'b1;
                            end
                        end

                        ST_PAYLOAD: begin
                            busy <= 1'b1;

                            if (!fifo_empty) begin
                                fifo_rd_en <= 1'b1;

                                wr_pending <= 1'b1;
                                wr_addr_r <= INPUT_BASE[ADDR_W-1:0] + payload_count[ADDR_W-1:0];
                                wr_data_r <= fifo_rd_data;
                                wr_is_last <= (payload_count == (FRAME_BYTES[31:0] - 32'd1));

                                payload_count <= payload_count + 32'd1;
                                idle_timeout_count <= 32'd0;
                            end else begin
                                // No new FIFO byte while mid-frame.
                                // Recover automatically from broken/aborted transactions.
                                if (idle_timeout_count >= PAYLOAD_TIMEOUT_CYCLES) begin
                                    state <= ST_IDLE;
                                    busy <= 1'b0;
                                    wr_pending <= 1'b0;
                                    wr_is_last <= 1'b0;
                                    payload_count <= 32'd0;

                                    // Latch error for APB/debug until clear.
                                    error <= 1'b1;
                                    len_error <= 1'b1;
                                    idle_timeout_count <= 32'd0;
                                end else begin
                                    idle_timeout_count <= idle_timeout_count + 32'd1;
                                end
                            end
                        end

                        ST_HOLD: begin
                            // Frame already completed. Drop any extra bytes until software clear.
                            // This is intentional for sender dummy-tail byte.
                            busy <= 1'b0;
                            idle_timeout_count <= 32'd0;

                            if (!fifo_empty) begin
                                fifo_rd_en <= 1'b1;
                            end
                        end

                        default: begin
                            state <= ST_IDLE;
                            busy <= 1'b0;
                            payload_count <= 32'd0;
                            idle_timeout_count <= 32'd0;
                        end
                    endcase
                end
            end
        end
    end

endmodule
