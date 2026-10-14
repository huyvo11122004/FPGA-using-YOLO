// spi_yolo_frontend.v
// Top SPI frontend: FT232H SPI pins -> byte RX -> FIFO -> YOLO frame loader.

`timescale 1ns/1ps

module spi_yolo_frontend #(
    parameter integer ADDR_W = 32,
    parameter [31:0]  INPUT_BASE = 32'h0000_0000,
    parameter integer FRAME_BYTES = 196608,
    parameter integer PLANE_BYTES = 65536,
    parameter integer FIFO_ADDR_BITS = 12
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              clear,

    // FIX_SPI_DROP_WHEN_BUSY:
    // At CS falling edge, if accept_enable=0 then drop this whole SPI transaction.
    // This prevents incoming video frames from overwriting DDR input while YOLO is detecting.
    input  wire              accept_enable,

    input  wire              spi_sck,
    input  wire              spi_mosi,
    input  wire              spi_cs_n,

    output wire              mem_wr_req,
    output wire [ADDR_W-1:0] mem_wr_addr,
    output wire [7:0]        mem_wr_data,
    input  wire              mem_wr_ready,

    output wire              spi_cs_active,
    output wire              frame_done_pulse,
    output wire              frame_loaded,
    output wire              busy,
    output wire              error,
    output wire              overrun_error,
    output wire [31:0]       last_frame_id,
    output wire [31:0]       payload_count,
    output wire [31:0]       rx_byte_count,
    output wire [31:0]       frames_loaded_count,
    output wire [7:0]        debug_state,

    // debug: first 8 raw SPI bytes captured after CS_N goes low
    output reg  [31:0]       debug_first4,
    output reg  [31:0]       debug_next4
);

    wire       rx_valid;
    wire [7:0] rx_data;
    wire       fifo_rd_en;
    wire [7:0] fifo_rd_data;
    wire       fifo_empty;
    wire       fifo_full;
    wire       fifo_overflow;
    wire [FIFO_ADDR_BITS:0] fifo_level;

    // FIX_SPI_DROP_WHEN_BUSY:
    // Latch accept/drop once per SPI transaction.
    reg        accept_transaction;
    reg        drop_transaction;
    wire       rx_valid_accept;

    assign rx_valid_accept = rx_valid && accept_transaction && !drop_transaction;

    spi_mode0_rx_byte u_spi_rx (
        .clk(clk),
        .rst_n(rst_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_cs_n(spi_cs_n),
        .cs_active(spi_cs_active),
        .byte_valid(rx_valid),
        .byte_data(rx_data),
        .rx_byte_count(rx_byte_count)
    );

    spi_byte_fifo #(
        .ADDR_BITS(FIFO_ADDR_BITS)
    ) u_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .clear(clear),
        .wr_en(rx_valid_accept),
        .wr_data(rx_data),
        .rd_en(fifo_rd_en),
        .rd_data(fifo_rd_data),
        .empty(fifo_empty),
        .full(fifo_full),
        .overflow(fifo_overflow),
        .level(fifo_level)
    );

    wire loader_overrun;

    spi_yolo_frame_loader #(
        .ADDR_W(ADDR_W),
        .INPUT_BASE(INPUT_BASE),
        .FRAME_BYTES(FRAME_BYTES),
        .PLANE_BYTES(PLANE_BYTES),
        .DEFAULT_HWC_U8_TO_CHW_I8(1'b0),
        .DEFAULT_BGR_INPUT(1'b0),
        .USE_HEADER_FLAGS(1'b0)
    ) u_loader (
        .clk(clk),
        .rst_n(rst_n),
        .clear(clear),
        .fifo_empty(fifo_empty),
        .fifo_rd_en(fifo_rd_en),
        .fifo_rd_data(fifo_rd_data),
        .mem_wr_req(mem_wr_req),
        .mem_wr_addr(mem_wr_addr),
        .mem_wr_data(mem_wr_data),
        .mem_wr_ready(mem_wr_ready),
        .busy(busy),
        .frame_done_pulse(frame_done_pulse),
        .frame_loaded(frame_loaded),
        .error(error),
        .len_error(),
        .overrun_error(loader_overrun),
        .last_frame_id(last_frame_id),
        .payload_count(payload_count),
        .frames_loaded_count(frames_loaded_count),
        .debug_state(debug_state)
    );

    assign overrun_error = fifo_overflow | loader_overrun;

    reg [3:0] debug_cap_count;
    reg       spi_cs_active_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debug_first4 <= 32'd0;
            debug_next4 <= 32'd0;
            debug_cap_count <= 4'd0;
            spi_cs_active_d <= 1'b0;
            accept_transaction <= 1'b0;
            drop_transaction <= 1'b0;
        end else begin
            spi_cs_active_d <= spi_cs_active;

            if (clear) begin
                debug_first4 <= 32'd0;
                debug_next4 <= 32'd0;
                debug_cap_count <= 4'd0;
                accept_transaction <= 1'b0;
                drop_transaction <= 1'b0;
            end else begin
                // Reset capture at falling edge of CS_N / start of packet.
                // Decide here whether this whole SPI transaction is accepted or dropped.
                if (spi_cs_active && !spi_cs_active_d) begin
                    debug_first4 <= 32'd0;
                    debug_next4 <= 32'd0;
                    debug_cap_count <= 4'd0;

                    if (accept_enable && !busy && !frame_loaded) begin
                        accept_transaction <= 1'b1;
                        drop_transaction <= 1'b0;
                    end else begin
                        accept_transaction <= 1'b0;
                        drop_transaction <= 1'b1;
                    end
                end else if (!spi_cs_active && spi_cs_active_d) begin
                    accept_transaction <= 1'b0;
                    drop_transaction <= 1'b0;
                end

                if (rx_valid_accept && (debug_cap_count < 4'd8)) begin
                    case (debug_cap_count)
                        4'd0: debug_first4[7:0]    <= rx_data;
                        4'd1: debug_first4[15:8]   <= rx_data;
                        4'd2: debug_first4[23:16]  <= rx_data;
                        4'd3: debug_first4[31:24]  <= rx_data;
                        4'd4: debug_next4[7:0]     <= rx_data;
                        4'd5: debug_next4[15:8]    <= rx_data;
                        4'd6: debug_next4[23:16]   <= rx_data;
                        4'd7: debug_next4[31:24]   <= rx_data;
                        default: begin end
                    endcase
                    debug_cap_count <= debug_cap_count + 4'd1;
                end
            end
        end
    end

endmodule
