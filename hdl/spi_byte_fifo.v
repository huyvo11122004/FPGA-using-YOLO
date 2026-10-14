// spi_byte_fifo.v
// Small synchronous FIFO for SPI byte stream.

`timescale 1ns/1ps

module spi_byte_fifo #(
    parameter integer ADDR_BITS = 6  // depth = 64 bytes
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       clear,

    input  wire       wr_en,
    input  wire [7:0] wr_data,

    input  wire       rd_en,
    output wire [7:0] rd_data,

    output wire       empty,
    output wire       full,
    output reg        overflow,
    output reg [ADDR_BITS:0] level
);

    localparam integer DEPTH = (1 << ADDR_BITS);

    reg [7:0] mem [0:DEPTH-1];
    reg [ADDR_BITS-1:0] wr_ptr;
    reg [ADDR_BITS-1:0] rd_ptr;

    assign empty = (level == {ADDR_BITS+1{1'b0}});
    assign full  = (level == DEPTH[ADDR_BITS:0]);
    assign rd_data = mem[rd_ptr];

    wire do_wr = wr_en && !full;
    wire do_rd = rd_en && !empty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr   <= {ADDR_BITS{1'b0}};
            rd_ptr   <= {ADDR_BITS{1'b0}};
            level    <= {ADDR_BITS+1{1'b0}};
            overflow <= 1'b0;
        end else begin
            if (clear) begin
                wr_ptr   <= {ADDR_BITS{1'b0}};
                rd_ptr   <= {ADDR_BITS{1'b0}};
                level    <= {ADDR_BITS+1{1'b0}};
                overflow <= 1'b0;
            end else begin
                if (wr_en && full && !do_rd)
                    overflow <= 1'b1;

                if (do_wr) begin
                mem[wr_ptr] <= wr_data;
                wr_ptr <= wr_ptr + {{(ADDR_BITS-1){1'b0}}, 1'b1};
            end

            if (do_rd) begin
                rd_ptr <= rd_ptr + {{(ADDR_BITS-1){1'b0}}, 1'b1};
            end

                case ({do_wr, do_rd})
                    2'b10: level <= level + {{ADDR_BITS{1'b0}}, 1'b1};
                    2'b01: level <= level - {{ADDR_BITS{1'b0}}, 1'b1};
                    default: level <= level;
                endcase
            end
        end
    end

endmodule
