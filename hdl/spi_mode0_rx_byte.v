// spi_mode0_rx_byte.v
// FT232H SPI mode-0 slave byte receiver for MPFS fabric clock domain.
// - SPI mode 0: CPOL=0, CPHA=0, sample MOSI on SCK rising edge.
// - MSB first.
// - spi_sck/spi_mosi/spi_cs_n are async to clk and are synchronized here.
// Use with FT232H AD0=SCK, AD1=MOSI, AD3=CS_N.

`timescale 1ns/1ps

module spi_mode0_rx_byte (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       spi_sck,
    input  wire       spi_mosi,
    input  wire       spi_cs_n,

    output wire       cs_active,
    output reg        byte_valid,
    output reg [7:0]  byte_data,
    output reg [31:0] rx_byte_count
);

    reg [2:0] sck_sync;
    reg [2:0] cs_sync;
    reg [1:0] mosi_sync;

    reg [2:0] bit_cnt;
    reg [7:0] shreg;

    wire sck_rise;
    wire cs_inactive;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_sync  <= 3'b000;
            cs_sync   <= 3'b111;
            mosi_sync <= 2'b00;
        end else begin
            sck_sync  <= {sck_sync[1:0], spi_sck};
            cs_sync   <= {cs_sync[1:0], spi_cs_n};
            mosi_sync <= {mosi_sync[0], spi_mosi};
        end
    end

    assign sck_rise   = (sck_sync[2:1] == 2'b01);
    assign cs_active  = (cs_sync[2] == 1'b0);
    assign cs_inactive = !cs_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt       <= 3'd0;
            shreg         <= 8'd0;
            byte_valid    <= 1'b0;
            byte_data     <= 8'd0;
            rx_byte_count <= 32'd0;
        end else begin
            byte_valid <= 1'b0;

            if (cs_inactive) begin
                bit_cnt <= 3'd0;
                shreg   <= 8'd0;
            end else if (sck_rise) begin
                shreg <= {shreg[6:0], mosi_sync[1]};

                if (bit_cnt == 3'd7) begin
                    byte_data     <= {shreg[6:0], mosi_sync[1]};
                    byte_valid    <= 1'b1;
                    rx_byte_count <= rx_byte_count + 32'd1;
                    bit_cnt       <= 3'd0;
                end else begin
                    bit_cnt <= bit_cnt + 3'd1;
                end
            end
        end
    end

endmodule
