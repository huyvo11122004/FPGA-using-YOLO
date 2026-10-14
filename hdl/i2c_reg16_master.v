`timescale 1ns/1ps

// Minimal open-drain I2C master for sensors that use a 16-bit register
// address and one byte of data.  It supports:
//   write: START, SLA+W, REG_H, REG_L, DATA, STOP
//   read : START, SLA+W, REG_H, REG_L, repeated START, SLA+R, DATA, NACK, STOP
// Clock stretching and multi-master arbitration are intentionally not used.
module i2c_reg16_master #(
    parameter integer CLK_HZ = 54_347_826,
    parameter integer I2C_HZ = 100_000,
    parameter [6:0]   DEVICE_ADDR = 7'h10
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        cmd_valid,
    output wire        cmd_ready,
    input  wire        cmd_read,
    input  wire [15:0] cmd_reg,
    input  wire [7:0]  cmd_wdata,

    output reg  [7:0]  cmd_rdata,
    output reg         busy,
    output reg         done,
    output reg         ack_error,

    inout  wire        scl,
    inout  wire        sda
);

    // SEND_SETUP, SEND_HIGH and SEND_LOW each last one tick.  Three ticks
    // therefore make approximately one I2C bit period.
    localparam integer RAW_DIV = CLK_HZ / (I2C_HZ * 3);
    localparam integer DIVIDER = (RAW_DIV < 2) ? 2 : RAW_DIV;
    localparam integer DIV_W   = (DIVIDER <= 2) ? 1 : $clog2(DIVIDER);

    reg [DIV_W-1:0] div_count;
    wire tick = (div_count == DIVIDER - 1);

    reg scl_low;
    reg sda_low;
    assign scl = scl_low ? 1'b0 : 1'bz;
    assign sda = sda_low ? 1'b0 : 1'bz;
    wire sda_in = sda;

    localparam [4:0]
        ST_IDLE          = 5'd0,
        ST_START_RELEASE = 5'd1,
        ST_START_SDA     = 5'd2,
        ST_START_SCL     = 5'd3,
        ST_SEND_SETUP    = 5'd4,
        ST_SEND_HIGH     = 5'd5,
        ST_SEND_LOW      = 5'd6,
        ST_ACK_SETUP     = 5'd7,
        ST_ACK_HIGH      = 5'd8,
        ST_ACK_LOW       = 5'd9,
        ST_RESTART_REL   = 5'd10,
        ST_RESTART_HIGH  = 5'd11,
        ST_RESTART_SDA   = 5'd12,
        ST_RESTART_LOW   = 5'd13,
        ST_READ_SETUP    = 5'd14,
        ST_READ_HIGH     = 5'd15,
        ST_READ_LOW      = 5'd16,
        ST_NACK_SETUP    = 5'd17,
        ST_NACK_HIGH     = 5'd18,
        ST_NACK_LOW      = 5'd19,
        ST_STOP_LOW      = 5'd20,
        ST_STOP_HIGH     = 5'd21,
        ST_STOP_RELEASE  = 5'd22,
        ST_FINISH        = 5'd23;

    reg [4:0] state;
    reg [2:0] bit_index;
    reg [2:0] stage;
    reg [7:0] tx_byte;
    reg [7:0] rx_byte;
    reg       read_latched;
    reg [15:0] reg_latched;
    reg [7:0]  wdata_latched;

    assign cmd_ready = (state == ST_IDLE) && !busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_count <= {DIV_W{1'b0}};
        end else if (tick || !busy) begin
            div_count <= {DIV_W{1'b0}};
        end else begin
            div_count <= div_count + 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            scl_low       <= 1'b0;
            sda_low       <= 1'b0;
            bit_index     <= 3'd7;
            stage         <= 3'd0;
            tx_byte       <= 8'd0;
            rx_byte       <= 8'd0;
            read_latched  <= 1'b0;
            reg_latched   <= 16'd0;
            wdata_latched <= 8'd0;
            cmd_rdata     <= 8'd0;
            busy          <= 1'b0;
            done          <= 1'b0;
            ack_error     <= 1'b0;
        end else begin
            done <= 1'b0;

            if (state == ST_IDLE) begin
                scl_low <= 1'b0;
                sda_low <= 1'b0;
                busy    <= 1'b0;

                if (cmd_valid) begin
                    read_latched  <= cmd_read;
                    reg_latched   <= cmd_reg;
                    wdata_latched <= cmd_wdata;
                    tx_byte       <= {DEVICE_ADDR, 1'b0};
                    bit_index     <= 3'd7;
                    stage         <= 3'd0;
                    ack_error     <= 1'b0;
                    busy          <= 1'b1;
                    state         <= ST_START_RELEASE;
                end
            end else if (tick) begin
                case (state)
                    ST_START_RELEASE: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        state   <= ST_START_SDA;
                    end

                    ST_START_SDA: begin
                        // START: SDA falls while SCL is high.
                        scl_low <= 1'b0;
                        sda_low <= 1'b1;
                        state   <= ST_START_SCL;
                    end

                    ST_START_SCL: begin
                        scl_low <= 1'b1;
                        state   <= ST_SEND_SETUP;
                    end

                    ST_SEND_SETUP: begin
                        scl_low <= 1'b1;
                        // Open drain: write one by releasing SDA.
                        sda_low <= ~tx_byte[bit_index];
                        state   <= ST_SEND_HIGH;
                    end

                    ST_SEND_HIGH: begin
                        scl_low <= 1'b0;
                        state   <= ST_SEND_LOW;
                    end

                    ST_SEND_LOW: begin
                        scl_low <= 1'b1;
                        if (bit_index == 3'd0) begin
                            state <= ST_ACK_SETUP;
                        end else begin
                            bit_index <= bit_index - 1'b1;
                            state     <= ST_SEND_SETUP;
                        end
                    end

                    ST_ACK_SETUP: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b0;
                        state   <= ST_ACK_HIGH;
                    end

                    ST_ACK_HIGH: begin
                        scl_low <= 1'b0;
                        if (sda_in)
                            ack_error <= 1'b1;
                        state <= ST_ACK_LOW;
                    end

                    ST_ACK_LOW: begin
                        scl_low   <= 1'b1;
                        bit_index <= 3'd7;

                        case (stage)
                            3'd0: begin
                                tx_byte <= reg_latched[15:8];
                                stage   <= 3'd1;
                                state   <= ST_SEND_SETUP;
                            end
                            3'd1: begin
                                tx_byte <= reg_latched[7:0];
                                stage   <= 3'd2;
                                state   <= ST_SEND_SETUP;
                            end
                            3'd2: begin
                                if (read_latched) begin
                                    stage <= 3'd4;
                                    state <= ST_RESTART_REL;
                                end else begin
                                    tx_byte <= wdata_latched;
                                    stage   <= 3'd3;
                                    state   <= ST_SEND_SETUP;
                                end
                            end
                            3'd3: begin
                                state <= ST_STOP_LOW;
                            end
                            3'd4: begin
                                // SLA+R was acknowledged.
                                rx_byte <= 8'd0;
                                state   <= ST_READ_SETUP;
                            end
                            default: state <= ST_STOP_LOW;
                        endcase
                    end

                    ST_RESTART_REL: begin
                        // Release SDA while SCL is still low.
                        scl_low <= 1'b1;
                        sda_low <= 1'b0;
                        state   <= ST_RESTART_HIGH;
                    end

                    ST_RESTART_HIGH: begin
                        scl_low <= 1'b0;
                        state   <= ST_RESTART_SDA;
                    end

                    ST_RESTART_SDA: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b1;
                        state   <= ST_RESTART_LOW;
                    end

                    ST_RESTART_LOW: begin
                        scl_low   <= 1'b1;
                        tx_byte   <= {DEVICE_ADDR, 1'b1};
                        bit_index <= 3'd7;
                        state     <= ST_SEND_SETUP;
                    end

                    ST_READ_SETUP: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b0;
                        state   <= ST_READ_HIGH;
                    end

                    ST_READ_HIGH: begin
                        scl_low <= 1'b0;
                        rx_byte[bit_index] <= sda_in;
                        state <= ST_READ_LOW;
                    end

                    ST_READ_LOW: begin
                        scl_low <= 1'b1;
                        if (bit_index == 3'd0) begin
                            state <= ST_NACK_SETUP;
                        end else begin
                            bit_index <= bit_index - 1'b1;
                            state     <= ST_READ_SETUP;
                        end
                    end

                    ST_NACK_SETUP: begin
                        // Single-byte read ends with NACK (SDA released).
                        scl_low <= 1'b1;
                        sda_low <= 1'b0;
                        state   <= ST_NACK_HIGH;
                    end

                    ST_NACK_HIGH: begin
                        scl_low <= 1'b0;
                        state   <= ST_NACK_LOW;
                    end

                    ST_NACK_LOW: begin
                        scl_low   <= 1'b1;
                        cmd_rdata <= rx_byte;
                        state     <= ST_STOP_LOW;
                    end

                    ST_STOP_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b1;
                        state   <= ST_STOP_HIGH;
                    end

                    ST_STOP_HIGH: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b1;
                        state   <= ST_STOP_RELEASE;
                    end

                    ST_STOP_RELEASE: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        state   <= ST_FINISH;
                    end

                    ST_FINISH: begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end

                    default: begin
                        scl_low   <= 1'b0;
                        sda_low   <= 1'b0;
                        busy      <= 1'b0;
                        ack_error <= 1'b1;
                        state     <= ST_IDLE;
                    end
                endcase
            end
        end
    end

endmodule
