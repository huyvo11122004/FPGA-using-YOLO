`timescale 1ns / 1ps

// cam_sccb_init_ov7670.v
//
// OV7670 SCCB init cho ngu?n ?nh VGA 640x480.
// M?c tiêu s?a l?i:
//   Top ðang ð? SRC_W=640, SRC_H=480 nhýng SCCB c? có b?t scaling/DCW.
//   Khi camera không th?t s? phát ð? 640x480, sampler 64x48 s? ð?i các t?a ð?
//   x t?i 635, y t?i 475 và không l?y ð? frame.
//
// B?n này:
//   - COM7 = 0x00: YUV, VGA
//   - COM3 = 0x00: t?t scaling/DCW
//   - COM14 = 0x00: t?t DCW/PCLK scaling
//   - V?n gi? ACK timing an toàn hõn: sample ACK sau khi SIOC ð? high ?n ð?nh.
//   - Ch? 50 ms sau COM7 reset.
//   - IGNORE_ACK_ERROR m?c ð?nh = 1 ð? debug không b? k?t b?i ACK gi?.

module cam_sccb_init_ov7670 #(
    parameter integer CLK_HZ             = 50_000_000,
    parameter integer SCCB_HZ            = 100_000,
    parameter integer PWRUP_WAIT_MS      = 20,
    parameter integer RESET_WAIT_MS      = 15,
    parameter integer SOFT_RESET_WAIT_MS = 50,
    parameter integer INTERWRITE_WAIT_MS = 2,
    parameter integer ENABLE_COLOR_BAR   = 0,
    parameter integer IGNORE_ACK_ERROR   = 1,
    parameter integer MAX_RETRIES        = 3
) (
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    output reg  busy,
    output reg  done,
    output reg  error,
    output reg  sioc,
    output reg  cam_reset_n,
    output reg  cam_pwdn,
    inout  wire siod
);

    // SCCB/I2C open-drain: FPGA ch? kéo th?p ho?c nh? Z.
    reg sda_oe;
    assign siod = sda_oe ? 1'b0 : 1'bz;
    wire sda_in = siod;

    localparam integer TICKS_PER_QTR_RAW = (CLK_HZ / (SCCB_HZ * 4));
    localparam integer TICKS_PER_QTR     = (TICKS_PER_QTR_RAW < 2) ? 2 : TICKS_PER_QTR_RAW;

    localparam [7:0] DEV_ADDR_WR = 8'h42; // OV7670 write address

    // Gi? s? lý?ng register nh? và r?.
    // Có th? thêm register màu/sáng sau khi VGA stream ?n ð?nh.
    localparam integer REG_COUNT = 10;

    function [7:0] reg_addr_at(input [7:0] idx);
        begin
            case (idx)
                8'd0: reg_addr_at = 8'h12; // COM7 reset
                8'd1: reg_addr_at = 8'h11; // CLKRC
                8'd2: reg_addr_at = 8'h12; // COM7
                8'd3: reg_addr_at = 8'h0C; // COM3
                8'd4: reg_addr_at = 8'h3E; // COM14
                8'd5: reg_addr_at = 8'h13; // COM8
                8'd6: reg_addr_at = 8'h15; // COM10
                8'd7: reg_addr_at = 8'h40; // COM15
                8'd8: reg_addr_at = 8'h3A; // TSLB
                8'd9: reg_addr_at = 8'h1E; // MVFP
                default: reg_addr_at = 8'h00;
            endcase
        end
    endfunction

    function [7:0] reg_data_at(input [7:0] idx);
        begin
            case (idx)
                8'd0: reg_data_at = 8'h80; // COM7 software reset
                8'd1: reg_data_at = 8'h01; // CLKRC: prescaler. N?u PCLK quá ch?m/cao có th? ch?nh sau.

                // COM7:
                // 0x00 = VGA + YUV. N?u mu?n color bar, OR thêm 0x02.
                8'd2: reg_data_at = ENABLE_COLOR_BAR ? 8'h02 : 8'h00;

                // QUAN TR?NG CHO 640x480:
                // T?t scaling/DCW.
                8'd3: reg_data_at = 8'h00; // COM3: disable scaling/DCW
                8'd4: reg_data_at = 8'h00; // COM14: disable DCW/PCLK scaling

                // Auto exposure/white balance/gain.
                8'd5: reg_data_at = 8'hE7; // COM8

                // COM10: gi? nhý b?n c? c?a project.
                8'd6: reg_data_at = 8'h20;

                // COM15: full output range/RGB option theo project c?.
                8'd7: reg_data_at = 8'hD0;

                // TSLB/MVFP: gi? nhý project c? ð? không ð?o pha d? li?u quá nhi?u.
                8'd8: reg_data_at = 8'h04;
                8'd9: reg_data_at = 8'h27;

                default: reg_data_at = 8'h00;
            endcase
        end
    endfunction

    localparam [4:0]
        ST_IDLE       = 5'd0,
        ST_PWRUP      = 5'd1,
        ST_RESET      = 5'd2,
        ST_LOAD       = 5'd3,
        ST_START_A    = 5'd4,
        ST_START_B    = 5'd5,
        ST_BIT_SET    = 5'd6,
        ST_BIT_HIGH   = 5'd7,
        ST_BIT_LOW    = 5'd8,
        ST_ACK_REL    = 5'd9,
        ST_ACK_HIGH   = 5'd10,
        ST_ACK_SAMPLE = 5'd11,
        ST_ACK_LOW    = 5'd12,
        ST_STOP_A     = 5'd13,
        ST_STOP_B     = 5'd14,
        ST_STOP_C     = 5'd15,
        ST_WAIT       = 5'd16,
        ST_DONE       = 5'd17,
        ST_RETRY      = 5'd18;

    reg [4:0]  state;
    (* syn_preserve = 1 *) reg [31:0] delay_cnt;
    reg [15:0] tick_cnt;
    reg        tick;

    reg [7:0]  reg_idx;
    reg [1:0]  byte_idx;
    reg [2:0]  bit_idx;
    reg [7:0]  curr_byte;
    reg        ack_fail;
    reg [3:0]  retry_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt <= 16'd0;
            tick     <= 1'b0;
        end else if (tick_cnt == TICKS_PER_QTR - 1) begin
            tick_cnt <= 16'd0;
            tick     <= 1'b1;
        end else begin
            tick_cnt <= tick_cnt + 1'b1;
            tick     <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            busy          <= 1'b0;
            done          <= 1'b0;
            error         <= 1'b0;
            sioc          <= 1'b1;
            sda_oe        <= 1'b0;
            cam_reset_n   <= 1'b0;
            cam_pwdn      <= 1'b1;
            delay_cnt     <= 32'd0;
            reg_idx       <= 8'd0;
            byte_idx      <= 2'd0;
            bit_idx       <= 3'd0;
            curr_byte     <= 8'd0;
            ack_fail      <= 1'b0;
            retry_count   <= 4'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    busy   <= 1'b0;
                    sioc   <= 1'b1;
                    sda_oe <= 1'b0;
                    done   <= 1'b0;

                    if (start) begin
                        busy        <= 1'b1;
                        done        <= 1'b0;
                        error       <= 1'b0;
                        ack_fail    <= 1'b0;
                        retry_count <= 4'd0;

                        cam_pwdn    <= 1'b0;
                        cam_reset_n <= 1'b0;

                        reg_idx     <= 8'd0;
                        byte_idx    <= 2'd0;
                        bit_idx     <= 3'd7;
                        curr_byte   <= 8'd0;

                        delay_cnt   <= PWRUP_WAIT_MS * (CLK_HZ / 1000);
                        state       <= ST_PWRUP;
                    end
                end

                ST_PWRUP: begin
                    if (delay_cnt > 0)
                        delay_cnt <= delay_cnt - 1'b1;
                    else begin
                        cam_reset_n <= 1'b1;
                        delay_cnt   <= RESET_WAIT_MS * (CLK_HZ / 1000);
                        state       <= ST_RESET;
                    end
                end

                ST_RESET: begin
                    if (delay_cnt > 0)
                        delay_cnt <= delay_cnt - 1'b1;
                    else
                        state <= ST_LOAD;
                end

                ST_LOAD: begin
                    byte_idx  <= 2'd0;
                    bit_idx   <= 3'd7;
                    curr_byte <= DEV_ADDR_WR;
                    state     <= ST_START_A;
                end

                ST_START_A: begin
                    if (tick) begin
                        sda_oe <= 1'b1; // START: SDA falling while SIOC high
                        state  <= ST_START_B;
                    end
                end

                ST_START_B: begin
                    if (tick) begin
                        sioc  <= 1'b0;
                        state <= ST_BIT_SET;
                    end
                end

                ST_BIT_SET: begin
                    if (tick) begin
                        sda_oe <= ~curr_byte[bit_idx]; // 0 drive low, 1 release
                        state  <= ST_BIT_HIGH;
                    end
                end

                ST_BIT_HIGH: begin
                    if (tick) begin
                        sioc  <= 1'b1;
                        state <= ST_BIT_LOW;
                    end
                end

                ST_BIT_LOW: begin
                    if (tick) begin
                        sioc <= 1'b0;
                        if (bit_idx > 0) begin
                            bit_idx <= bit_idx - 1'b1;
                            state   <= ST_BIT_SET;
                        end else begin
                            state <= ST_ACK_REL;
                        end
                    end
                end

                ST_ACK_REL: begin
                    if (tick) begin
                        sda_oe <= 1'b0; // release SDA for ACK
                        state  <= ST_ACK_HIGH;
                    end
                end

                ST_ACK_HIGH: begin
                    if (tick) begin
                        sioc  <= 1'b1;
                        state <= ST_ACK_SAMPLE;
                    end
                end

                ST_ACK_SAMPLE: begin
                    if (tick) begin
                        if (sda_in != 1'b0)
                            ack_fail <= 1'b1;
                        state <= ST_ACK_LOW;
                    end
                end

                ST_ACK_LOW: begin
                    if (tick) begin
                        sioc <= 1'b0;

                        if (byte_idx == 2'd0) begin
                            curr_byte <= reg_addr_at(reg_idx);
                            bit_idx   <= 3'd7;
                            byte_idx  <= 2'd1;
                            state     <= ST_BIT_SET;
                        end else if (byte_idx == 2'd1) begin
                            curr_byte <= reg_data_at(reg_idx);
                            bit_idx   <= 3'd7;
                            byte_idx  <= 2'd2;
                            state     <= ST_BIT_SET;
                        end else begin
                            state <= ST_STOP_A;
                        end
                    end
                end

                ST_STOP_A: begin
                    if (tick) begin
                        sda_oe <= 1'b1;
                        state  <= ST_STOP_B;
                    end
                end

                ST_STOP_B: begin
                    if (tick) begin
                        sioc  <= 1'b1;
                        state <= ST_STOP_C;
                    end
                end

                ST_STOP_C: begin
                    if (tick) begin
                        sda_oe <= 1'b0;

                        if (reg_idx == REG_COUNT - 1) begin
                            state <= ST_DONE;
                        end else begin
                            if (reg_idx == 8'd0)
                                delay_cnt <= SOFT_RESET_WAIT_MS * (CLK_HZ / 1000);
                            else
                                delay_cnt <= INTERWRITE_WAIT_MS * (CLK_HZ / 1000);

                            reg_idx <= reg_idx + 1'b1;
                            state   <= ST_WAIT;
                        end
                    end
                end

                ST_WAIT: begin
                    if (delay_cnt > 0)
                        delay_cnt <= delay_cnt - 1'b1;
                    else
                        state <= ST_LOAD;
                end

                ST_DONE: begin
                    busy <= 1'b0;

                    if (ack_fail && (IGNORE_ACK_ERROR == 0)) begin
                        if (retry_count < MAX_RETRIES[3:0]) begin
                            state <= ST_RETRY;
                        end else begin
                            done  <= 1'b0;
                            error <= 1'b1;
                            state <= ST_IDLE;
                        end
                    end else begin
                        done  <= 1'b1;
                        error <= 1'b0;
                        state <= ST_IDLE;
                    end
                end

                ST_RETRY: begin
                    retry_count <= retry_count + 1'b1;

                    busy        <= 1'b1;
                    done        <= 1'b0;
                    error       <= 1'b0;
                    ack_fail    <= 1'b0;

                    sioc        <= 1'b1;
                    sda_oe      <= 1'b0;
                    cam_pwdn    <= 1'b0;
                    cam_reset_n <= 1'b0;

                    reg_idx     <= 8'd0;
                    byte_idx    <= 2'd0;
                    bit_idx     <= 3'd7;
                    curr_byte   <= 8'd0;

                    delay_cnt   <= PWRUP_WAIT_MS * (CLK_HZ / 1000);
                    state       <= ST_PWRUP;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
