module ov7670_start_pulse #(
    parameter integer DELAY_CYCLES = 50000000,
    parameter integer PULSE_CYCLES = 1
) (
    input  wire clk,
    input  wire rst_n,
    output reg  start_pulse
);
    localparam integer DELAY_W = (DELAY_CYCLES <= 1) ? 1 : $clog2(DELAY_CYCLES);
    localparam integer PULSE_W = (PULSE_CYCLES <= 1) ? 1 : $clog2(PULSE_CYCLES + 1);

    reg [DELAY_W-1:0] delay_cnt;
    reg [PULSE_W-1:0] pulse_cnt;
    reg fired;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            delay_cnt   <= {DELAY_W{1'b0}};
            pulse_cnt   <= {PULSE_W{1'b0}};
            fired       <= 1'b0;
            start_pulse <= 1'b0;
        end else begin
            start_pulse <= 1'b0;

            if (!fired) begin
                if (delay_cnt == DELAY_CYCLES-1) begin
                    if (pulse_cnt < PULSE_CYCLES) begin
                        start_pulse <= 1'b1;
                        pulse_cnt   <= pulse_cnt + 1'b1;
                    end else begin
                        fired     <= 1'b1;
                        pulse_cnt <= {PULSE_W{1'b0}};
                    end
                end else begin
                    delay_cnt <= delay_cnt + 1'b1;
                end
            end
        end
    end
endmodule
