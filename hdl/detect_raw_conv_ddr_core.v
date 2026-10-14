// detect_raw_conv_ddr_core.v
// DDR-weight version: weights/bias are fetched from DDR, not hard-coded ROM.

module detect_raw_conv_ddr_core #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32,

    parameter IN_CH  = 64,
    parameter DET_CH = 33,
    parameter H      = 4,
    parameter W      = 4,

    parameter W_BASE = 32'h0390_0000,
    parameter B_BASE = 32'h0390_0000
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,

    input  wire [ADDR_W-1:0]       input_base,

    output wire                    mem_rd_req,
    output wire [ADDR_W-1:0]       mem_rd_addr,
    input  wire                    mem_rd_ready,
    input  wire                    mem_rd_valid,
    input  wire [DATA_W-1:0]       mem_rd_data,

    output wire                    busy,
    output wire                    done,

    output wire                    raw_valid,
    output wire [31:0]             raw_index,
    output wire signed [ACC_W-1:0] raw_data
);

    localparam OUT_N = DET_CH * H * W;

    localparam S_IDLE      = 4'd0;
    localparam S_BIAS_REQ  = 4'd1;
    localparam S_BIAS_WAIT = 4'd2;
    localparam S_W_REQ     = 4'd3;
    localparam S_W_WAIT    = 4'd4;
    localparam S_ACT_REQ   = 4'd5;
    localparam S_ACT_WAIT  = 4'd6;
    localparam S_OUT       = 4'd7;
    localparam S_DONE      = 4'd8;

    reg [3:0] state;

    integer oc;
    integer oy;
    integer ox;
    integer ic;

    reg signed [ACC_W-1:0] acc;
    reg signed [7:0]       w_byte_r;
    reg [31:0]             bias_r;
    reg [1:0]              bias_byte_idx;

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    wire [31:0] in_offset;
    wire [31:0] w_index;
    wire [31:0] out_index;

    assign in_offset = ic * (H * W) + oy * W + ox;
    assign w_index   = oc * IN_CH + ic;
    assign out_index = oc * (H * W) + oy * W + ox;

    wire [31:0] bias_next;
    assign bias_next = bias_r | ({24'd0, mem_rd_data} << (bias_byte_idx * 8));

    assign mem_rd_req = (state == S_BIAS_REQ) || (state == S_W_REQ) || (state == S_ACT_REQ);
    assign mem_rd_addr = (state == S_BIAS_REQ) ? (B_BASE[ADDR_W-1:0] + (oc * 4) + bias_byte_idx) :
                         (state == S_W_REQ)    ? (W_BASE[ADDR_W-1:0] + w_index) :
                                                  (input_base + in_offset);

    wire signed [7:0] act_s;
    assign act_s = mem_rd_data;

    wire last_ic;
    assign last_ic = (ic == IN_CH-1);

    wire last_output;
    assign last_output = (oc == DET_CH-1) && (oy == H-1) && (ox == W-1);

    assign raw_valid = (state == S_OUT);
    assign raw_index = out_index;
    assign raw_data  = acc;

    task advance_output;
        begin
            if (ox == W-1) begin
                ox <= 0;
                if (oy == H-1) begin
                    oy <= 0;
                    if (oc == DET_CH-1) begin
                        oc <= 0;
                    end else begin
                        oc <= oc + 1;
                    end
                end else begin
                    oy <= oy + 1;
                end
            end else begin
                ox <= ox + 1;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            oc <= 0; oy <= 0; ox <= 0; ic <= 0;
            acc <= 0;
            w_byte_r <= 0;
            bias_r <= 0;
            bias_byte_idx <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        oc <= 0;
                        oy <= 0;
                        ox <= 0;
                        ic <= 0;
                        bias_r <= 32'd0;
                        bias_byte_idx <= 2'd0;
                        state <= S_BIAS_REQ;
                    end
                end

                S_BIAS_REQ: begin
                    if (mem_rd_ready)
                        state <= S_BIAS_WAIT;
                end

                S_BIAS_WAIT: begin
                    if (mem_rd_valid) begin
                        bias_r <= bias_next;
                        if (bias_byte_idx == 2'd3) begin
                            acc <= $signed(bias_next);
                            ic <= 0;
                            state <= S_W_REQ;
                        end else begin
                            bias_byte_idx <= bias_byte_idx + 1'b1;
                            state <= S_BIAS_REQ;
                        end
                    end
                end

                S_W_REQ: begin
                    if (mem_rd_ready)
                        state <= S_W_WAIT;
                end

                S_W_WAIT: begin
                    if (mem_rd_valid) begin
                        w_byte_r <= mem_rd_data;
                        state <= S_ACT_REQ;
                    end
                end

                S_ACT_REQ: begin
                    if (mem_rd_ready)
                        state <= S_ACT_WAIT;
                end

                S_ACT_WAIT: begin
                    if (mem_rd_valid) begin
                        acc <= acc + (act_s * w_byte_r);
                        if (last_ic) begin
                            state <= S_OUT;
                        end else begin
                            ic <= ic + 1;
                            state <= S_W_REQ;
                        end
                    end
                end

                S_OUT: begin
                    if (last_output) begin
                        state <= S_DONE;
                    end else begin
                        advance_output();
                        bias_r <= 32'd0;
                        bias_byte_idx <= 2'd0;
                        state <= S_BIAS_REQ;
                    end
                end

                S_DONE: begin
                    state <= S_IDLE;
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
