// shared_detect_raw_ddr_engine.v
// FIX177_DET_PIXEL_MAJOR_REUSE:
// Runtime-configurable raw 1x1 detect conv.
// Keeps existing bias/weight cache, but changes DET compute order internally:
//   old: oc -> pixel -> ic, rereading the same input tensor for each DET channel
//   new: pixel -> ic -> oc, reading each input activation once per pixel and reusing it
//        across DET_CH cached weights.
// raw_index is still oc-major (oc*spatial+pixel). detect_output_mem_writer is patched
// to place records by branch/index so the DDR det_raw layout remains compatible.
module shared_detect_raw_ddr_engine #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32,
    parameter DET_CH = 33
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire [1:0] branch_id,
    input  wire [15:0] in_ch,
    input  wire [15:0] h,
    input  wire [15:0] w,
    input  wire [ADDR_W-1:0] input_base,
    input  wire [ADDR_W-1:0] weight_base,
    input  wire [ADDR_W-1:0] bias_base,
    input  wire [4:0] out_shift,
    input  wire [31:0] out_mult,
    input  wire [4:0] out_mult_shift,
    output wire mem_rd_req,
    output wire [ADDR_W-1:0] mem_rd_addr,
    input  wire mem_rd_ready,
    input  wire mem_rd_valid,
    input  wire [DATA_W-1:0] mem_rd_data,
    output wire busy,
    output wire done,
    input  wire raw_ready,
    output wire raw_valid,
    output wire [1:0] raw_branch,
    output wire [31:0] raw_index,
    output wire signed [ACC_W-1:0] raw_data,
    output wire [3:0] prof_state // FIX186_DET_PROFILER: DET FSM state for APB counters
);
    localparam integer MAX_DET_IN_CH  = 256;
    localparam integer MAX_DET_WEIGHT = DET_CH * MAX_DET_IN_CH;
    // Large enough gap so detect_output_mem_writer can write 16 bytes even with stalls.
    localparam [7:0] OUT_GAP_CYCLES = 8'd0; // FIX_DET_READY: S_OUT waits for writer ready; no fixed 255-cycle delay

    localparam S_IDLE           = 4'd0;
    localparam S_BIAS_LOAD_REQ  = 4'd1;
    localparam S_BIAS_LOAD_WAIT = 4'd2;
    localparam S_WCACHE_REQ     = 4'd3;
    localparam S_WCACHE_WAIT    = 4'd4;
    localparam S_PIXEL_INIT     = 4'd5;
    localparam S_ACT_REQ        = 4'd6;
    localparam S_ACT_WAIT       = 4'd7;
    localparam S_OC_MAC         = 4'd8;
    localparam S_OUT            = 4'd9;
    localparam S_OUT_GAP        = 4'd10;
    localparam S_DONE           = 4'd11;

    reg [3:0] state;
    reg [15:0] oy, ox, ic;
    reg [7:0]  oc;
    reg [7:0]  out_oc;
    reg [7:0]  out_gap;

    reg signed [7:0] act_r;
    reg signed [ACC_W-1:0] acc_vec [0:DET_CH-1];
    reg [31:0] bias_r;
    reg [1:0]  bias_byte_idx;
    reg [7:0]  bias_load_oc;
    reg [13:0] w_load_idx;
    integer init_i;

    reg signed [ACC_W-1:0] bias_cache   [0:DET_CH-1];
    reg signed [7:0]       weight_cache [0:MAX_DET_WEIGHT-1];

    function signed [ACC_W-1:0] det_requant_shift_round;
        input signed [ACC_W-1:0] din;
        input [4:0] sh;
        reg signed [ACC_W-1:0] mag;
        reg signed [ACC_W-1:0] rounded;
        begin
            if (sh == 5'd0) begin
                det_requant_shift_round = din;
            end else if (din >= 0) begin
                det_requant_shift_round = (din + ({{(ACC_W-1){1'b0}},1'b1} << (sh-1))) >>> sh;
            end else begin
                mag = -din;
                rounded = (mag + ({{(ACC_W-1){1'b0}},1'b1} << (sh-1))) >>> sh;
                det_requant_shift_round = -rounded;
            end
        end
    endfunction

    function signed [ACC_W-1:0] det_requant_mult_round;
        input signed [ACC_W-1:0] din;
        input [31:0] mult;
        input [4:0] sh;
        reg signed [63:0] prod;
        reg signed [63:0] mag;
        reg signed [63:0] rounded64;
        begin
            if (mult == 32'd0) begin
                det_requant_mult_round = din;
            end else begin
                prod = din * $signed({1'b0, mult});
                if (sh == 5'd0) begin
                    rounded64 = prod;
                end else if (prod >= 0) begin
                    rounded64 = (prod + (64'sd1 << (sh-1))) >>> sh;
                end else begin
                    mag = -prod;
                    rounded64 = -((mag + (64'sd1 << (sh-1))) >>> sh);
                end
                if (rounded64 > 64'sd2147483647)
                    det_requant_mult_round = 32'sh7fffffff;
                else if (rounded64 < -64'sd2147483648)
                    det_requant_mult_round = -32'sd2147483648;
                else
                    det_requant_mult_round = rounded64[ACC_W-1:0];
            end
        end
    endfunction

    wire [31:0] spatial    = {16'd0,h} * {16'd0,w};
    wire [31:0] pix_offset = oy * {16'd0,w} + ox;
    wire [31:0] in_offset  = {16'd0,ic} * spatial + pix_offset;
    wire [31:0] w_index    = {24'd0,oc} * {16'd0,in_ch} + {16'd0,ic};
    wire [31:0] out_index  = {24'd0,out_oc} * spatial + pix_offset;
    wire [31:0] bias_next  = bias_r | ({24'd0,mem_rd_data} << (bias_byte_idx * 8));
    wire signed [7:0] act_s = mem_rd_data;
    wire signed [7:0] cached_w_s = weight_cache[w_index[13:0]];
    wire last_ic = (ic == in_ch-1);
    wire last_pixel = (oy == h-1) && (ox == w-1);
    wire last_output = last_pixel && (out_oc == DET_CH-1);
    wire [31:0] total_weights = {16'd0,in_ch} * DET_CH;

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    assign mem_rd_req = (state == S_BIAS_LOAD_REQ) || (state == S_WCACHE_REQ) || (state == S_ACT_REQ);
    assign mem_rd_addr = (state == S_BIAS_LOAD_REQ) ? (bias_base + ({24'd0,bias_load_oc} * 4) + bias_byte_idx) :
                         (state == S_WCACHE_REQ)    ? (weight_base + {{(ADDR_W-14){1'b0}}, w_load_idx}) :
                                                       (input_base + in_offset[ADDR_W-1:0]);

    assign raw_valid  = (state == S_OUT) && raw_ready;
    assign raw_branch = branch_id;
    assign raw_index  = out_index;
    wire signed [ACC_W-1:0] out_acc = acc_vec[out_oc];
    wire signed [ACC_W-1:0] raw_requant = (out_mult != 32'd0) ? det_requant_mult_round(out_acc, out_mult, out_mult_shift) : det_requant_shift_round(out_acc, out_shift);
    assign raw_data = raw_requant;
    assign prof_state = state; // FIX186_DET_PROFILER

    task advance_pixel;
    begin
        if (ox == w-1) begin
            ox <= 16'd0;
            if (oy == h-1) begin
                oy <= 16'd0;
            end else begin
                oy <= oy + 16'd1;
            end
        end else begin
            ox <= ox + 16'd1;
        end
    end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            oy <= 0; ox <= 0; ic <= 0; oc <= 0; out_oc <= 0; out_gap <= 0;
            act_r <= 8'sd0;
            bias_r <= 0;
            bias_byte_idx <= 0;
            bias_load_oc <= 0;
            w_load_idx <= 0;
            for (init_i = 0; init_i < DET_CH; init_i = init_i + 1) begin
                acc_vec[init_i] <= {ACC_W{1'b0}};
            end
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        oy <= 0; ox <= 0; ic <= 0; oc <= 0; out_oc <= 0; out_gap <= 0;
                        bias_r <= 0;
                        bias_byte_idx <= 0;
                        bias_load_oc <= 0;
                        w_load_idx <= 0;
                        state <= S_BIAS_LOAD_REQ;
                    end
                end

                S_BIAS_LOAD_REQ: begin
                    if (mem_rd_ready) state <= S_BIAS_LOAD_WAIT;
                end

                S_BIAS_LOAD_WAIT: begin
                    if (mem_rd_valid) begin
                        bias_r <= bias_next;
                        if (bias_byte_idx == 2'd3) begin
                            bias_cache[bias_load_oc] <= $signed(bias_next);
                            bias_r <= 32'd0;
                            bias_byte_idx <= 2'd0;
                            if (bias_load_oc == (DET_CH-1)) begin
                                w_load_idx <= 14'd0;
                                state <= S_WCACHE_REQ;
                            end else begin
                                bias_load_oc <= bias_load_oc + 8'd1;
                                state <= S_BIAS_LOAD_REQ;
                            end
                        end else begin
                            bias_byte_idx <= bias_byte_idx + 2'd1;
                            state <= S_BIAS_LOAD_REQ;
                        end
                    end
                end

                S_WCACHE_REQ: begin
                    if (mem_rd_ready) state <= S_WCACHE_WAIT;
                end

                S_WCACHE_WAIT: begin
                    if (mem_rd_valid) begin
                        weight_cache[w_load_idx] <= $signed(mem_rd_data);
                        if ({18'd0,w_load_idx} + 32'd1 >= total_weights) begin
                            oy <= 0; ox <= 0; ic <= 0; oc <= 0; out_oc <= 0;
                            state <= S_PIXEL_INIT;
                        end else begin
                            w_load_idx <= w_load_idx + 14'd1;
                            state <= S_WCACHE_REQ;
                        end
                    end
                end

                // Initialize all DET output accumulators for current pixel.
                S_PIXEL_INIT: begin
                    for (init_i = 0; init_i < DET_CH; init_i = init_i + 1) begin
                        acc_vec[init_i] <= bias_cache[init_i];
                    end
                    ic <= 16'd0;
                    oc <= 8'd0;
                    out_oc <= 8'd0;
                    state <= S_ACT_REQ;
                end

                // Read each input activation once per pixel/ic.
                S_ACT_REQ: begin
                    if (mem_rd_ready) state <= S_ACT_WAIT;
                end

                S_ACT_WAIT: begin
                    if (mem_rd_valid) begin
                        act_r <= act_s;
                        oc <= 8'd0;
                        state <= S_OC_MAC;
                    end
                end

                // Reuse current activation across all DET_CH output weights.
                S_OC_MAC: begin
                    acc_vec[oc] <= acc_vec[oc] + (act_r * cached_w_s);
                    if (oc == (DET_CH-1)) begin
                        oc <= 8'd0;
                        if (last_ic) begin
                            out_oc <= 8'd0;
                            state <= S_OUT;
                        end else begin
                            ic <= ic + 16'd1;
                            state <= S_ACT_REQ;
                        end
                    end else begin
                        oc <= oc + 8'd1;
                    end
                end

                S_OUT: begin
                    // FIX_DET_READY: hold record stable until writer can accept it.
                    // raw_valid is asserted only when raw_ready=1, so no record is dropped.
                    if (raw_ready) begin
                        out_gap <= 8'd0;
                        state <= S_OUT_GAP;
                    end
                end

                S_OUT_GAP: begin
                    if (out_gap < OUT_GAP_CYCLES) begin
                        out_gap <= out_gap + 8'd1;
                    end else begin
                        if (out_oc == (DET_CH-1)) begin
                            if (last_pixel) begin
                                state <= S_DONE;
                            end else begin
                                advance_pixel();
                                state <= S_PIXEL_INIT;
                            end
                        end else begin
                            out_oc <= out_oc + 8'd1;
                            state <= S_OUT;
                        end
                    end
                end

                S_DONE: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
