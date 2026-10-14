// maxpool5x5_ddr_core.v
// Shared DDR-style MaxPool 5x5 stride1 pad2 core.
// Used by real YOLOv5n SPPF.
// Input/output shape is same: CH x H x W.
// Values are treated as signed-int8 feature values.

module maxpool5x5_ddr_core #(
    parameter DATA_W = 8,
    parameter ADDR_W = 32,
    parameter ACC_W  = 32,

    parameter CH     = 128,
    parameter H      = 10,
    parameter W      = 10
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,

    input  wire [ADDR_W-1:0]       input_base,
    input  wire [ADDR_W-1:0]       output_base,

    output wire                    mem_rd_req,
    output wire [ADDR_W-1:0]       mem_rd_addr,
    input  wire                    mem_rd_ready,
    input  wire                    mem_rd_valid,
    input  wire [DATA_W-1:0]       mem_rd_data,

    output wire                    mem_wr_req,
    output wire [ADDR_W-1:0]       mem_wr_addr,
    output wire [DATA_W-1:0]       mem_wr_data,
    input  wire                    mem_wr_ready,

    output wire                    busy,
    output wire                    done,

    output wire                    out_valid,
    output wire signed [ACC_W-1:0] out_data,

    output wire                    final_wr_en,
    output wire [ADDR_W-1:0]       final_wr_addr,
    output wire [DATA_W-1:0]       final_wr_data
);

    localparam S_IDLE    = 4'd0;
    localparam S_PREP    = 4'd1;
    localparam S_SCAN    = 4'd2;
    localparam S_RD_REQ  = 4'd3;
    localparam S_WAIT_RD = 4'd4;
    localparam S_WRITE   = 4'd5;
    localparam S_DONE    = 4'd6;

    reg [3:0] state;

    integer ch;
    integer oy;
    integer ox;
    integer ky;
    integer kx;

    reg signed [DATA_W-1:0] max_val;

    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    wire signed [31:0] iy_calc;
    wire signed [31:0] ix_calc;

    assign iy_calc = oy + ky - 2;
    assign ix_calc = ox + kx - 2;

    wire in_bound;
    assign in_bound = (iy_calc >= 0) && (iy_calc < H) &&
                      (ix_calc >= 0) && (ix_calc < W);

    wire [31:0] in_offset;
    wire [31:0] out_offset;

    assign in_offset  = ch * (H * W) + iy_calc * W + ix_calc;
    assign out_offset = ch * (H * W) + oy * W + ox;

    assign mem_rd_req  = (state == S_RD_REQ);
    assign mem_rd_addr = input_base + in_offset;

    assign mem_wr_req  = (state == S_WRITE);
    assign mem_wr_addr = output_base + out_offset;
    assign mem_wr_data = max_val[DATA_W-1:0];

    assign final_wr_en   = mem_wr_req && mem_wr_ready;
    assign final_wr_addr = mem_wr_addr;
    assign final_wr_data = mem_wr_data;

    assign out_valid = final_wr_en;
    assign out_data  = {{(ACC_W-DATA_W){max_val[DATA_W-1]}}, max_val};

    wire last_k;
    assign last_k = (ky == 4) && (kx == 4);

    wire last_output;
    assign last_output = (ch == CH-1) && (oy == H-1) && (ox == W-1);

    task advance_k;
        begin
            if (kx == 4) begin
                kx <= 0;
                if (ky == 4) begin
                    ky <= 0;
                end else begin
                    ky <= ky + 1;
                end
            end else begin
                kx <= kx + 1;
            end
        end
    endtask

    task advance_output;
        begin
            if (ox == W-1) begin
                ox <= 0;
                if (oy == H-1) begin
                    oy <= 0;
                    if (ch == CH-1) begin
                        ch <= 0;
                    end else begin
                        ch <= ch + 1;
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

            ch <= 0;
            oy <= 0;
            ox <= 0;
            ky <= 0;
            kx <= 0;

            max_val <= -128;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        ch <= 0;
                        oy <= 0;
                        ox <= 0;
                        state <= S_PREP;
                    end
                end

                S_PREP: begin
                    ky <= 0;
                    kx <= 0;
                    max_val <= -128;
                    state <= S_SCAN;
                end

                S_SCAN: begin
                    if (in_bound) begin
                        state <= S_RD_REQ;
                    end else begin
                        if (last_k) begin
                            state <= S_WRITE;
                        end else begin
                            advance_k();
                            state <= S_SCAN;
                        end
                    end
                end

                S_RD_REQ: begin
                    if (mem_rd_ready)
                        state <= S_WAIT_RD;
                end

                S_WAIT_RD: begin
                    if (mem_rd_valid) begin
                        if ($signed(mem_rd_data) > max_val)
                            max_val <= $signed(mem_rd_data);

                        if (last_k) begin
                            state <= S_WRITE;
                        end else begin
                            advance_k();
                            state <= S_SCAN;
                        end
                    end
                end

                S_WRITE: begin
                    if (mem_wr_ready) begin
                        if (last_output) begin
                            state <= S_DONE;
                        end else begin
                            advance_output();
                            state <= S_PREP;
                        end
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
