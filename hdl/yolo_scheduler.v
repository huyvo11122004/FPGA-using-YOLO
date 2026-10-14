// yolo_scheduler.v
// Simple sequential operation scheduler for YOLOv5n DDR full-flow.
// It only controls op order. The top module selects the corresponding core.

module yolo_scheduler #(
    parameter OP_W = 6
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              start,
    input  wire              op_done,

    output reg               busy,
    output reg               done,
    output reg  [OP_W-1:0]   op_id,
    output reg               op_start
);

    localparam [OP_W-1:0] OP_L0      = 6'd0;
    localparam [OP_W-1:0] OP_L1      = 6'd1;
    localparam [OP_W-1:0] OP_STAGE0  = 6'd2;   // layer2
    localparam [OP_W-1:0] OP_L3      = 6'd3;
    localparam [OP_W-1:0] OP_STAGE1  = 6'd4;   // layer4
    localparam [OP_W-1:0] OP_L5      = 6'd5;
    localparam [OP_W-1:0] OP_STAGE2  = 6'd6;   // layer6
    localparam [OP_W-1:0] OP_L7      = 6'd7;
    localparam [OP_W-1:0] OP_STAGE3  = 6'd8;   // layer8
    localparam [OP_W-1:0] OP_SPPF    = 6'd9;   // layer9
    localparam [OP_W-1:0] OP_L10     = 6'd10;
    localparam [OP_W-1:0] OP_L11     = 6'd11;
    localparam [OP_W-1:0] OP_L12     = 6'd12;
    localparam [OP_W-1:0] OP_L13     = 6'd13;
    localparam [OP_W-1:0] OP_L14     = 6'd14;
    localparam [OP_W-1:0] OP_L15     = 6'd15;
    localparam [OP_W-1:0] OP_L16     = 6'd16;
    localparam [OP_W-1:0] OP_L17     = 6'd17;
    localparam [OP_W-1:0] OP_L18     = 6'd18;
    localparam [OP_W-1:0] OP_L19     = 6'd19;
    localparam [OP_W-1:0] OP_L20     = 6'd20;
    localparam [OP_W-1:0] OP_L21     = 6'd21;
    localparam [OP_W-1:0] OP_L22     = 6'd22;
    localparam [OP_W-1:0] OP_L23     = 6'd23;
    localparam [OP_W-1:0] OP_DETECT  = 6'd24;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy     <= 1'b0;
            done     <= 1'b0;
            op_id    <= OP_L0;
            op_start <= 1'b0;
        end else begin
            done     <= 1'b0;
            op_start <= 1'b0;

            if (!busy) begin
                if (start) begin
                    busy     <= 1'b1;
                    op_id    <= OP_L0;
                    op_start <= 1'b1;
                end
            end else begin
                if (op_done) begin
                    if (op_id == OP_DETECT) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                    end else begin
                        op_id    <= op_id + 1'b1;
                        op_start <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
