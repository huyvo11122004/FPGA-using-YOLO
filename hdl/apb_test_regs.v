module apb_test_regs (
    input  wire        PCLK,
    input  wire        PRESETN,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [7:0]  PADDR,
    input  wire [31:0] PWDATA,
    output reg  [31:0] PRDATA,
    output wire        PREADY,
    output wire        PSLVERR
);

    reg [31:0] reg_ctrl;      // 0x00
    reg [31:0] reg_status;    // 0x04 bit0=busy, bit1=done
    reg [31:0] reg_threshold; // 0x08
    reg [31:0] reg_out;       // 0x10

    reg [7:0] in0, in1, in2, in3;
    reg [7:0] out0, out1, out2, out3;

    localparam IDLE = 3'd0,
               BUSY0 = 3'd1,
               BUSY1 = 3'd2,
               BUSY2 = 3'd3,
               BUSY3 = 3'd4,
               DONE = 3'd5;

    reg [2:0] state;

    assign PREADY  = 1'b1;
    assign PSLVERR = 1'b0;

    always @(posedge PCLK or negedge PRESETN) begin
        if (!PRESETN) begin
            reg_ctrl      <= 32'h00000001;   // start luôn d? demo
            reg_status    <= 32'h00000000;
            reg_threshold <= 32'h00000014;   // threshold = 20
            reg_out       <= 32'h00000000;

            in0 <= 8'd10;
            in1 <= 8'd25;
            in2 <= 8'd18;
            in3 <= 8'd40;

            out0 <= 8'd0;
            out1 <= 8'd0;
            out2 <= 8'd0;
            out3 <= 8'd0;

            state <= IDLE;
        end else begin
            case (state)
                IDLE: begin
                    reg_status[0] <= 1'b0; // busy
                    reg_status[1] <= 1'b0; // done

                    if (reg_ctrl[0]) begin
                        state <= BUSY0;
                        reg_status[0] <= 1'b1;
                        reg_status[1] <= 1'b0;
                    end
                end

                BUSY0: begin
                    out0 <= (in0 > reg_threshold[7:0]) ? 8'hFF : 8'h00;
                    state <= BUSY1;
                end

                BUSY1: begin
                    out1 <= (in1 > reg_threshold[7:0]) ? 8'hFF : 8'h00;
                    state <= BUSY2;
                end

                BUSY2: begin
                    out2 <= (in2 > reg_threshold[7:0]) ? 8'hFF : 8'h00;
                    state <= BUSY3;
                end

                BUSY3: begin
                    out3 <= (in3 > reg_threshold[7:0]) ? 8'hFF : 8'h00;
                    reg_out <= {
                        (in3 > reg_threshold[7:0]) ? 8'hFF : 8'h00,
                        (in2 > reg_threshold[7:0]) ? 8'hFF : 8'h00,
                        (in1 > reg_threshold[7:0]) ? 8'hFF : 8'h00,
                        (in0 > reg_threshold[7:0]) ? 8'hFF : 8'h00
                    };
                    state <= DONE;
                end

                DONE: begin
                    reg_status[0] <= 1'b0;
                    reg_status[1] <= 1'b1;
                    state <= DONE;
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

    always @(*) begin
        case (PADDR[7:0])
            8'h00: PRDATA = reg_ctrl;
            8'h04: PRDATA = reg_status;
            8'h08: PRDATA = reg_threshold;
            8'h10: PRDATA = reg_out;
            default: PRDATA = 32'h00000000;
        endcase
    end

endmodule