module accel_core (
    // Chu?n giao ti?p APB Slave
    input  wire        PCLK,
    input  wire        PRESETN,
    input  wire [7:0]  PADDR,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [31:0] PWDATA,
    output reg  [31:0] PRDATA,
    output wire        PREADY,
    output wire        PSLVERR // Thêm chân này d? Libero gom Bus d? hon
);

    // M?c d?nh luôn s?n sàng và không có l?i bus
    assign PREADY = 1'b1;
    assign PSLVERR = 1'b0;

    // Các thanh ghi
    reg [31:0] reg_ctrl;   // 0x00
    reg [31:0] reg_status; // 0x04
    reg [31:0] reg_in_a;   // 0x08
    reg [31:0] reg_in_b;   // 0x0C
    reg [31:0] reg_out;    // 0x10

    // Máy tr?ng thái (FSM)
    localparam IDLE = 2'b00, BUSY = 2'b01, DONE = 2'b10;
    reg [1:0] state;

    always @(posedge PCLK or negedge PRESETN) begin
        if (!PRESETN) begin
            state <= IDLE;
            reg_status <= 32'h0;
            reg_out <= 32'h0;
        end else begin
            case (state)
                IDLE: begin
                    if (reg_ctrl[0]) begin // START = 1
                        state <= BUSY;
                        reg_status[0] <= 1'b1; // BUSY = 1
                        reg_status[1] <= 1'b0; // DONE = 0
                    end
                end
                BUSY: begin
                    reg_out <= reg_in_a + reg_in_b; 
                    state <= DONE;
                end
                DONE: begin
                    reg_status[0] <= 1'b0; // BUSY = 0
                    reg_status[1] <= 1'b1; // DONE = 1
                    if (!reg_ctrl[0]) begin // Reset START
                        state <= IDLE;
                    end
                end
            endcase
        end
    end

    // Ghi d? li?u t? CPU xu?ng FPGA
    always @(posedge PCLK or negedge PRESETN) begin
        if (!PRESETN) begin
            reg_ctrl <= 32'h0;
            reg_in_a <= 32'h0;
            reg_in_b <= 32'h0;
        end else if (PSEL && PENABLE && PWRITE) begin
            case (PADDR[7:0])
                8'h00: reg_ctrl <= PWDATA;
                8'h08: reg_in_a <= PWDATA;
                8'h0C: reg_in_b <= PWDATA;
            endcase
        end
    end

    // Ð?c d? li?u t? FPGA lên CPU
    always @(*) begin
        case (PADDR[7:0])
            8'h00: PRDATA = reg_ctrl;
            8'h04: PRDATA = reg_status;
            8'h08: PRDATA = reg_in_a;
            8'h0C: PRDATA = reg_in_b;
            8'h10: PRDATA = reg_out;
            default: PRDATA = 32'h00000000;
        endcase
    end

endmodule

