module memory_access(
    input MemWrite,
    input MemRead,
    input MemtoReg,
    input [63:0] address,
    output reg invMemAddr
);

    always @(*) begin
        // Default values
        invMemAddr = 0;

        // Check for valid memory address when MemRead or MemWrite is active
        if ((MemRead || MemWrite) && (((address / 8) > 1023) || (address[1:0] != 0))) begin
            invMemAddr = 1;
        end
    end

endmodule

module MEM_WB_Reg (
    input wire clk,
    input wire rst,
    input wire [63:0] alu_result_in,
    input wire [63:0] read_data_in,
    input wire [4:0] write_reg_in,
    input wire memtoreg_in,
    input wire regwrite_in,

    output reg [63:0] alu_result_out,
    output reg [63:0] read_data_out,
    output reg [4:0] write_reg_out,
    output reg memtoreg_out,
    output reg regwrite_out
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        alu_result_out <= 32'b0;
        read_data_out  <= 32'b0;
        write_reg_out  <= 5'b0;
        memtoreg_out   <= 1'b0;
        regwrite_out   <= 1'b0;
    end else begin
        alu_result_out <= alu_result_in;
        read_data_out  <= read_data_in;
        write_reg_out  <= write_reg_in;
        memtoreg_out   <= memtoreg_in;
        regwrite_out   <= regwrite_in;
    end
end

endmodule