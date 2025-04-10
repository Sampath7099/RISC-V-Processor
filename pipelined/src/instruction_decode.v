module instruction_decode(
    input [31:0] instruction, 
    output [4:0] rs1,
    output [4:0] rs2,
    output [4:0] write_addr,
    output [9:0] alu_control, // Updated to 10-bit concatenation
    output [1:0] ALUOp,
    output ALUSrc,
    output RegWrite,
    output MemRead,
    output MemtoReg,
    output MemWrite,
    output Branch,
    output invOp,
    output invFunc,
    output invRegAddr
);
   
    wire [6:0] opcode = instruction[6:0];
    
    assign rs1 = instruction[19:15];
    assign rs2 = (opcode == 7'b0110011 || opcode == 7'b1100011 || opcode == 7'b0100011) ? instruction[24:20] : 5'bx; // Return 'x' when rs2 is not used
    assign write_addr = instruction[11:7];
    assign alu_control = {instruction[31:25], instruction[14:12]};

    // Instantiate the Control Unit
    ControlUnit CU (
        .opcode(opcode),
        .RegWrite(RegWrite),
        .MemtoReg(MemtoReg),
        .ALUSrc(ALUSrc), // Might be used elsewhere
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .Branch(Branch),
        .ALUOp(ALUOp),
        .invOp(invOp)
    );

endmodule

module ControlUnit (
    input [6:0] opcode,
    output reg RegWrite,
    output reg ALUSrc,
    output reg MemRead,
    output reg MemtoReg,
    output reg MemWrite,
    output reg Branch,
    output reg [1:0] ALUOp,
    output reg invOp  // Flag for invalid opcode
);
    
    always @(*) begin
        // Default values
        RegWrite = 0;
        ALUSrc   = 0;
        MemRead  = 0;
        MemtoReg = 0;
        MemWrite = 0;
        Branch   = 0;
        ALUOp    = 2'b00;
        invOp    = 0;  // Assume opcode is valid

        case (opcode)
            7'b0110011: begin  // R-type
                RegWrite = 1;
                ALUOp    = 2'b10;
            end
            7'b0000011: begin  // Load (LW)
                RegWrite = 1;
                ALUSrc   = 1;
                MemRead  = 1;
                MemtoReg = 1;
            end
            7'b0100011: begin  // Store (SW)
                ALUSrc   = 1;
                MemWrite = 1;
            end
            7'b1100011: begin  // Branch (BEQ)
                Branch   = 1;
                ALUOp    = 2'b01;
            end
            default: begin  // Invalid opcode detected
                invOp = 1;
            end
        endcase
    end
endmodule


module ID_EX_Reg (
    input wire clk,
    input wire rst,
    input wire [63:0] pc_in,
    input wire [63:0] read_data1_in,
    input wire [63:0] read_data2_in,
    input wire [63:0] imm_val_in,
    input wire [4:0] write_reg_in,
    input wire [9:0] alu_control_in,
    input wire alusrc_in,
    input wire branch_in,
    input wire memwrite_in,
    input wire memread_in,
    input wire memtoreg_in,
    input wire regwrite_in,
    input wire [1:0] alu_op_in,
    input wire [4:0] register_rs1_in,
    input wire [4:0] register_rs2_in,
    input wire [31:0] instruction_in,

    output reg [63:0] pc_out,
    output reg [63:0] read_data1_out,
    output reg [63:0] read_data2_out,
    output reg [63:0] imm_val_out,
    output reg [4:0] write_reg_out,
    output reg [9:0] alu_control_out,
    output reg alusrc_out,
    output reg branch_out,
    output reg memwrite_out,
    output reg memread_out,
    output reg memtoreg_out,
    output reg regwrite_out,
    output reg [4:0] register_rs1_out,
    output reg [4:0] register_rs2_out,
    output reg [1:0] alu_op_out,
    output reg [31:0] instruction_out

);
always @(posedge clk or posedge rst) begin
    if (rst) begin
        pc_out         <= 64'b0;
        read_data1_out <= 32'b0;
        read_data2_out <= 32'b0;
        imm_val_out    <= 64'b0;
        write_reg_out  <= 5'b0;
        alu_control_out <= 10'b0;
        alusrc_out     <= 1'b0;
        branch_out     <= 1'b0;
        memwrite_out   <= 1'b0;
        memread_out    <= 1'b0;
        memtoreg_out   <= 1'b0;
        regwrite_out   <= 1'b0;
        register_rs1_out <= 5'b0;
        register_rs2_out <= 5'b0;
        alu_op_out <= 2'b0;
        instruction_out <= 32'b0;
    end else begin
        pc_out         <= pc_in;
        read_data1_out <= read_data1_in;
        read_data2_out <= read_data2_in;
        imm_val_out    <= imm_val_in;
        write_reg_out  <= write_reg_in;
        alu_control_out <= alu_control_in;
        alusrc_out     <= alusrc_in;
        branch_out     <= branch_in;
        memwrite_out   <= memwrite_in;
        memread_out    <= memread_in;
        memtoreg_out   <= memtoreg_in;
        regwrite_out   <= regwrite_in;
        register_rs1_out <= register_rs1_in;
        register_rs2_out <= register_rs2_in;
        alu_op_out <= alu_op_in;
        instruction_out <= instruction_in;
    end
end

endmodule

module Mux(
    input [63:0] input1,
    input [63:0] input2,
    input select,
    output [63:0] out
);
    assign out = select ? input2 : input1;
endmodule