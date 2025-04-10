module instruction_decode(
    input [31:0] instruction, 
    output [4:0] rs1,
    output [4:0] rs2,
    output [4:0] write_addr,
    output [3:0] alu_control_signal,
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
    wire [1:0] ALUOp;
    
    assign rs1 = instruction[19:15];
    assign rs2 = instruction[24:20];
    assign write_addr = instruction[11:7];

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
    
    // Instantiate the ALU Control
    alu_control ALU_CTRL (
        .instruction(instruction),
        .alu_op(ALUOp),
        .invFunc(invFunc),
        .alu_control_signal(alu_control_signal)
    );

endmodule

// Fixed Multiplexer Module (No Changes Needed)
module Mux(
    input [63:0] input1,
    input [63:0] input2,
    input select,
    output [63:0] out
);
    assign out = select ? input2 : input1;
endmodule
