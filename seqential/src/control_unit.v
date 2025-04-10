module ControlUnit (
    input [6:0] opcode,
    output reg RegWrite,
    output reg ALUSrc,
    output reg MemRead,
    output reg MemtoReg,
    output reg MemWrite,
    output reg Branch,
    output reg [1:0] ALUOp,
    output reg invOp  
);
    
    always @(*) begin
        RegWrite = 0;
        ALUSrc   = 0;
        MemRead  = 0;
        MemtoReg = 0;
        MemWrite = 0;
        Branch   = 0;
        ALUOp    = 2'b00;
        invOp    = 0;  

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
