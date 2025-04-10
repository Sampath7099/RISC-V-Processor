module Forwarding_Unit (
    input wire [4:0] ID_EX_Rs1,      // Source register 1 in ID/EX stage
    input wire [4:0] ID_EX_Rs2,      // Source register 2 in ID/EX stage
    input wire [4:0] EX_MEM_Rd,      // Destination register in EX/MEM stage
    input wire [4:0] MEM_WB_Rd,      // Destination register in MEM/WB stage
    input wire EX_MEM_RegWrite,      // Register write signal from EX/MEM
    input wire MEM_WB_RegWrite,      // Register write signal from MEM/WB

    output reg [1:0] ForwardA,       // Forwarding control for ALU input A
    output reg [1:0] ForwardB        // Forwarding control for ALU input B
);

always @(*) begin
    // Default values (no forwarding)
    ForwardA = 2'b00;
    ForwardB = 2'b00;

    // Check forwarding for Rs1 (ForwardA)
    if (EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs1)) 
        ForwardA = 2'b10; // Forward from EX/MEM stage

    else if (MEM_WB_RegWrite && (MEM_WB_Rd != 0) && 
            !(EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs1)) && 
            (MEM_WB_Rd == ID_EX_Rs1)) 
        ForwardA = 2'b01; // Forward from MEM/WB stage

    // Check forwarding for Rs2 (ForwardB)
    if (EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs2)) 
        ForwardB = 2'b10; // Forward from EX/MEM stage

    else if (MEM_WB_RegWrite && (MEM_WB_Rd != 0) && 
            !(EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs2)) && 
            (MEM_WB_Rd == ID_EX_Rs2)) 
        ForwardB = 2'b01; // Forward from MEM/WB stage
end

endmodule

module MUX3 (
    input wire [63:0] in0,  // Input 0
    input wire [63:0] in1,  // Input 1
    input wire [63:0] in2,  // Input 2
    input wire [1:0] sel,   // 2-bit select signal
    output reg [63:0] out   // Output
);

always @(*) begin
    case (sel)
        2'b00: out = in0;   // Select input 0
        2'b01: out = in1;   // Select input 1
        2'b10: out = in2;   // Select input 2
        default: out = 32'b0; // Default case (could be in0 or another value)
    endcase
end

endmodule
