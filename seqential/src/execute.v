module execute(
    input [3:0] alu_control_signal,
    input [63:0] rd1,
    input [63:0] rd2,
    input [63:0] PC,
    input [63:0] immediate,
    input Branch, 
    output [63:0] alu_output,
    output [63:0] next_PC
);
    
    wire [63:0] updated_PC;
    wire [63:0] branch_target;

    // ALU computation for normal operations
    ALU alu_main (
        .a(rd1),
        .b(rd2),
        .alu_control_signal(alu_control_signal),
        .alu_result(alu_output)
    );

    // ALU for PC + 4
    ALU alu_pc_update (
        .a(PC),
        .b(64'd4),
        .alu_control_signal(4'b0010), // Addition
        .alu_result(updated_PC)
    );

    // ALU for shifting immediate left by 1 bit
    wire [63:0] shifted_immediate;
    ALU alu_shift (
        .a(immediate),
        .b(64'd1),
        .alu_control_signal(4'b0011), // Logical Shift Left
        .alu_result(shifted_immediate)
    );
    assign branch_signal = Branch & (alu_output == 0);
    // ALU for branch target calculation (PC + shifted immediate)
    ALU alu_branch (
        .a(PC),
        .b(shifted_immediate),
        .alu_control_signal(4'b0010), // Addition
        .alu_result(branch_target)
    );

    // MUX to choose between branch_target and updated_PC
    Mux next_pc_mux (
        .input1(updated_PC),
        .input2(branch_target),
        .select(branch_signal),
        .out(next_PC)
    );

endmodule
