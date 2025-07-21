`include "instruction_fetch.v"
`include "instruction_decode.v"
`include "control_unit.v"
`include "alu_control.v"
`include "alu.v"
`include "execute.v"
`include "memory_access.v"

module datapath(
    input wire clock,
    input wire reset
);

    // ----------------------------------------------------
    // Registers and Memories
    // ----------------------------------------------------
    reg [63:0] PC;
    reg [63:0] register [0:31];
    reg [63:0] data_memory [0:1023];

    initial begin
        $readmemb("register.txt", register);
    end

    initial begin
        PC = 64'h0;
        data_memory[1] = 64'h000000000000001F;
    end

    // ----------------------------------------------------
    // Instruction Fetch Wires
    // ----------------------------------------------------
    wire [31:0] instruction;
    wire invAddr;

    // ----------------------------------------------------
    // Instruction Fetch Stage
    // ----------------------------------------------------
    instruction_fetch IF_stage (
        .PC(PC),
        .instruction(instruction),
        .invAddr(invAddr)
    );

    

    // ----------------------------------------------------
    // Instruction Decode Wires
    // ----------------------------------------------------
    wire [4:0] rs1, rs2;
    wire [4:0] write_addr;
    wire [3:0] alu_control_signal;
    wire RegWrite, MemRead, MemtoReg, MemWrite, Branch;
    wire invOp, invFunc, invRegAddr;

    // ----------------------------------------------------
    // Immediate Value Calculation
    // ----------------------------------------------------
    wire [63:0] immediate_value;
    assign immediate_value = MemWrite ? {{52{instruction[31]}}, instruction[31:25], instruction[11:7]} : // Store
                                          {{52{instruction[31]}}, instruction[31:20]};                 // Load
    
    // ----------------------------------------------------
    // Instruction Decode Stage
    // ----------------------------------------------------
    instruction_decode ID_stage (
        .instruction(instruction),
        .rs1(rs1),
        .rs2(rs2),
        .write_addr(write_addr),
        .alu_control_signal(alu_control_signal),
        .ALUSrc(ALUSrc),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .MemWrite(MemWrite),
        .Branch(Branch),
        .invOp(invOp),
        .invFunc(invFunc),
        .invRegAddr(invRegAddr)
    );

    assign immediate = (alu_control_signal == 4'b0010) ? immediate_value :
                       (alu_control_signal == 4'b0110) ? {{51{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8]} : // Branch
                       64'd0;

    // ----------------------------------------------------
    // Register File Read
    // ----------------------------------------------------
    assign invRegAddr = (rs1 > 5'd31) | (rs2 > 5'd31);
    assign rd1 = register[rs1];
    assign w1 = register[rs2];

    // ----------------------------------------------------
    // ALU Mux
    // ----------------------------------------------------
    Mux alu_mux (
        .input1(register[rs2]),
        .input2(immediate),
        .select(ALUSrc),
        .out(rd2)
    );

    // ----------------------------------------------------
    // Execute & ALU Wires
    // ----------------------------------------------------
    wire [63:0] rd1, rd2;
    wire [63:0] alu_output, next_PC;
    wire [63:0] immediate;
    wire [63:0] wd;

    // ----------------------------------------------------
    // Execute Stage
    // ----------------------------------------------------
    execute EX_stage (
        .alu_control_signal(alu_control_signal),
        .rd1(rd1),
        .rd2(rd2),
        .PC(PC),
        .immediate(immediate),
        .Branch(Branch),
        .alu_output(alu_output),
        .next_PC(next_PC)
    );

    // ----------------------------------------------------
    // Memory Access Wires
    // ----------------------------------------------------
    wire invMemAddr;
    reg [63:0] read_data;

    // ----------------------------------------------------
    // Memory Access Stage
    // ----------------------------------------------------
    memory_access MEM_stage (
        .MemWrite(MemWrite),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .address(alu_output),
        .invMemAddr(invMemAddr)
    );

    // ----------------------------------------------------
    // Memory to Register Mux
    // ----------------------------------------------------
    Mux mem_mux (
        .input1(alu_output),
        .input2(read_data),
        .select(MemtoReg),
        .out(wd)
    );

    // ----------------------------------------------------
    // Register Writeback and Memory Write
    // ----------------------------------------------------
    always @(posedge clock) begin
        if (RegWrite & !invRegAddr & write_addr != 0)
            register[write_addr] <= wd;
        else if (MemWrite & !invMemAddr)
            data_memory[alu_output / 8] <= w1;
    end

    // ----------------------------------------------------
    // Memory Read
    // ----------------------------------------------------
    always @(*) begin
        if (~invMemAddr) begin
            if (MemRead)
                read_data <= data_memory[alu_output / 8];
        end
    end


    // ----------------------------------------------------
    // PC Update & Halt Conditions
    // ----------------------------------------------------
    always @(posedge clock or posedge reset) begin
        if (reset)
            PC <= 64'd0; // Reset PC to 0

        else if (instruction == 64'hFFFFFFFF) begin
            $display("Program Execution completed.");
            $finish;
        end

        else if (invAddr) begin
            $display("ERROR: Program halted due to Invalid Instruction Address.");
            $finish;
        end

        else if (invOp) begin
            $display("ERROR: Program halted due to Invalid Opcode.");
            $finish;
        end

        else if (invRegAddr) begin
            $display("ERROR: Program halted due to Invalid Register Address.");
            $finish;
        end

        else if (invMemAddr) begin
            $display("ERROR: Program halted due to Invalid Memory Address.");
            $finish;
        end

        else if (invFunc) begin
            $display("ERROR: Program halted due to Invalid Function Code.");
            $finish;
        end

        else begin
            if (write_addr == 0 & RegWrite == 1)
                $display("WARNING: Attempt to write in X0.");

            PC <= next_PC;
        end
    end

    
endmodule
