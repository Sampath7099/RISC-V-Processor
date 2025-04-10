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

    reg [63:0] PC;
    reg [63:0] register [0:31];
    reg [63:0] data_memory [0:1023];

    initial begin
        PC = 64'h0;
        register[0]  = 64'h0000000000000000; // x0  = 0
        register[1]  = 64'h0000000000000001; // x1  = 1
        register[2]  = 64'h0000000000000002; // x2  = 2
        register[3]  = 64'h0000000000000003; // x3  = 3
        register[4]  = 64'h0000000000000004; // x4  = 4
        register[5]  = 64'h0000000000000005; // x5  = 5
        register[6]  = 64'h0000000000000006; // x6  = 6
        register[7]  = 64'h0000000000000007; // x7  = 7
        register[8]  = 64'h0000000000000008; // x8  = 8
        register[9]  = 64'h0000000000000009; // x9  = 9
        register[10] = 64'h000000000000000A; // x10 = 10
        register[11] = 64'h000000000000000B; // x11 = 11
        register[12] = 64'h000000000000000C; // x12 = 12
        register[13] = 64'h000000000000000D; // x13 = 13
        register[14] = 64'h0000000000000008; // x14 = 8
        register[15] = 64'h000000000000000F; // x15 = 15
        register[16] = 64'h0000000000000010; // x16 = 16
        register[17] = 64'h0000000000000008; // x17 = 4
        register[18] = 64'h0000000000000012; // x18 = 18
        register[19] = 64'h0000000000000013; // x19 = 19
        register[20] = 64'h0000000000000014; // x20 = 20
        register[21] = 64'h0000000000000015; // x21 = 21
        register[22] = 64'h0000000000000016; // x22 = 22
        register[23] = 64'h0000000000000017; // x23 = 23
        register[24] = 64'h0000000000000018; // x24 = 24
        register[25] = 64'h0000000000000019; // x25 = 25
        register[26] = 64'h000000000000001A; // x26 = 26
        register[27] = 64'h000000000000001B; // x27 = 27
        register[28] = 64'h000000000000001C; // x28 = 28
        register[29] = 64'h000000000000001D; // x29 = 29
        register[30] = 64'h000000000000001E; // x30 = 30
        register[31] = 64'h000000000000001F; // x31 = 31
        data_memory[1] = 64'h000000000000001F;
    end

    
    wire [31:0] instruction;
    wire invAddr;
    wire [4:0] rs1, rs2;
    wire [63:0] rd1, rd2;
    wire [4:0] write_addr;
    wire [3:0] alu_control_signal;
    wire RegWrite, MemRead, MemtoReg, MemWrite, Branch;
    wire invOp, invFunc, invRegAddr;
    wire [63:0] alu_output, next_PC;
    wire [63:0] wd;
    wire invMemAddr;
    wire [63:0] immediate;
    reg [63:0] read_data;
    

    instruction_fetch IF_stage (
        .PC(PC),
        .instruction(instruction),
        .invAddr(invAddr)
    );

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

    wire [63:0] immediate_value;
    assign immediate_value = MemWrite ? {{52{instruction[31]}}, instruction[31:25], instruction[11:7]}  // Store
                                    : {{52{instruction[31]}}, instruction[31:20]};  // Load

    assign immediate = (alu_control_signal == 4'b0010) ? immediate_value :
                    (alu_control_signal == 4'b0110) ? {{51{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8]}  // Branch
                                                    : 64'd0;


    assign invRegAddr = (rs1 > 5'd31) | (rs2 > 5'd31);
    assign rd1 = register[rs1];
    assign w1 = register[rs2];
    
    Mux alu_mux (
        .input1(register[rs2]),
        .input2(immediate),
        .select(ALUSrc),
        .out(rd2)
    );

    // Execute Stage
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
    
    // Memory Access Stage
    memory_access MEM_stage (
        .MemWrite(MemWrite),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .address(alu_output),
        .invMemAddr(invMemAddr)
    );

    Mux mem_mux (
        .input1(alu_output),
        .input2(read_data),
        .select(MemtoReg),
        .out(wd)
    );


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


    always @(posedge clock) begin
        if(RegWrite & !invRegAddr & write_addr != 0)
            register[write_addr] <= wd;
        else if (MemWrite & !invMemAddr)
            data_memory[alu_output / 8] <= w1;
    end
    always @(*) begin
        if (~invMemAddr)  begin
            if (MemRead) 
                read_data <= data_memory[alu_output / 8]; 
        end
    end
    
    
endmodule



