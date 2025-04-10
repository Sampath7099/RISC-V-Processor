`include "instruction_fetch.v"
`include "instruction_decode.v"
`include "alu_control.v"
`include "alu.v"
`include "execute.v"
`include "memory_access.v"
`include "hazard_unit.v"
`include "fwdunit.v"

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
        register[14] = 64'h0000000000000010; // x14 = 8
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
        data_memory[0] = 64'h0000000000000000;
        data_memory[2] = 64'h0000000000000001;
        data_memory[1] = 64'h0000000000000100;
        
    end

    // Fetch Stage
    wire [31:0] instruction;
    wire invAddr;
    instruction_fetch fetch_unit (
        .PC(PC),
        .instruction(instruction),
        .invAddr(invAddr)
    );
    wire [63:0] updated_PC, next_PC_final;
    ALU alu_pc_update (
        .a(PC),
        .b(64'd4),
        .alu_control_signal(4'b0010), // Addition
        .alu_result(updated_PC)
    );

    // ID/EX Pipeline Register
    wire [63:0] pc_if_id;
    wire [31:0] instruction_if_id;
    // id_if write is different
    IF_ID_Reg if_id_register (
        .clk(clock),
        .rst(reset),
        .pc_in(PC),
        .instruction_in(instruction),
        .branch_signal(branch_signal),
        .IF_ID_Write(IF_ID_Write),
        .pc_out(pc_if_id),
        .instruction_out(instruction_if_id)
    );

    // Instruction Decode Stage
    wire [4:0] rs1, rs2, write_reg;
    wire [9:0] alu_control;
    wire alusrc, regwrite, memread, memtoreg, memwrite, branch, invOp, invFunc, invRegAddr;
    wire [1:0] ALUOp;

    instruction_decode decode_unit (
        .instruction(instruction_if_id),
        .rs1(rs1),
        .rs2(rs2),
        .write_addr(write_reg),
        .alu_control(alu_control),
        .ALUSrc(alusrc),
        .ALUOp(ALUOp),
        .RegWrite(regwrite),
        .MemRead(memread),
        .MemtoReg(memtoreg),
        .MemWrite(memwrite),
        .Branch(branch),
        .invOp(invOp),
        .invFunc(invFunc),
        .invRegAddr(invRegAddr)
    );

    
    wire [63:0] imm_val;
    wire [63:0] immediate;

    // Immediate extraction for Load and Store
    assign imm_val = memwrite ? 
        {{52{instruction_if_id[31]}}, instruction_if_id[31:25], instruction_if_id[11:7]} // S-type (Store)
        : {{52{instruction_if_id[31]}}, instruction_if_id[31:20]}; // I-type (Load)

    // Immediate selection based on ALUOp
    assign immediate = (ALUOp == 2'b00) ? imm_val : 
                    (ALUOp == 2'b01) ? {{51{instruction_if_id[31]}}, instruction_if_id[31], instruction_if_id[7], 
                                        instruction_if_id[30:25], instruction_if_id[11:8] } // B-type (Branch)
                    : 64'd0;


    wire [63:0] shifted_immediate, next_PC;
    ALU alu_shift (
        .a(immediate),
        .b(64'd1),
        .alu_control_signal(4'b0011), // Logical Shift Left
        .alu_result(shifted_immediate)
    );

    ALU alu_branch (
        .a(pc_if_id),
        .b(shifted_immediate),
        .alu_control_signal(4'b0010), // Addition
        .alu_result(next_PC)
    );

    assign branch_signal = branch & (register[rs1] == register[rs2]);
    
    Mux next_pc_mux (
        .input1(updated_PC),
        .input2(next_PC),
        .select(branch_signal),
        .out(next_PC_final)
    );


    //Hazard Unit
    wire PCWrite, IF_ID_Write, stall, alusrc_after_stall, ALUOp_after_stall, branch_after_stall, memwrite_after_stall, memread_after_stall, memtoreg_after_stall, regwrite_after_stall;
    // ID/EX Pipeline Register
    wire [63:0] pc_id_ex, rd1_id_ex, rd2_id_ex, imm_val_id_ex;
    wire [9:0] alu_control_id_ex;
    wire [4:0] write_reg_id_ex, register_rs1_id_ex, register_rs2_id_ex;
    wire alusrc_id_ex, branch_id_ex, memwrite_id_ex, memread_id_ex, memtoreg_id_ex, regwrite_id_ex;
    wire [1:0] alu_op_id_ex;
    wire [31:0] instruction_id_ex;

    HazardUnit hazard_unit (
        .PCWrite(PCWrite),
        .IF_ID_Write(IF_ID_Write),
        .ID_EX_MemRead(memread_id_ex),
        .ID_EX_RegisterRd(write_reg_id_ex),
        .IF_ID_RegisterRs1(instruction_if_id[19:15]),
        .IF_ID_RegisterRs2(instruction_if_id[24:20]),
        .stall(stall)
    );

    //stall logic
    assign alusrc_after_stall   = stall ? 0 : alusrc;
    assign ALUOp_after_stall    = stall ? 0 : ALUOp;
    assign branch_after_stall   = stall ? 0 : branch;
    assign memwrite_after_stall = stall ? 0 : memwrite;
    assign memread_after_stall  = stall ? 0 : memread;
    assign memtoreg_after_stall = stall ? 0 : memtoreg;
    assign regwrite_after_stall = stall ? 0 : regwrite;

    

    // Immediate Generation

    
    ID_EX_Reg id_ex_register (
        .clk(clock),
        .rst(reset),
        .pc_in(pc_if_id),
        .read_data1_in(register[rs1]),
        .read_data2_in(register[rs2]),
        .imm_val_in(immediate),
        .write_reg_in(write_reg),
        .alu_control_in(alu_control),
        .alusrc_in(alusrc_after_stall),
        .branch_in(branch_after_stall),
        .memwrite_in(memwrite_after_stall),
        .memread_in(memread_after_stall),
        .memtoreg_in(memtoreg_after_stall),
        .regwrite_in(regwrite_after_stall),
        .register_rs1_in(instruction_if_id[19:15]),
        .register_rs2_in(rs2),
        .alu_op_in(ALUOp),
        .instruction_in(instruction_if_id),
        .pc_out(pc_id_ex),
        .read_data1_out(rd1_id_ex),
        .read_data2_out(rd2_id_ex),
        .imm_val_out(imm_val_id_ex),
        .write_reg_out(write_reg_id_ex),
        .alu_control_out(alu_control_id_ex),
        .alusrc_out(alusrc_id_ex),
        .branch_out(branch_id_ex),
        .memwrite_out(memwrite_id_ex),
        .memread_out(memread_id_ex),
        .memtoreg_out(memtoreg_id_ex),
        .regwrite_out(regwrite_id_ex),
        .register_rs1_out(register_rs1_id_ex),
        .register_rs2_out(register_rs2_id_ex),
        .alu_op_out(alu_op_id_ex),
        .instruction_out(instruction_id_ex)
    );

    // ALU Control
    wire [3:0] alu_control_signal;
    alu_control ALU_CTRL (
        .alu_control(alu_control_id_ex),
        .alu_op(alu_op_id_ex),
        .invFunc(invFunc),
        .alu_control_signal(alu_control_signal)
    );


    wire [63:0] rd1, rd2, wd, w1, alu_in1, alu_in2;
    assign invRegAddr = (rs1 > 5'd31) | (rs2 > 5'd31);
    assign rd1 = rd1_id_ex;
    assign w1 = rd2_id_ex;
    assign rd2 = (alusrc_id_ex) ? imm_val_id_ex : w1;


    // Forwarding
    MUX3 mux3_alu_in1 (
        .in0(rd1),
        .in1(wd),
        .in2(alu_result_ex_mem),
        .sel(ForwardA),
        .out(alu_in1)
    );

    MUX3 mux3_alu_in2 (
        .in0(rd2),
        .in1(wd),
        .in2(alu_result_ex_mem),
        .sel(ForwardB),
        .out(alu_in2)
    );

    // Execute Stage
    wire [63:0] alu_output;
    wire zer0_flag;
    execute execute_unit (
        .alu_control_signal(alu_control_signal),
        .rd1(alu_in1),
        .rd2(alu_in2),
        .PC(pc_id_ex),
        .ALUOp(alu_op_id_ex),
        .immediate(imm_val_id_ex),
        .Branch(branch_id_ex),
        .alu_output(alu_output),
        .next_PC(next_PC),
        .zero(zer0_flag)
    );

    // EX/MEM Pipeline Register
    wire [63:0] pc_ex_mem;
    wire [63:0] alu_result_ex_mem;
    wire [4:0] write_reg_ex_mem;
    wire branch_ex_mem, memwrite_ex_mem, memread_ex_mem, memtoreg_ex_mem, regwrite_ex_mem;
    wire zer0_id_ex;
    EX_MEM_Reg ex_mem_register (
        .clk(clock),
        .rst(reset),
        .pc_in(next_PC),
        .zero_in(zero_flag),
        .alu_result_in(alu_output[63:0]),
        .write_reg_in(write_reg_id_ex),
        .branch_in(branch_id_ex),
        .memwrite_in(memwrite_id_ex),
        .memread_in(memread_id_ex),
        .memtoreg_in(memtoreg_id_ex),
        .regwrite_in(regwrite_id_ex),
        .pc_out(pc_ex_mem),
        .zero_out(zer0_ex_mem),
        .alu_result_out(alu_result_ex_mem),
        .write_reg_out(write_reg_ex_mem),
        .branch_out(branch_ex_mem),
        .memwrite_out(memwrite_ex_mem),
        .memread_out(memread_ex_mem),
        .memtoreg_out(memtoreg_ex_mem),
        .regwrite_out(regwrite_ex_mem)
    );
    
    // Branch Logic


    wire [1:0] ForwardA, ForwardB;
    Forwarding_Unit fwdunit(
        .ID_EX_Rs1(register_rs1_id_ex),
        .ID_EX_Rs2(register_rs2_id_ex),
        .EX_MEM_Rd(write_reg_ex_mem),
        .MEM_WB_Rd(write_reg_mem_wb),
        .EX_MEM_RegWrite(regwrite_ex_mem),
        .MEM_WB_RegWrite(regwrite_mem_wb),
        .ForwardA(ForwardA),
        .ForwardB(ForwardB)
    );
    
    wire invMemAddr;

    // Memory Access Stage
    memory_access MEM_stage (
        .MemWrite(memwrite_ex_mem),
        .MemRead(memread_ex_mem),
        .MemtoReg(memtoreg_ex_mem),
        .address(alu_result_ex_mem),
        .invMemAddr(invMemAddr)
    );

    reg [63:0] read_data;
    always @(*) begin
        if (~invMemAddr)  begin
            if (memread_ex_mem) 
                read_data <= data_memory[alu_result_ex_mem >> 3]; // Read from memory
            else if (memwrite_ex_mem & !invMemAddr)
                data_memory[alu_result_ex_mem >> 3] <= w1;    
        end
    end

    // MEM/WB Pipeline Register
    wire [63:0] alu_result_mem_wb, read_data_mem_wb;
    wire [4:0] write_reg_mem_wb;
    MEM_WB_Reg mem_wb_register (
        .clk(clock),
        .rst(reset),
        .alu_result_in(alu_result_ex_mem),
        .read_data_in(read_data),
        .write_reg_in(write_reg_ex_mem),
        .memtoreg_in(memtoreg_ex_mem),
        .regwrite_in(regwrite_ex_mem),
        .alu_result_out(alu_result_mem_wb),
        .read_data_out(read_data_mem_wb),
        .write_reg_out(write_reg_mem_wb),
        .memtoreg_out(memtoreg_mem_wb),
        .regwrite_out(regwrite_mem_wb)
    );

    Mux mem_mux (
        .input1(alu_result_mem_wb),
        .input2(read_data_mem_wb),
        .select(memtoreg_mem_wb),
        .out(wd)
    );

    integer halt = 1; // Use reg instead of parameter

    always @(posedge clock) begin
        if (reset)
            PC <= 0;
        else if (halt == 4)
            $finish;
        else if (instruction == 64'hFFFFFFFF) begin
            PC <= PC;
            halt <= halt + 1; // Use <= for sequential updates
        end
        else if (PCWrite)
            PC <= next_PC_final; // Your next-PC logic
        else
            PC <= PC; // Hold PC value
    end



    always @(posedge clock) begin
        if(regwrite_mem_wb & !invRegAddr)
            register[write_reg_mem_wb] <= wd;
    end
    
endmodule