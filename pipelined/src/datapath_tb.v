`timescale 1ns/1ps

module datapath_tb;
    reg clock;
    reg reset;
    integer i;

    // Instantiate the datapath module
    datapath uut (
        .clock(clock),
        .reset(reset)
    );

    // Clock Generation (50% Duty Cycle)
    always #5 clock = ~clock;
    initial begin
            $readmemb("instructions.txt", uut.fetch_unit.instr_mem);
    end
    initial begin
        $dumpfile("file.vcd"); // Name of the VCD file
        $dumpvars(0, datapath_tb); // Dumps all signals from the testbench module
    end

    initial begin
        // Initialize clock and reset
        clock = 0;
        reset = 1;
        
        $display("Cycle 0:");
        $display("PC: %d", uut.PC);
        $display("IF:  Instruction = %h", uut.fetch_unit.instr_mem[uut.PC >> 2]);
        
            $display("------------------------------------------------------");

        #5 reset = 0; // Release reset after some time
        // Run for a few clock cycles to allow pipelining
        forever begin
            #10;
            $display("Cycle %d:", $time/10);
            $display("PC: %d", uut.PC);
            
            // Fetch Stage
            $display("IF:  Instruction = %b  (%h)", uut.fetch_unit.instr_mem[uut.PC >> 2] , uut.fetch_unit.instr_mem[uut.PC >> 2]);
            
            // Decode Stage
            $display("ID:  Instruction = %h, Rs1 = %d, Rs2 = %d, Rd = %d, imm = %d", 
                     uut.instruction_if_id, uut.rs1, uut.rs2, uut.write_reg, uut.imm_val);
            
            // Execute Stage
            $display("EX:  ALU Control = %h, Alu_in1 = %h, Alu_in2 = %h, Alu_output = %h", 
                     uut.instruction_id_ex, uut.alu_in1, uut.alu_in2, uut.alu_output);
            
            // Memory Access Stage
            $display("MEM: Address = %h, MemRead = %b, MemWrite = %b, Data = %h",
                     uut.alu_result_ex_mem, uut.memread_ex_mem, uut.memwrite_ex_mem, uut.data_memory[uut.alu_result_ex_mem >> 3]);
            
            // Write Back Stagez
            $display("WB:  RegWrite = %b, WriteReg = %d, WriteData = %h", 
                     uut.regwrite_mem_wb, uut.write_reg_mem_wb, uut.wd);
            
            $display("------------------------------------------------------");
        end
        for (i = 0; i <= 20; i = i + 1) begin
                $display("Register[%0d] = %2d", i, uut.register[i]);
            end

        // Stop the simulation
        $finish;
    end
endmodule
