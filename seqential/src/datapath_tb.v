module datapath_tb;
    // Testbench signals
    reg clock;
    reg [63:0] PC;
    
    // Instantiate the datapath module
    datapath uut (
        .clock(clock)
    );
    
    // Clock generation
    always #5 clock = ~clock; // Generate a clock with period 10ns
    
    initial begin
        $readmemb("instructions.txt", uut.IF_stage.instr_mem);
    end
    
    // Test sequence
    initial begin
        $dumpfile("file.vcd");
        $dumpvars(0, datapath_tb);
        
        clock = 0;
        uut.PC = 64'd0;

        $display(" PC | rs1 | rs2 | rd1 | rd2 | ALU | Write Data | Reg_Addr | Mem_Addr | Reg_Val | Mem_Val |");
        $display("-----------------------------------------------------------------------------------------");
        
        #5;
        $display("%3d | %3d | %3d | %3d | %3d | %3d | %10d | %8d | %8d |%8d |%8d | ", 
                uut.PC, uut.rs1, uut.rs2,uut.rd1, uut.rd2, $signed(uut.alu_output), $signed(uut.wd) , uut.write_addr, $signed(uut.alu_output)/8,uut.register[uut.write_addr], uut.data_memory[uut.alu_output/8]);
        
        forever begin
            #10;
            $display("%3d | %3d | %3d | %3d | %3d | %3d | %10d | %8d | %8d |%8d |%8d |", 
                uut.PC, uut.rs1, uut.rs2,uut.rd1, uut.rd2, $signed(uut.alu_output), $signed(uut.wd) , uut.write_addr,$signed(uut.alu_output)/8,uut.register[uut.write_addr], uut.data_memory[uut.alu_output/8]);
        end
        
        #50;
        $finish;
    end
    
endmodule
