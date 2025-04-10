module instruction_fetch (
    input [63:0] PC,  
    output reg [31:0] instruction,  
    output reg invAddr
);
    reg [31:0] instr_mem [0:1023]; 

    always @(*) begin
        if (PC[1:0] != 0 || PC[63:2] > 1023) begin
            invAddr = 1'b1;
            instruction = 32'hxxxxxxxx;  // Invalid instruction
        end else begin
            invAddr = 1'b0;
            instruction = instr_mem[PC[11:2]];  // Fetch instruction
        end
    end
endmodule
