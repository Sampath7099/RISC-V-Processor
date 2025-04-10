module alu_control (
    input [31:0] instruction,
    input [1:0] alu_op,
    output reg [3:0] alu_control_signal,
    output reg invFunc
);
    always @(*) begin
        invFunc = 0; 
        
        if (alu_op == 2'b00) 
            alu_control_signal = 4'b0010; // ADD operation
        else if (alu_op == 2'b01) 
            alu_control_signal = 4'b0110; // SUB operation
        else if (alu_op == 2'b10) begin
            case ({instruction[31:25], instruction[14:12]})
                10'b0000000000: alu_control_signal = 4'b0010; // ADD
                10'b0100000000: alu_control_signal = 4'b0110; // SUB
                10'b0000000110: alu_control_signal = 4'b0001; // OR
                10'b0000000111: alu_control_signal = 4'b0000; // AND
                default: begin
                    alu_control_signal = 4'b1111; // Undefined operation
                    invFunc = 1; // Set invalid function flag
                end
            endcase
        end 
        else begin
            alu_control_signal = 4'b1111; // Undefined ALU operation
            invFunc = 1;
        end
    end
endmodule
