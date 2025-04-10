module add_sub_unit (a, b, result, alu_control_signal, Cout);
    input  [63:0] a;
    input  [63:0] b;
    input [3:0] alu_control_signal;
    output [63:0] result;
    output Cout;
    
    wire [63:0] xor_b;
    wire [63:0] xor_bit = {64{alu_control_signal[2]}};
    reg [63:0] b_selected;
    wire Cin;
    assign Cin = alu_control_signal[2];
    xor_unit Xor_unit (.a(xor_bit), .b(b), .result(xor_b));
    adder_unit Add_Sub_Unit (.a(a), .b(xor_b), .sum(result), .Cin(Cin), .Cout(Cout));
    
endmodule
module bitwise_and (a, b, result);
    input a;
    input b;
    output wire result;
    
    assign result = a & b;
endmodule

module and_unit (a, b, out);
    input [63:0] a;
    input [63:0] b;
    output wire [63:0] out;
    
    genvar i;
    generate
        for (i = 0; i < 64; i = i + 1) begin : bitwise_and_loop
            bitwise_and and_inst (
                .a(a[i]),
                .b(b[i]),
                .result(out[i])
            );
        end
    endgenerate
endmodule
module bitwise_or (a, b, result);
    input a;
    input b;
    output wire result;
    
    assign result = a | b;
endmodule

module or_unit (a, b, out);
    input [63:0] a;
    input [63:0] b;
    output wire [63:0] out;
    
    genvar i;
    generate
        for (i = 0; i < 64; i = i + 1) begin : bitwise_or_loop
            bitwise_or or_inst (
                .a(a[i]),
                .b(b[i]),
                .result(out[i])
            );
        end
    endgenerate
endmodule

module bitwise_xor (a, b, result);
    input a;
    input b;
    output wire result;
    
    assign result = a ^ b;
endmodule

module xor_unit (a, b, result);
    input [63:0] a;
    input [63:0] b;
    output wire [63:0] result;
    
    genvar i;
    generate
        for (i = 0; i < 64; i = i + 1) begin 

            bitwise_xor xor_inst (
                .a(a[i]),
                .b(b[i]),
                .result(result[i])
            );
        end
    endgenerate
endmodule
module shift_unit (a, b, direction, result);
    input [63:0] a;
    input [63:0] b;
    input [1:0] direction;
    output reg [63:0] result;
    reg [63:0] temp;
    wire [4:0] shift;
    assign shift = b[4:0];
    always @(*) begin
        temp = a;
        if (shift[0] == 1)
            case (direction)
                2'b00: temp = {temp[62:0], 1'b0};
                2'b11: temp = {temp[63], temp[63:1]};
                default: temp = {1'b0, temp[63:1]};
            endcase
        if (shift[1] == 1)
            case (direction)
                2'b00: temp = {temp[61:0], 2'b0};
                2'b11: temp = {{2{temp[63]}}, temp[63:2]};
                default: temp = {2'b0, temp[63:2]};
            endcase
        if (shift[2] == 1)
            case (direction)
                2'b00: temp = {temp[59:0], 4'b0};
                2'b11: temp = {{4{temp[63]}}, temp[63:4]};
                default: temp = {4'b0, temp[63:4]};
            endcase
        if (shift[3] == 1)
            case (direction)
                2'b00: temp = {temp[55:0], 8'b0};
                2'b11: temp = {{8{temp[63]}}, temp[63:8]};
                default: temp = {8'b0, temp[63:8]};
            endcase
        if (shift[4] == 1)
            case (direction)
                2'b00: temp = {temp[47:0], 16'b0};
                2'b11: temp = {{16{temp[63]}}, temp[63:16]};
                default: temp = {16'b0, temp[63:16]};
            endcase
        result = temp;
    end
    
endmodule

module bitwise_adder (a, b, cin, sum, cout);
    input a;
    input b;
    input cin;
    output sum;
    output cout;
    wire w1, w2, w3;

    xor x1(w1, a, b);
    xor x2(sum, w1, cin);
    and a1(w2, a, b);
    and a2(w3, w1, cin);
    or o1(cout, w2, w3);
endmodule

module adder_unit (a, b, sum, Cin, Cout);
    input  [63:0] a;
    input  [63:0] b;
    input Cin;
    output wire [63:0] sum;
    output Cout;
    wire [64:0] carry;
    assign carry[0] = Cin;

    genvar i;
    generate
        for (i = 0; i < 64; i = i + 1) 
        begin
            bitwise_adder Adder (a[i], b[i], carry[i], sum[i], carry[i+1]);
        end
    endgenerate
    assign Cout = carry[64];

    
endmodule

module ALU (a,b,alu_control_signal, alu_result);
    
    input [63:0] a;
    input [63:0] b;
    input [3:0] alu_control_signal;
    output reg [63:0]  alu_result;

    wire [63:0] add_sub_result;
    wire Cout;
    add_sub_unit Add_Sub_unit (.a(a),.b(b),.result(add_sub_result),.alu_control_signal(alu_control_signal),.Cout(Cout));

    wire [63:0] shift_result;
    wire [1:0] shift;
    assign shift = alu_control_signal[3:2];
    shift_unit Shift_unit (.a(a),.b(b),.direction(shift),.result(shift_result));

    wire [63:0] and_result;
    and_unit And_unit (.a(a),.b(b),.out(and_result));

    wire [63:0] or_result;
    or_unit Or_unit (.a(a),.b(b),.out(or_result));

    wire [63:0] xor_result;
    xor_unit xor_unit (.a(a),.b(b),.result(xor_result));
    
    always @(*) begin
        if (alu_control_signal == 4'b0010 || alu_control_signal == 4'b0110) 
             alu_result = add_sub_result;
        else if (alu_control_signal == 4'b0100) 
             alu_result = xor_result;
        else if (alu_control_signal == 4'b0001) 
             alu_result = or_result;
        else if (alu_control_signal == 4'b0000) 
             alu_result = and_result;
        else if (alu_control_signal == 4'b0011)
             alu_result = shift_result;
        else 
             alu_result = 0; 
    end


endmodule


