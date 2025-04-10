module HazardUnit(
    output reg PCWrite,
    output reg IF_ID_Write,
    input ID_EX_MemRead,
    input [4:0] ID_EX_RegisterRd,
    input [4:0] IF_ID_RegisterRs1,
    input [4:0] IF_ID_RegisterRs2,
    output reg stall
);

    always @(*) begin
        PCWrite = 1'b1;
        IF_ID_Write = 1'b1;
        stall = 1'b0;
        
        if (ID_EX_MemRead && 
            ((ID_EX_RegisterRd == IF_ID_RegisterRs1) || 
             (ID_EX_RegisterRd == IF_ID_RegisterRs2))) begin
            stall = 1'b1;
            PCWrite = 1'b0;
            IF_ID_Write = 1'b0;
        end
    end   
endmodule
