module memory_access(
    input MemWrite,
    input MemRead,
    input MemtoReg,
    input [63:0] address,
    output reg invMemAddr
);

    always @(*) begin
        // Default values
        invMemAddr = 0;

        // Check for valid memory address when MemRead or MemWrite is active
        if ((MemRead || MemWrite) && (((address / 8) > 1023) || (address[1:0] != 0))) begin
            invMemAddr = 1;
        end
    end

endmodule
