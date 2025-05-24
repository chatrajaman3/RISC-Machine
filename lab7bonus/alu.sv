module ALU(Ain, Bin, ALUop, out, {Z_in, N_in, V_in});
    input [15:0] Ain, Bin;
    input [1:0] ALUop;
    output reg[15:0] out;
    output reg Z_in, N_in, V_in;

    wire sid, siod;

    always_comb begin
        case(ALUop)
        2'b00: out = Ain + Bin;
        2'b01: out = Ain - Bin;
        2'b10: out = Ain & Bin;
        2'b11: out = ~Bin;
        default: out = 16'bx;
        endcase

        // modified for lab 6, 
        casex(out)
        16'b0: begin Z_in = 1'b1; N_in = 1'b0; end    //z bit represents zero flag
        16'b1xxx_xxxx_xxxx_xxxx: begin Z_in = 1'b0; N_in = 1'b1; end //negative flag, set to 1 if msb is 1
        default: begin 
                Z_in = 1'b0;
                N_in = 1'b0;
            end
        endcase
    end

    //code for signed overflow detection from fig.10.11 in dally book
    //xor gate for sid internal wire, connecting msb of ain and bin, detects if 2 input signs are diff
    assign sid = Ain[15] ^ Bin[15];
    //xor gate for siod, from msb of output and ain, detects if an input sign is diff from output sign
    assign siod = Ain[15] ^ out[15];

    //and gate for overflow bit, with not'd sid and siod, checks if two input signs are the same and diff from output, if yes means overflow
    //ex. we add pos # + pos # and get neg # output, this means overflow
    assign V_in = sid & siod;
endmodule