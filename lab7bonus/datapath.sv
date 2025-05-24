module mux2bb(a3,a2,a1,a0,s,b);
    parameter k = 1; //width of input and output
    input [k-1: 0] a0, a1, a2, a3;
    //2 bit BINARY select
    input [1:0] s;
    output [k-1: 0] b;
    reg [k-1: 0] b;

    always_comb begin
        case(s)
            2'b00: b = a0;
            2'b01: b = a1;
            2'b10: b = a2;
            2'b11: b = a3;
            default: b = {k{1'bx}};
        endcase
    end
endmodule

module mux4a(a3,a2,a1,a0,s,b);
    parameter k = 1; //width of input and output
    input [k-1: 0] a0, a1, a2, a3;
    //4 bit one hot select
    input [3:0]s;
    output [k-1: 0] b;
    reg [k-1: 0] b;

    always_comb begin
        case(s)
            4'b0001: b = a0;
            4'b0010: b = a1;
            4'b0100: b = a2;
            4'b1000: b = a3;
            default: b = {k{1'bx}};
        endcase
    end
endmodule


//modifications added sximm8, sximm5 inputs
module datapath(mdata, sximm8, sximm5, vsel, out,
                writenum, write, readnum, clk,
                loada, asel,
                loadb, shift, bsel,
                ALUop,
                loadc, loads, N, V, Z, PC);

    input write,clk,loada,loadb,loadc,loads;
    //modification to vsel 1 bits --> 4 bits
    input [3:0] vsel;
    //modification: added sximm8, sximm5, mdata, PC (16 bits)
    input [15:0] sximm8, sximm5;
    input [15:0] mdata;
    input [2:0] writenum, readnum;
    input [1:0] shift, ALUop, asel,bsel;
    wire [15:0] data_in, data_out, out_A, out_B, sout, Ain, Bin, ALUout;
    input [8:0] PC;
    //modification: added Z_in, N_in, V_in (1 bit)
    wire Z_in, N_in, V_in;
    ////modification: added V, N, Z(1 bit)
    output V,N,Z;
    output [15:0] out;
    
    //Mux for selecting data_in PC replaced with 8'b0
    mux4a #(16) datapathMUX(mdata, sximm8, { {7'b0}, PC}, out, vsel, data_in); //(a3, a2, a1, a0, s, b)

    regfile REGFILE(data_in,writenum,write,readnum,clk,data_out);

    //instantiate registers A,B,C and status
    //(clk, en, in, out)
    vDFFE #(16) EA(clk, loada, data_out, out_A);
    vDFFE #(16) EB(clk, loadb, data_out, out_B);
    vDFFE #(16) EC(clk, loadc, ALUout, out);

    //modification: status reg extended to 3 bitS {zero flag, neg flag, overflow tag} 
    //              output to 3 bits concatenated for flags mentioned
    vDFFE #(3) ESTATUS(clk, loads, {Z_in, N_in, V_in}, {Z, N, V});


    //Muxes for registers A and B
    mux2bb #(16) A_MUX( 16'bx,{{7'b0},PC}, 16'b0, out_A, asel, Ain); //(a3, a2, a1, a0, s, b)
    mux2bb #(16) B_MUX( 16'bx,sximm8, sximm5, sout, bsel, Bin); //(a3, a2, a1, a0, s, b)

    //alu instantiation
    ALU alu(Ain, Bin, ALUop, ALUout, {Z_in, N_in, V_in});

    //shifter instantiation
    shifter shifter(out_B,shift,sout);
endmodule
