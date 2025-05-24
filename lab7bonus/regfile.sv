//reg with load enable module
//citation: code used from slide set: lab 5 introduction
module vDFFE(clk, en, in, out);
    parameter n=1; //width
    input clk, en;
    input [n-1:0] in;
    output [n-1:0] out;
    reg[n-1:0] out;
    wire[n-1:0] next_out;

    assign next_out = en ? in: out;

    always @(posedge clk)
        out=next_out;
endmodule

//module for n:m decoder, for instatiation
//citation: slide set 6 - verilog
module Dec(a,b); //a = in, b = out
    parameter n = 3; //in width
    parameter m = 8; //out width
    
    input[n-1: 0] a;
    output[m-1: 0] b;

    wire[m-1: 0] b = 1 << a;
endmodule

//module for parameterized mux, 8input , for instatiation
//citation: set 6, verilog
//a=in, s=select, b=out
module Mux8a(a7, a6, a5, a4, a3, a2, a1, a0, s, b);
    parameter k = 1; //width of input and output
    input [k-1: 0] a0, a1, a2, a3, a4, a5, a6, a7;
    //8 bit select
    input [7:0] s;
    output [k-1: 0] b;
    reg [k-1: 0] b;

    always_comb begin
        case(s)
            8'b00000001: b = a0;
            8'b00000010: b = a1;
            8'b00000100: b = a2;
            8'b00001000: b = a3;
            8'b00010000: b = a4;
            8'b00100000: b = a5;
            8'b01000000: b = a6;
            8'b10000000: b = a7;
            default: b = {k{1'bx}};
        endcase
    end
endmodule


module regfile(data_in,writenum,write,readnum,clk,data_out);
    input [15:0] data_in;
    input [2:0] writenum, readnum;
    input write, clk;
    output [15:0] data_out;


    //instantiate 3:8 decoder for writenum
    wire [7:0] writeval; //One hot code of register number
    Dec U1(writenum,writeval);

    //and gates for decoder output then becomes input for reg load enable
    wire r0_in;
    wire r1_in;
    wire r2_in;
    wire r3_in;
    wire r4_in;
    wire r5_in;
    wire r6_in;
    wire r7_in;

    //and gates will go into load enable as inputs en
    assign r0_in = write & writeval[0];
    assign r1_in = write & writeval[1];
    assign r2_in = write & writeval[2];
    assign r3_in = write & writeval[3];
    assign r4_in = write & writeval[4];
    assign r5_in = write & writeval[5];
    assign r6_in = write & writeval[6];
    assign r7_in = write & writeval[7];

    //output values of the load enable registers
    wire [15:0] R0;
    wire [15:0] R1;
    wire [15:0] R2;
    wire [15:0] R3;
    wire [15:0] R4;
    wire [15:0] R5;
    wire [15:0] R6;
    wire [15:0] R7;

    //instantiate load enable registers
    vDFFE #(16) E0(clk, r0_in, data_in, R0);
    vDFFE #(16) E1(clk, r1_in, data_in, R1);
    vDFFE #(16) E2(clk, r2_in, data_in, R2);
    vDFFE #(16) E3(clk, r3_in, data_in, R3);
    vDFFE #(16) E4(clk, r4_in, data_in, R4);
    vDFFE #(16) E5(clk, r5_in, data_in, R5);
    vDFFE #(16) E6(clk, r6_in, data_in, R6);
    vDFFE #(16) E7(clk, r7_in, data_in, R7);

    wire [7:0] readval; //one hot code of register to be read

    Dec U2(readnum, readval); //3:8 decoder

    //16 bit mux to select correct register
    Mux8a #(16) RegSelect (R7,R6,R5,R4,R3,R2,R1,R0,readval,data_out);
endmodule
