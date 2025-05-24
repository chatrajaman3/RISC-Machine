//32 different states possible
`define s_reset     5'd0     //RESET
`define s_if1       5'd1     //IF1
`define s_if2       5'd2     //IF1
`define s_pc        5'd3     //UPDATE PC
`define s_decode    5'd4     //decode (NOT USING)
`define s_wx8_rn    5'd5     //WriteImm8Rn
`define s_a_rn      5'd6     //Get A
`define s_b_rm      5'd7     //Get B
`define s_cmp       5'd8     //CMP
`define s_mvnmovrd  5'd9     //MVN/MOV(Rd)
`define s_aa        5'd10    //AND/AND
`define s_w_rd      5'd11    //WriteRd
`define s_add_sx5   5'd12    //ADD (sximm5)
`define s_str_addr  5'd13    //STR ADDR
`define s_w_rd_mdata 5'd14   //WriteRd (mdata)
`define s_get_rd    5'd15    //Get Rd
`define s_str       5'd16    //STR
`define s_halt      5'd17    //HALT
`define s_placeholder 5'd18  //placeholder state
`define s_placeholder2 5'd19 //placeholder state for str
`define s_addpc     5'd20    //PC+1
`define s_addsximm8pc 5'd21  //PC+sximm8
`define s_updatepc2 5'd22    //UPDATE PC 2
`define s_updatepc3 5'd23    //UPDATE PC 3
`define s_w_r7pc    5'd24    //WriteR7_pc
`define s_get_rd_pc 5'd25    //Get_rd_pc
`define s_loadoutpc 5'd26    //load_out
`define s_pcrd      5'd27    //PC=Rd


`define MNONE       2'b00
`define MREAD       2'b01
`define MWRITE      2'b10

module cpu(clk,reset, /*,s,load,N,V,Z,*/
                mem_cmd, mem_addr, read_data,
                    datapath_out, LEDR8);
    input clk, reset /*s, load*/;
    input [15:0] read_data;
    output [15:0] datapath_out;
    output reg LEDR8;
    //output N, V, Z;

    wire [15:0] IR_out; //output of instruction reg

    //wires for instruction decoder i/o
    wire [1:0] ALUop, shift, op;
    wire [15:0] sximm5, sximm8;
    wire [2:0] readnum, writenum, opcode;
    wire [2:0] nsel;

    //wires for FSM controller i/o

    wire loada, load_ir, loadb, loadc, loads, load_pc, write;
    wire [3:0] vsel;

    //wires for datapath i/o
    wire V, N, Z;

    wire addr_sel, load_addr;
    wire [1:0] reset_pc, asel, bsel;

    output [1:0] mem_cmd;
    output [8:0] mem_addr;
    wire [8:0] next_pc, out_p, out_add, PC, out_addr;

    //instantiate instruction register
    vDFFE #(16) IR(clk, load_ir, read_data, IR_out); //(clk, en, in, out)

    //instantiate instruction decoder
    instruction_decoder ID(IR_out, nsel, 
                            opcode, op, ALUop, sximm5, sximm8, shift, readnum, writenum);
    
    //instantiate datapath
    datapath DP(read_data, sximm8, sximm5, vsel, datapath_out,
                writenum, write, readnum, clk,
                loada, asel,
                loadb, shift, bsel,
                ALUop,
                loadc, loads, N, V, Z, PC);

    //instantiate state machine
    state_machine FSM(clk, reset, opcode, op, 
                    nsel, loada, loadb, loadc, loads, asel, bsel, vsel, write,
                        load_ir, mem_cmd, addr_sel, load_pc, reset_pc,
                            load_addr,
                                N, V, Z, readnum, LEDR8);

    //program counter
    vDFFE #(9) PROGCOUNTER(clk, load_pc, next_pc, PC); //(clk, en, in, out)

    //mux for selecting next PC address/reset/datapath_out
    mux2bb #(9) PCSEL( { {6'b0},readnum}, datapath_out[8:0], 9'b000_000_000, out_add, reset_pc, next_pc);

    //add 1 to PC address out
    assign out_add = PC + 1;

    //mux for selecting address
    mux1bb #(9) ADDRSEL(PC, out_addr, addr_sel, mem_addr);

    //Data address
    vDFFE #(9) DATAADDRESS(clk, load_addr, datapath_out[8:0], out_addr); //(clk, en, in, out)

endmodule

module mux1bb(a1,a0,sel,out); //parametrized #()
    parameter k=1;  //width of in/out
    input[k-1:0] a1,a0;
    input sel;
    output reg[k-1:0] out;
    always_comb begin
        case(sel)
        1'b1: out = a1;
        1'b0: out = a0;
        default: out = {k{1'bx}};
        endcase
    end
endmodule


module mux3a(a2,a1,a0,s,b);
    parameter k = 1; //width of input and output
    input [k-1: 0] a0, a1, a2;
    //3 bit ONE HOT select
    input [2:0] s;
    output reg [k-1: 0] b;

    always_comb begin
        case(s)
            3'b001: b = a0;
            3'b010: b = a1;
            3'b100: b = a2;
            default: b = {k{1'bx}};
        endcase
    end
endmodule

module instruction_decoder(IR_out, nsel, 
                            opcode, op, ALUop, sximm5, sximm8, shift, readnum, writenum);
    //declare inputs
    input [2:0] nsel;       //3 bits
    input [15:0] IR_out;    //16 bits

    //declare outputs
    output [1:0] ALUop, shift, op;          //2 bits
    reg [1:0] ALUop, shift;
    output [2:0] readnum, writenum, opcode; //3 bits
    output [15:0] sximm5, sximm8;           //16 bits

    //declare internal signals
    wire[2:0] Rn,Rd,Rm;     //3bits
    wire[4:0] imm5;         //5 bits
    wire[7:0] imm8;         //8 bits

    //assign internal signals
    assign opcode = IR_out[15:13];
    assign op = IR_out[12:11];

    assign Rn =  IR_out[10:8];
    assign Rd =  IR_out[7:5];
    assign Rm =  IR_out[2:0];

    //assign shift =  IR_out[4:3];

    assign imm8 =  IR_out[7:0];
    assign imm5 =  IR_out[4:0];

    always_comb begin
        case(IR_out[15:11])
        5'b01011: ALUop = 2'b00;
        default: ALUop =  IR_out[12:11];
        endcase

        case(IR_out[15:11])
        5'b10000: shift = 00;
        default: shift = IR_out[4:3];
        endcase
    end
    //assign ALUop =  IR_out[12:11];

    //sign extend imm5 & imm8
    assign sximm5 = { { 11{imm5[4]} }, imm5};
    assign sximm8 = { { 8{imm8[7]} }, imm8};

    //3 bit one hot sel mux for Rn/Rd/Rm --> readnum & write num
    mux3a #(3) RMUX(Rn, Rd, Rm, nsel, readnum); //(a2, a1, a0, s, b)
    assign writenum = readnum; //writenum is same as readnum
endmodule


module state_machine (clk, reset, opcode, op, 
                    nsel, loada, loadb, loadc, loads, asel, bsel, vsel, write,
                        load_ir, mem_cmd, addr_sel, load_pc, reset_pc,
                            load_addr,
                                N,V,Z, readnum, LEDR8);
    input reset, clk, N, V, Z;
    input [2:0] opcode;
    input [1:0] op;
    output reg LEDR8;

    output reg loada, loadb, loadc, loads, write, load_ir, 
                load_pc, addr_sel, load_addr;
    output reg [1:0] mem_cmd, asel, bsel, reset_pc;
    output reg[2:0] nsel;
    output reg [3:0] vsel;
    reg N,V,Z;
    input reg [2:0] readnum;
    reg [4:0] present_state;
    wire [4:0] oper = { opcode, op};

    always_comb begin
        case(present_state)
            `s_reset: begin     //Reset
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'bxx;
                bsel = 2'bxx;
                vsel = 4'bxxxx;
                nsel = 3'bxxx;
                write = 1'b0;
                
                reset_pc = 2'b01;
                load_pc = 1;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end

            `s_if1: begin      //IF1
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'bxx;
                bsel = 2'bxx;
                vsel = 4'bxxxx;
                nsel = 3'bxxx;
                write = 1'b0;
                
                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MREAD;
                addr_sel = 1;

                load_addr=0;
                LEDR8=0;
            end

            `s_if2: begin      //IF2
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'bxx;
                bsel = 2'bxx;
                vsel = 4'bxxxx;
                nsel = 3'bxxx;
                write = 1'b0;
                
                reset_pc = 0;
                load_pc = 0;
                load_ir = 1;
                mem_cmd = `MREAD;
                addr_sel = 1;

                load_addr=0;
                LEDR8=0;
            end

            `s_pc: begin       //UpdatePC
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'bxx;
                bsel = 2'bxx;
                vsel = 4'bxxxx;
                nsel = 3'b100;
                write = 1'b0;
                
                reset_pc = 2'b00;
                load_pc = 1;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end



            `s_decode: begin    //decode
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'bxx;
                bsel = 2'bxx;
                vsel = 4'bxxxx;
                nsel = 3'bxxx;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end

            `s_wx8_rn: begin    //WriteRn(sximm8)
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b01;
                bsel = 2'b01;
                vsel = 4'b0100;
                nsel = 3'b100;
                write = 1'b1;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end


            `s_a_rn: begin      //GetA(Rn)
                loada = 1'b1;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b01;
                bsel = 2'b01;
                vsel = 4'bxxxx;
                nsel = 3'b100;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end

            `s_b_rm: begin      //GetB(Rm)
                loada = 1'b0;
                loadb = 1'b1;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b01;
                bsel = 2'b01;
                vsel = 4'bxxxx;
                nsel = 3'b001;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end

            `s_aa: begin        //ADD/AND
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b1;
                loads = 1'b0;
                asel = 2'b00;
                bsel = 2'b00;
                vsel = 4'b0001;
                nsel = 3'b010;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end

            `s_cmp: begin       //CMP
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b1;
                asel = 2'b00;
                bsel = 2'b00;
                vsel = 4'bxxxx;
                nsel = 3'bxxx;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end

            `s_mvnmovrd: begin  //MVN/MOV(Rd)
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b1;
                loads = 1'b0;
                asel = 2'b01;
                bsel = 2'b00;
                vsel = 4'bxxxx;
                nsel = 3'bxxx;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;LEDR8=0;
            end
            
            `s_w_rd: begin      //WriteRd
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b01;
                bsel = 2'b00;
                vsel = 4'b0001;
                nsel = 3'b010;
                write = 1'b1;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;LEDR8=0;
            end

            `s_add_sx5:begin      //ADD (sximm5)
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b1;
                loads = 1'b0;
                asel = 2'b00;
                bsel = 2'b01;
                vsel = 4'b0001;
                nsel = 3'b100;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;LEDR8=0;

            end

            `s_str_addr:begin      //STR ADDR
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b00;
                bsel = 2'b01;
                vsel = 4'b0001;
                nsel = 3'b100;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MREAD;
                addr_sel = 0;

                load_addr=1;LEDR8=0;
            end

            `s_placeholder: begin
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b00;
                bsel = 2'b01;
                vsel = 4'b0001;
                nsel = 3'b100;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MREAD;
                addr_sel = 0;

                load_addr=1;LEDR8=0;
            end

            `s_w_rd_mdata:begin      //WriteRd (mdata)
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b00;
                bsel = 2'b01;
                vsel = 4'b1000;
                nsel = 3'b010;
                write = 1'b1;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MREAD;
                addr_sel = 0;

                load_addr=1;LEDR8=0;
            end

            `s_get_rd:begin         //Get Rd
                loada = 1'b0;
                loadb = 1'b1;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b01;
                bsel = 2'b00;
                vsel = 4'b0001;
                nsel = 3'b010;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 1;

                load_addr=1;LEDR8=0;
            end

            `s_str:begin           //STR
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b1;
                loads = 1'b0;
                asel = 2'b01;
                bsel = 2'b00;
                vsel = 4'b0001;
                nsel = 3'b010;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MWRITE;
                addr_sel = 0;

                load_addr=0;LEDR8=0;
            end

            `s_placeholder2: begin
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b01;
                bsel = 2'b00;
                vsel = 4'b0001;
                nsel = 3'b010;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MWRITE;
                addr_sel = 0;

                load_addr=0;LEDR8=0;
            end

            `s_halt:begin          //HALT
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b01;
                bsel = 2'b01;
                vsel = 4'b0001;
                nsel = 3'b010;
                write = 1'b0;

                reset_pc = 2'b01;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=1;
            end

            `s_addpc: begin
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b10;
                bsel = 2'b10;
                vsel = 4'b0001;
                nsel = 3'b100;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0; //~~ CHANGES
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;LEDR8=0;
            end

            `s_addsximm8pc: begin
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b1;
                loads = 1'b0;
                asel = 2'b10;
                bsel = 2'b10;
                vsel = 4'b0001;
                nsel = 3'b100;
                write = 1'b0;

                reset_pc = 2'b10;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;LEDR8=0;
            end

            `s_updatepc2: begin
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b10;
                bsel = 2'b10;
                vsel = 4'b0001;
                nsel = 3'b100;
                write = 1'b0;

                reset_pc = 2'b10;
                load_pc = 1;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;LEDR8=0;
            end

            `s_updatepc3: begin
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b10;
                bsel = 2'b10;
                vsel = 4'b0001;
                nsel = 3'b100;
                write = 1'b0;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;LEDR8=0;
            end

            `s_w_r7pc: begin
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b10;
                bsel = 2'b10;
                vsel = 4'b0010;
                nsel = 3'b100;
                write = 1'b1;

                reset_pc = 2'b00;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end

            `s_get_rd_pc: begin
                loada = 1'b0;
                loadb = 1'b1;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b01;
                bsel = 2'b00;
                vsel = 4'b0010;
                nsel = 3'b010;
                write = 1'b0;

                reset_pc = 2'b11;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end

            `s_loadoutpc: begin
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b1;
                loads = 1'b0;
                asel = 2'b01;
                bsel = 2'b00;
                vsel = 4'b0010;
                nsel = 3'b010;
                write = 1'b0;

                reset_pc = 2'b10;
                load_pc = 0;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end

            `s_pcrd: begin
                loada = 1'b0;
                loadb = 1'b0;
                loadc = 1'b0;
                loads = 1'b0;
                asel = 2'b10;
                bsel = 2'b10;
                vsel = 4'b0010;
                nsel = 3'b010;
                write = 1'b0;

                reset_pc = 2'b10;
                load_pc = 1;
                load_ir = 0;
                mem_cmd = `MNONE;
                addr_sel = 0;

                load_addr=0;
                LEDR8=0;
            end
            
            default: begin
                loada = 1'bx;
                loadb = 1'bx;
                loadc = 1'bx;
                loads = 1'bx;
                asel = 2'bxx;
                bsel = 2'bxx;
                vsel = 4'bxxxx;
                nsel = 3'bxxx;
                write = 1'bx;

                reset_pc = 2'bxx;
                load_pc = 1'bx;
                load_ir = 1'bx;
                mem_cmd = `MNONE;
                addr_sel = 1'bx;

                load_addr=1'bx;LEDR8=0;
            end
        endcase
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            present_state<=`s_reset;
        end
        else begin
            case(present_state)
            `s_reset: present_state<=`s_if1;
            `s_if1: present_state<= `s_if2;
            `s_if2: present_state<=`s_pc;
            `s_pc: begin  
                    case(oper)
                    5'b11010: present_state = `s_wx8_rn;
                    5'b11000: present_state = `s_b_rm; 
                    5'b10111: present_state = `s_b_rm;
                    5'b10100: present_state = `s_a_rn;
                    5'b10101: present_state = `s_a_rn;
                    5'b10110: present_state = `s_a_rn;
                    5'b11100: present_state = `s_halt;
                    5'b10000: present_state = `s_a_rn;
                    5'b01100: present_state = `s_a_rn;
                    5'b00100: present_state = `s_addpc;
                    5'b01010: present_state = `s_w_r7pc;
                    5'b01011: present_state = `s_w_r7pc;
                    5'b01000: present_state = `s_get_rd_pc;
                    default: present_state = 5'bxxxxx;
                    endcase
            end

            `s_a_rn: begin 
                case(oper)
                5'b01100: present_state = `s_add_sx5;
                5'b10000: present_state = `s_add_sx5;
                default: present_state = `s_b_rm;
                endcase
            end


            `s_b_rm: begin
                case(oper)
                5'b10101: present_state = `s_cmp;
                5'b10111: present_state = `s_mvnmovrd;

                5'b10110: present_state = `s_aa;
                5'b10100: present_state = `s_aa;
                5'b11000: present_state = `s_mvnmovrd;
                default: present_state = 5'bxxxxx;
                endcase
            end
            
            `s_aa: present_state = `s_w_rd;

            `s_mvnmovrd: present_state = `s_w_rd;

            `s_w_rd: present_state = `s_if1;

            `s_wx8_rn: present_state = `s_if1;

            `s_cmp: present_state = `s_if1;

            `s_add_sx5: present_state = `s_str_addr;

            `s_str_addr: begin
                case (oper)
                5'b01100: present_state = `s_placeholder;
                5'b10000: present_state = `s_get_rd;
                default: present_state = {5{1'bx}};
                endcase
            end

            `s_placeholder: present_state = `s_w_rd_mdata;

            `s_get_rd: present_state = `s_str;

            `s_str: present_state = `s_placeholder2;

            `s_placeholder2: present_state = `s_if1;

            `s_halt: present_state = `s_halt;

            `s_w_rd_mdata: present_state = `s_if1;

            `s_addpc: begin
                case(readnum)
                3'b000: present_state = `s_addsximm8pc;
                3'b001: begin
                    if (Z==1) begin
                        present_state = `s_addsximm8pc;
                    end
                    else present_state = `s_updatepc3;
                end
                3'b010: begin
                    if (Z==0) begin
                        present_state = `s_addsximm8pc;
                    end
                    else present_state = `s_updatepc3;
                end
                3'b011: begin
                    if (N!=V) begin
                        present_state = `s_addsximm8pc;
                    end
                    else present_state = `s_updatepc3;
                end
                3'b100: begin
                    if ( (N!=V) || (Z==1)) begin
                        present_state = `s_addsximm8pc;
                    end
                    else present_state = `s_updatepc3;
                end
                default: present_state = `s_updatepc3;
                endcase
            end

            `s_addsximm8pc: present_state = `s_updatepc2;

            `s_updatepc2: present_state = `s_if1;
            `s_updatepc3: present_state = `s_if1;

            `s_w_r7pc: begin
                case(oper)
                5'b01011: present_state = `s_addsximm8pc;
                5'b01010: present_state = `s_get_rd_pc;
                default: present_state = 5'bx;                
                endcase
            end

            `s_get_rd_pc: present_state = `s_loadoutpc;

            `s_loadoutpc: present_state = `s_pcrd;
            
            `s_pcrd: present_state = `s_if1;

            default: present_state = 5'bxxxxx;
            endcase
        end
    end
endmodule