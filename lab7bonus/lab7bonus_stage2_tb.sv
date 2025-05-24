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
`define s_pcrd      5'd25    //PC = Rd

`define MNONE       2'b00
`define MREAD       2'b01
`define MWRITE      2'b10

module lab7bonus_stage2_tb;
  reg [3:0] KEY;
  reg [9:0] SW;
  wire [9:0] LEDR; 
  wire [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
  reg err;
  reg CLOCK_50;

  lab7bonus_top DUT(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,CLOCK_50);

  initial forever begin
    CLOCK_50 = 0; #5;
    CLOCK_50 = 1; #5;
  end
  wire break_ = (LEDR[8] == 1'b1);
  initial begin
    err = 0;
    KEY[1] = 1'b0; // reset asserted
    #10; // wait until next falling edge of clock
    KEY[1] = 1'b1; // reset de-asserted, PC still undefined if as in Figure 4

    /*while (~break_) begin
      // Change the following line to wait until your CPU starts to you fetch
      // the next instruction (e.g., IF1 state from Lab 7 or equivalent in
      // your design).  DUT.CPU.FSM is not required for by the autograder
      // for Lab 8. 
      @(posedge (DUT.CPU.FSM.present_state == `s_if1) or posedge break_);  

      @(negedge CLOCK_50); // show advance to negative edge of clock
      $display("PC = %h", DUT.CPU.PC);
    end
    */

    @(negedge CLOCK_50); // wait until next falling edge of clock
    @(negedge CLOCK_50); // wait until next falling edge of clock

    if (DUT.CPU.PC !== 9'b0) begin err = 1; $display("FAILED: PC is not reset to zero."); $stop; end
    $display("PC = %h", DUT.CPU.PC);


    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 1 *before* executing MOV R0, X
    $display("PC = %h", DUT.CPU.PC);

    if (DUT.CPU.PC !== 9'h1) begin err = 1; $display("FAILED: PC should be 1."); $stop; end
    
    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 2 *after* executing MOV R0, X
    $display("PC = %h", DUT.CPU.PC);
    if (DUT.CPU.DP.REGFILE.R1 !== 16'd2) begin err = 1; $display("FAILED: R1 should be 2."); $stop; end

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 2 *after* executing MOV R0, X
    $display("PC = %h", DUT.CPU.PC);
    if (DUT.CPU.DP.REGFILE.R0 !== 16'd7) begin err = 1; $display("FAILED: R0 should be 7."); $stop; end

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 2 *after* executing MOV R0, X
    $display("PC = %h", DUT.CPU.PC);
    $display("N = %h", DUT.CPU.DP.N);
    $display("V = %h", DUT.CPU.DP.V);
    $display("Z = %h", DUT.CPU.DP.Z);
    if (DUT.CPU.DP.N !== 1'b1) begin err = 1; $display("FAILED: N should be 1"); $stop; end
    if (DUT.CPU.DP.V !== 1'b0) begin err = 1; $display("FAILED: V should be 0"); $stop; end
    if (DUT.CPU.DP.Z !== 1'b0) begin err = 1; $display("FAILED: A should be 0"); $stop; end

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);
    $display("PC = %h", DUT.CPU.PC);
    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);
    $display("PC = %h", DUT.CPU.PC);

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);
    $display("PC = %h", DUT.CPU.PC);
    $display("PC = %h", DUT.CPU.DP.REGFILE.R5);
    if (DUT.CPU.DP.REGFILE.R5 !== 16'h8000) begin err = 1; $display("FAILED: R5 should be 0x8000"); $stop; end;

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);
    $display("PC = %h", DUT.CPU.PC);
    $display("N = %h", DUT.CPU.DP.N);
    $display("V = %h", DUT.CPU.DP.V);
    $display("Z = %h", DUT.CPU.DP.Z);
    if (DUT.CPU.DP.N !== 1'b0) begin err = 1; $display("FAILED: N should be 0"); $stop; end
    if (DUT.CPU.DP.V !== 1'b1) begin err = 1; $display("FAILED: V should be 1"); $stop; end
    if (DUT.CPU.DP.Z !== 1'b0) begin err = 1; $display("FAILED: A should be 0"); $stop; end

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);
    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);
    $display("PC = %d", DUT.CPU.PC);
    $display("R5 = %d", DUT.CPU.DP.REGFILE.R5);
    if (DUT.CPU.DP.REGFILE.R5 !== 16'h7E) begin err = 1; $display("FAILED: R5 should be 0x7E."); $stop; end

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);
    $display("PC = %d", DUT.CPU.PC);
    $display("R6 = %d", DUT.CPU.DP.REGFILE.R6);
    if (DUT.CPU.DP.REGFILE.R6 !== 16'd42) begin err = 1; $display("FAILED: R6 should be 42."); $stop; end

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);
    $display("PC = %d", DUT.CPU.PC);
    $display("mem[16'd124] = %d", DUT.MEM.mem[16'd124]);
    if (DUT.MEM.mem[16'd124] !== 16'd42) begin err = 1; $display("FAILED: mem[16'h7C] wrong"); $stop; end

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);
    $display("PC = %d", DUT.CPU.PC);
    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);
    $display("PC = %d", DUT.CPU.PC);
    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);
    $display("PC = %d", DUT.CPU.PC);

    //if (DUT.MEM.mem[25] !== -16'd23) begin err = 1; $display("FAILED: mem[25] wrong"); $stop; end
 
    #500
    if (~err) $display("PASSED");
    $stop;
  end
endmodule
