`define  MNONE   2'b00
`define  MREAD   2'b01
`define  MWRITE  2'b10
//48 cycles

module lab7bonus_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,CLOCK_50);
    input [3:0] KEY;
    input [9:0] SW;
    output [9:0] LEDR;
    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
    input CLOCK_50;

    wire [1:0] mem_cmd;
    wire [8:0] mem_addr;
    wire [15:0] read_data,write_data, dout, in ;


    wire reset, clk, write, comp1, comp2, msel ;

    assign clk = ~KEY[0];
    assign reset = ~KEY[1];

    //instantiate RAM(clk,read_address,write_address,write,din,dout)
    RAM MEM(CLOCK_50,mem_addr[7:0],mem_addr[7:0],write,write_data,dout);

    //instantiate CPU
    cpu CPU(CLOCK_50,reset, /*,s,load,N,V,Z,*/
                mem_cmd, mem_addr, read_data,
                    write_data, LEDR[8]);

    wire enable_dout;


    //16-bit tri-state driver (from lab 7 booklet)
    //if enable = 1, read_data = dout, else zzzz
    assign read_data = enable_dout ? dout : {16{1'bz}};

    //comparator to check `MREAD==mem_cmd
    assign comp1 = (`MREAD==mem_cmd);

    assign msel = (mem_addr[8:8]==1'b0);
    assign enable_dout = comp1&&msel;

    //comparator to check `MREAD==`MWRITE
    assign comp2 = (mem_cmd==`MWRITE);

    assign write = comp2&&msel;


    //LDR LEDs

    wire enable_ldrled,is_memcmd_mread,is_mem_addr_0x140;
    wire [7:0] upper_data, lower_data;

    assign is_memcmd_mread = (mem_cmd==`MREAD);
    assign is_mem_addr_0x140 = (mem_addr==9'h140);

    assign enable_ldrled = (is_memcmd_mread & is_mem_addr_0x140);

    assign upper_data = enable_ldrled ? 8'h00 : {8{1'bz}}; //tri state buffer for upper bits
    assign lower_data = enable_ldrled ? SW[7:0] : {8{1'bz}}; //tri state buffer for lower bits
    assign read_data = { {upper_data}, {lower_data}};


    //STR LEDS

    wire [1:0] is_memcmd_mwrite;
    wire [8:0] is_mem_addr_0x100;
    wire enable_strdff;


    assign is_memcmd_mwrite = (mem_cmd==`MWRITE);
    assign is_mem_addr_0x100 = (mem_addr==9'h100); 

    assign enable_strdff = is_memcmd_mwrite && is_mem_addr_0x100;

    vDFFE #(8) ESTRLED (clk, enable_strdff, write_data[7:0], LEDR[7:0]);
endmodule





// To ensure Quartus uses the embedded MLAB memory blocks inside the Cyclone
// V on your DE1-SoC we follow the coding style from in Altera's Quartus II
// Handbook (QII5V1 2015.05.04) in Chapter 12, “Recommended HDL Coding Style”
//
// 1. "Example 12-11: Verilog Single Clock Simple Dual-Port Synchronous RAM 
//     with Old Data Read-During-Write Behavior" 
// 2. "Example 12-29: Verilog HDL RAM Initialized with the readmemb Command"
module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 16; 
  parameter addr_width = 8;
  parameter filename = "data.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @ (posedge clk) begin
    if (write)
      mem[write_address] <= din;
    dout <= mem[read_address]; // dout doesn't get din in this clock cycle 
                               // (this is due to Verilog non-blocking assignment "<=")
  end 
endmodule
