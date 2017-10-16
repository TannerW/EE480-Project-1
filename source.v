// file: source.v
// EE 480 Assignment 2
// Authors: Charles Vanderpool

`define word		[15:0]
`define opcode
`define dest
`define src
`define state
`define regsize		[15:0]
`define memesize 	[65535:0]
`define width		16;

//op codes
`define OPad
`define OPal
`define OPan
`define OPbr
`define OPcl
`define OPco
`define OPdl
`define OPeo
`define OPjr
`define OPli
`define OPlo
`define OPml
`define OPmi
`define OPnl
`define OPno
`define OPor
`define OPsi
`define OPsr
`define OPst
`define OPsy

module processor(halt, reset, clk)
	output reg halt;
	input reset, clk;
	
	
	
		
endmodule

//--------------------regfile------------------------
module regfile(clk, out, in, regc, regsel);
	input clk;
	input `word in;
	input [3:0] regc, regsel;
	output `word out;
	reg `word register `regsize;
	
	always@ (posedge clk)begin
		case(regc)
		0: register[regsel] <= register[regsel];
		1: register[regsel] <= in;
		2: register[regsel] <= register[regsel] << 1;
		3:register[regsel] <= register[regsel] >> 1;
	endcase
	end
	assign out = register[regsel];
	
endmodule

//----------------------ALU---------------------------
module alu(out, a, b, c, cond)
	input `word a, b;
	input [2:0] cl;
	output `word out;
	output [7:0] cond;
	
	always@ (c) begin
		case(c)
			0: out = a + b;
			1: out = a & b;
			2: out = a | b;
			3: out = a ^ b;
			4: out = ~a;
		endcase
	end
	
	always@ (a, b)begin
		if (a>b) cond = 8'b00001111; // f lt le eq ne ge gt t
		else if (b>a) cond = 8'b01101001;
		else	cond = 00010001;
	end
endmodule

//-----------------------------------------------------control logic---------------------------
module oracle();
	
endmodule

//-----------------------------------------------------main mem--------------------------------
module mainMem(clk, write, datain, dataout, addr);
	input clk, write;
	input `word datain, addr;
	output `word dataout;
	
	reg `word memory `memsize;
	
	always@ (posedge clk) begin
		if (write) memory[addr] <= datain;
		dataout <= memory[addr];
	end
	
endmodule

//-----------------------------------------------------lss_reg---------------------------------

module register(clk, out, in, c)
	input clk;
	input [1:0] c;
	input `word in;
	output `word out;
	
	always@ (posedge clk)begin
		case(c)
		0: out <= out;
		1: out <= in;
		2: out <= out << 1;
		3: out <= out >> 1;
		endcase
	end
endmodule
