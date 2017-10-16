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
module regfile();

	
	
endmodule

//----------------------ALU---------------------------
module alu(out, a, b, control, condition)
	input `word a, b;
	input [2:0] control;
	output `word out;
	output [7:0] condition;
	
	always@ (control) begin
		case(control)
			0: out = a + b;
			1: out = a & b;
			2: out = a | b;
			3: out = a ^ b;
			4: out = ~a;
		endcase
	end
	
	always@ (a, b)begin
		if (a>b) condition = 8'b00001111; // f lt le eq ne ge gt t
		else if (b>a) condition = 8'b01101001;
		else	condition = 00010001;
	end
endmodule

//-----------------------------------------------------control logic---------------------------
module oracle();
	
endmodule

//-----------------------------------------------------main mem--------------------------------
module mainMem();

endmodule

//-----------------------------------------------------lss_reg---------------------------------
//module nbit_lss_struct #(parameter n = 4) (clk, in, lssc, arithnolog, set, clr, out);
//
//	input 				clk, set, clr,arithnolog;
//	input 	[n-1:0] 	in;
//	input 	[1:0] 		lssc;
//	output	[n-1:0] 	out;
//	logic 				left_end, right_end;
//
//	assign left_end = arithnolog ? out[n-1]:0;
//	assign right_end = arithnolog ? out[0]:0;
//	
//	//mux2x1_rtl ({})
//	
//	genvar i;
//	generate
//		for (i = 1; i != n-1; i = i + 1)
//		begin: b
//			lss_bitslice_struct lss_reg(clk, out[i-1], out[i+1], in[i], lssc, set, clr, out[i]);
//		end
//	endgenerate 
//	
//	lss_bitslice_struct b0(clk, left_end, out[1], in[0], lssc, set, clr, out[0]);
//	lss_bitslice_struct bn(clk, out[n-2], right_end, in[n-1], lssc, set, clr, out[n-1]);
//	
//
//endmodule
//
//module mux4x1_rtl(in, sel, out);
//
//    input	[3:0] 	in;
//    input	[1:0] 	sel;
//    output 			out;
//    
//    assign out = sel[1] ? (sel[0] ? in[3]:in[2]) : (sel[0] ? in[1]:in[0]);
//    
//    
//endmodule
//
//
//module lss_bitslice_struct(clk, sl_in, sr_in, in, lssc, set, clr, out);
//
//	input			clk, in, sl_in, sr_in, set, clr;
//	input	[1:0] 	lssc;
//	output			out;
//	
//	wire mux_o, dff_o, dff_o_cmp;
//	
//	mux4x1_rtl funcSel({sr_in, dff_o, in, sl_in}, lssc, mux_o);
//	
//	dff_beh dff(clk, mux_o, set, clr, dff_o, dff_o_cmp);
//	
//	assign out = dff_o;
//
//endmodule
//
//module dff_beh(clk, d, set, clr, q, q_not);
//
//	input	clk, d, set, clr;
//	output	q, q_not;
//	
//	assign q_not = ~q;
//	
//	always@ (posedge clk) begin
//		if(set == 1)		q <= 1;
//		else if(clr == 1) 	q <= 0;
//		else 				q <= d;
//	end
//
//endmodule

