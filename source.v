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
module n_bit_alu_struct(x, y, c, out, v, z);

	parameter n = 4; // alu bitwidth
	input logic [n-1:0] x, y;
	input logic [2:0] c;
	output logic [n-1:0] out;
	output logic z, v;
	wire [n-1:0] cout, z_inter;
	
	assign z = &z_inter;
	
	bit_slice_lsb_alu_struct b0(x[0], y[0], c, cout[0], z_inter[0], out[0]);
	bit_slice_msb_alu_struct bn(x[n-1], y[n-1], cout[n-2], c, v, z_inter[n-1], out[n-1]);
	
	genvar i;
	generate
		for (i=1; i!=n-1 ; i = i+1 )
		begin : b
			bit_slice_alu_struct alu(x[i], y[i], cout[i-1], c, cout[i], z_inter[i], out[i]);
		end 
	endgenerate 
	
endmodule
//-----------------------------------------------------------------
module bit_slice_alu_struct(x, y, cin, c, cout, z, out); // intermediate bit with cout output

	input logic x, y, cin;
	input logic [2:0] c;
	output cout, z, out;
	wire a, b, s, and_out, or_out, comp_out;
	
	input_prep_rtl input_prep(x, y, c, a, b);
	arith_functions_struct funcs(a, b, cin, s, cout, c, and_out, or_out, comp_out);
	out_sel outsel(s, and_out, or_out, comp_out, c, out);	// (s, and_out, or_out, not_out, c, out);
	
	assign z = !out;

endmodule
//-----------------------------------------------------------------------------
module bit_slice_msb_alu_struct(x, y, cin, c, v, z, out); // most signigant bit with v output

	input logic x, y, cin;
	input logic [2:0] c;
	output v, z, out;
	wire a, b, s, and_out, or_out, comp_out, cout;
	
	input_prep_rtl input_prep(x, y, c, a, b);
	arith_functions_struct funcs(a, b, cin, s, cout, c, and_out, or_out, comp_out);
	out_sel outsel(s, and_out, or_out, comp_out, c, out);	// (s, and_out, or_out, not_out, c, out);
	
	assign z = !out;
	assign v = cout & !(c[1] & c[2]);

endmodule

module bit_slice_lsb_alu_struct(x, y, c, cout, z, out); // least signifigant bit with cout output

	input logic x, y;
	input logic [2:0] c;
	output cout, z, out;
	wire a, b, s, and_out, or_out, comp_out;
	
	input_prep_rtl input_prep(x, y, c, a, b);
	arith_functions_struct funcs(a, b, c[0], s, cout, c, and_out, or_out, comp_out);
	out_sel outsel(s, and_out, or_out, comp_out, c, out);	// (s, and_out, or_out, not_out, c, out);
	
	assign z = !out;

endmodule
//---------------
module input_prep_rtl(a, b,c, a_prime, b_prime);

	input logic a, b;
	input logic [2:0] c;
	output a_prime, b_prime;
	wire logic c_a, c_b;
	
	assign c_a = c[0] & c[1] & c[2];
	assign c_b = c[0] | (c[1] & c[2]);
	xor x0(a_prime, a, c_a);
	xor x1(b_prime, b, c_b);
	

endmodule
//--------------
module arith_functions_struct(a, b, cin, s, cout, c, and_out, or_out, comp_out);

	input logic a, b, cin;
	input logic [2:0] c;
	output and_out, or_out, comp_out, s, cout;
	wire logic mux_o;
	
	
	mux_2x1_g mux0({a,b}, c[0], comp_out);
	bfa_1b_beh bfa(a, b, cin, s, cout);
	and a0(and_out, a, b);
	or or_0(or_out, a, b);
	//not inv_0(comp_out, mux_o);
	
	
endmodule
//------------------
module out_sel(s, and_out, or_out, not_out, c, out);

	input logic s, and_out, or_out, not_out;
	input logic [2:0] c;
	output logic out;

	wire logic c_1_2_or, c_1_2_and, mux1_out, mux2_out;
	
	or n0(c_1_2_or, c[2], c[1]);
	and a0(c_1_2_and, c[1], c[2]);
	
	mux_2x1_g mux0({mux1_out, s}, c_1_2_or, out);
	mux_2x1_g mux1({not_out, mux2_out}, c_1_2_and, mux1_out);
	mux_2x1_g mux2({or_out, and_out}, c[1], mux2_out);

endmodule

//-----------------------------------------------------control logic---------------------------
module control();
	
endmodule

//-----------------------------------------------------main mem--------------------------------
module mainMem();

endmodule

//-----------------------------------------------------lss_reg---------------------------------
module nbit_lss_struct #(parameter n = 4) (clk, in, lssc, arithnolog, set, clr, out);

	input 				clk, set, clr,arithnolog;
	input 	[n-1:0] 	in;
	input 	[1:0] 		lssc;
	output	[n-1:0] 	out;
	logic 				left_end, right_end;

	assign left_end = arithnolog ? out[n-1]:0;
	assign right_end = arithnolog ? out[0]:0;
	
	//mux2x1_rtl ({})
	
	genvar i;
	generate
		for (i = 1; i != n-1; i = i + 1)
		begin: b
			lss_bitslice_struct lss_reg(clk, out[i-1], out[i+1], in[i], lssc, set, clr, out[i]);
		end
	endgenerate 
	
	lss_bitslice_struct b0(clk, left_end, out[1], in[0], lssc, set, clr, out[0]);
	lss_bitslice_struct bn(clk, out[n-2], right_end, in[n-1], lssc, set, clr, out[n-1]);
	

endmodule

module mux4x1_rtl(in, sel, out);

    input	[3:0] 	in;
    input	[1:0] 	sel;
    output 			out;
    
    assign out = sel[1] ? (sel[0] ? in[3]:in[2]) : (sel[0] ? in[1]:in[0]);
    
    
endmodule


module lss_bitslice_struct(clk, sl_in, sr_in, in, lssc, set, clr, out);

	input			clk, in, sl_in, sr_in, set, clr;
	input	[1:0] 	lssc;
	output			out;
	
	wire mux_o, dff_o, dff_o_cmp;
	
	mux4x1_rtl funcSel({sr_in, dff_o, in, sl_in}, lssc, mux_o);
	
	dff_beh dff(clk, mux_o, set, clr, dff_o, dff_o_cmp);
	
	assign out = dff_o;

endmodule

module dff_beh(clk, d, set, clr, q, q_not);

	input	clk, d, set, clr;
	output	q, q_not;
	
	assign q_not = ~q;
	
	always@ (posedge clk) begin
		if(set == 1)		q <= 1;
		else if(clr == 1) 	q <= 0;
		else 				q <= d;
	end

endmodule

