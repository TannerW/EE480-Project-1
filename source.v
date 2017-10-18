// file: source.v
// EE 480 Assignment 2
// Authors: Charles Vanderpool

`define word		[15:0]
`define halfword	[7:0]
`define opcode      [15:12]
`define dest        [11:8]
`define src         [7:4]
`define Tsrc        [3:0]
`define state
`define regsize		[15:0]
`define memsize 	[65535:0]
`define width		16;
`define aluc		[2:0]
`define regc		[1:0]
`define regsel		[3:0]

//op codes
`define OPad	4'b0000
`define OPan	4'b0001
`define OPor	4'b0010
`define OPno	4'b0011
`define OPeo	4'b0100
`define OPmi	4'b0101
`define OPal	4'b0110
`define OPdl	4'b0111
`define OPml	4'b1000
`define OPsr	4'b1001
`define OPbr	4'b1010
`define OPjr	4'b1011
`define OPli	4'b1100
`define OPsi	4'b1101
`define OPlo	4'b1110
`define OPcl	4'b1110
`define OPco	4'b1110
`define OPst	4'b1110
`define OPnl	4'b1110
`define OPsy	4'b1111

//module processor(halt, reset, clk)
//	output reg halt;
//	input reset, clk;
//	
//	
//	reg `word regfile `regsize;
//	reg `word mainMem `memsize;
//	
//	
//	
//	
//	
//
//	
//		
//endmodule
module processor(halt, reset, clk)
	output reg halt;
	input reset, clk;
	wire start;
	wire `aluc alu_c,//control signals
	wire `regc A_c, Y_c, mdr_c, mar_c, cond_c;
	wire `regc reg_c;
	wire `regsel reg_sel;
	wire `word databus, reg_out, reg_in, alu_out, alu_a, alu_b, Y_out, mar_d, mdr_d, mar_out, mdr_out, mem_out, pc_out, ir_out;
	wire `halfword alu_cond, cond_out;
	wire regRead, YtoBus, MDRtoBus, MARtoBus, PCtoBus, IRtoBus;
	
	//reg `word regfile `regsize;
	//reg `word mainMem `memsize;
	
	always@ (reset) begin
		pc <= 0;
		halt <= 0;
		start <= 1;
	end
	
	//control
	oracle();
	
	//regfile
	regfile(clk, reg_out, reg_in, regc, regsel);
	tri regfileBus(databus, regRead, reg_out);
	
	//alu
	alu(alu_out, alu_a, alu_b, alu_c, alu_cond);
	
	//aux registers
	lss_reg A(clk, alu_a, databus, A_c);
	lss_reg Y(clk, Y_out, alu_out, Y_c);
	tri yBus(databus, YtoBus, Y_out);
	lss_reg cond(clk, cond_out, alu_cond, cond_c);//condition register
	
	lss_reg	mdr(clk, mdr_out, databus, mdr_c);//(clk, out, in, c)
	tri mdrreg(databus, MDRtoBus, mdr_out);
	
	lss_reg	mar(clk, mar_out, mar_d, mar_c);//(clk, out, in, c)
	tri marreg(mar_out, MARtoBus, mar_d);
	
	
	mainMem(clk, memWrite, mdr_out, mem_out, mar_out);//(clk, write, datain, dataout, addr);
	tri(databus, memRead, mem_out);
	
	lss_reg pc(clk, pc_out, databus, pc_c);
	tri(databus, PCtoBus, pc_out);
	
	lss_reg ir(clk, ir_out, databus, ir_c);
	tri(databus, IRtoBus, ir_out);
	
	
endmodule

module oracle(clk, start, reset, pc, ir, cond, constout);
	input clk, start, reset;
	input `word pc, ir;
	input `halfword cond;
	wire [1:0] A_c, Y_c, mdr_c, mar_c, cond_c;
	wire [3:0] regc, regsel;
	wire regRead, YtoBus, MDRtoBus, MARtoBus, PCtoBus, IRtoBus, memRead, memWrite;
	output reg [25:0] controlOut;
	reg [7:0] state, nextstate;
	
	localparam start0 	= 	8'd0   ,
               start1 	= 	8'd1   ,
               start2 	= 	8'd2   ,
               start3 	= 	8'd3   ,
			   ad0      =	8'd4   ,
			   ad1      =	8'd5   ,
			   ad2      =	8'd6   ,
			   an0      =	8'd7   ,
			   an1      =	8'd8   ,
			   an2      =	8'd9   ,
			   or0      =	8'd10  ,
			   or1      =	8'd11  ,
			   or2      =	8'd12  ,
			   no0      =	8'd13  ,
			   no1      =	8'd14  ,
			   eo0      =	8'd15  ,
			   eo1      =	8'd16  ,
			   eo2      =	8'd17  ,
			   mi0      =	8'd18  ,
			   mi1      =	8'd19  ,
			   mi2      =	8'd20  ,
			   mi3      =	8'd21  ,
			   sr0      =	8'd22  ,
			   sr1      =	8'd23  ,
			   sr2      =	8'd24  ,
			   sr3      =	8'd25  ,
			   br0      =	8'd26  ,
			   br1      =	8'd27  ,
			   br2      =	8'd28  ,
			   br3      =	8'd29  ,
			   br4      =	8'd30  ,
			   br5      =	8'd31  ,
			   br6      =	8'd32  ,
			   br7      =	8'd33  ,
			   br8      =	8'd34  ,
			   br9      =	8'd35  ,
			   br10     =	8'd36  ,
			   br11     =	8'd37  ,
			   jr0      =	8'd38  ,
			   jr1      =	8'd39  ,
			   li0      =	8'd40  ,
			   si0      =	8'd41  ,
			   si1      =	8'd42  ,
			   si2      =	8'd43  ,
			   si3      =	8'd44  ,
			   co0      =	8'd45  ,
			   co1      =	8'd46  ,
			   lo0      =	8'd47  ,
			   lo1      =	8'd48  ,
			   lo2      =	8'd49  ,
			   st0      =	8'd50  ,
			   st1      =	8'd51  ,
			   st2      =	8'd52;
			   
			   
			   always@ (posedge clk);
				if(reset) state <= start0;
				else 		state <= nextstate;
				end
				
				always@ (posedge clk)
					case(state)
						start0: 	
						start1: 	
						start2: 	
						start3: 	
						ad0   :  begin regsel <= ir[src]; regRead <= 1; regc <= 0; A_c <= 1; state <= ; end
						ad1   :  begin regsel <= ir[Tsrc]; regRead <= 1; cond_c <= 0; Y_c <= 1; state <= ; end
						ad2   :  begin YtoBus <= 1; regsel <= ir[dest]; regc <= 1; state <= ; end
						an0   :  begin regsel <= ir[src]; regRead <= 1; regc <= 0; A_c <= 1; state <= ; end
						an1   :  begin regsel <= ir[Tsrc]; regRead <= 1; cond_c <= 1; Y_c <= 1; state <= ; end
						an2   :  begin YtoBus <= 1; regsel <= ir[dest]; regc <= 1; state <= ; end
						or0   :  begin regsel <= ir[src]; regRead <= 1; regc <= 0; A_c <= 1; state <= ; end
						or1   :  begin regsel <= ir[Tsrc]; regRead <= 1; cond_c <= 2; Y_c <= 1; state <= ; end
						or2   :  begin YtoBus <= 1; regsel <= ir[dest]; regc <= 1; state <= ; end
						no0   :  begin regsel <= ir[src]; regRead <= 1; cond_c <= 3; Y_c <= 1; state <= ; end
						no1   :  begin YtoBus <= 1; regsel <= ir[dest]; regc <= 1; state <= ; end
						eo0   :  begin regsel <= ir[src]; regRead <= 1; regc <= 0; A_c <= 1; state <= ; end
						eo1   :  begin regsel <= ir[Tsrc]; regRead <= 1; cond_c <= 4; Y_c <= 1; state <= ; end
						eo2   :  begin YtoBus <= 1; regsel <= ir[dest]; regc <= 1; state <= ; end
						mi0   :  const(1); A_c <= 1; state <= ; end
						mi1   :  begin regsel <= ir[src]; regRead <= 1; cond_c <= 3; Y_c <= 1; state <= ; end
						mi2   :  begin YtoBus <= 1; cond_c <= 0; Y_c <= 1; state <= ; end
						mi3   :  begin YtoBus <= 1; regsel <= ir[dest]; regc <= 1; state <= ; end
						sr0   :  
						sr1   :  
						sr2   :  
						sr3   :  
						br0   :  
						br1   :  
						br2   :  
						br3   :  
						br4   :  
						br5   :  
						br6   :  
						br7   :  
						br8   :  
						br9   :  
						br10  :  
						br11  :  
						jr0   :  
						jr1   :  
						li0   :  
						si0   :  
						si1   :  
						si2   :  
						si3   :  
						co0   :  
						co1   :  
						lo0   :  begin regsel <= ir[src]; regRead <= 1; mar_c <= 1; state <= ; end
						lo1   :  begin memRead <= 1; mdr_c <= 1; mar_c <= 0; state <= ; end
						lo2   :  begin memRead <= 0; mdr_c <= 0; regsel <= ir[dest]; regc <= 1; MDRtoBus <= 1; state <= ; end
						st0   :  begin regsel <= ir[dest]; regRead <= 1; mdr_c <= 1; state <= ; end
						st1   :  begin memWrite <= 1; mar_c <= 1; mdr_c <= 0; state <= ; end
						st2   :  begin memWrite <= 0; mar_c <= 0; state <= ; end
						
						
						
						
						
						
						
						
						
					endcase
				end
	assign controlOut = {};
endmodule

//--------------------regfile------------------------
module regfile(clk, out, in, c, sel);
	input clk;
	input `word in; //write data if reg control is set to write
	input [3:0] c, sel; 
	output `word out; 
	reg `word r `regsize;
	
	always@ (posedge clk)begin
		case(regc)
		0: r[sel] <= r[sel];
		1: r[sel] <= in;
		2: r[sel] <= r[sel] << 1;
		3: r[sel] <= r[sel] >> 1;
	endcase
	end
	assign out = r[sel];
	
endmodule

//----------------------ALU---------------------------
module alu(out, a, b, c, cond);
	input `word a, b;
	input [2:0] c;
	output reg `word out;
	output reg [7:0] cond;
	
	always@ (c) begin
		case(c)
			0: out = a + b;
			1: out = a & b;
			2: out = a | b;
			3: out = ~b;
			4: out = a ^ b;
		endcase
	end
	
	always@ (a, b)begin
		if (a>b) cond = 8'b00001111; // f lt le eq ne ge gt t
		else if (b>a) cond = 8'b01101001;
		else	cond = 00010001;
	end
endmodule

//-----------------------------------------------------control logic---------------------------


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

-----------------------------------------------------lss_reg---------------------------------

module lss_reg(clk, out, in, c)
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

module lss_reg_tb;
  reg [15:0] in;
  wire [15:0] out;
  reg reset, clk;
  reg [1:0] c;
  
  lss_reg uut(clk, out, in, c, reset);
  
  always begin
    clk <= ~clk;
    #5;
  end
  
  initial begin
    clk <= 0;
    reset <= 1;
    c <= 0;
    #50;
    reset <= 0;
    #50;
    $display("r = %b", out);
    #50;
    
    in <= 16'h5555;
    c <= 1;
    #50;
    in <= 0;
    c <= 0;
    $display("r = %b", out);
    #50;
    
    c <= 2;
    #50;
    c <= 0;
    $display("r = %b", out);
    
    c <= 3;
    #50;
    c <= 0;
    $display("r = %b", out);
      
  end
  
endmodule

module alu_tb;
  reg [15:0] a, b;
  wire [15:0] out;
  reg [2:0] c;
  wire [7:0] cond;
  
  integer i;
  
  alu uut(out, a, b, c, cond);
  
  initial begin
  	a <= 8; b <= 128;
    for(i = 0; i != 5; i = i + 1) begin
      c<=i; #10;
      $display("case %d : out = %b  cond = %b", i, out, cond);
    end
      
  end
  
endmodule
