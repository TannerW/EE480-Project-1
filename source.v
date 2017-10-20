// file: source.v
// EE 480 Assignment 2
// Authors: Charles Vanderpool

`define word		[15:0]
`define halfword	[7:0]
`define opcode      [15:12]
`define dest        [11:8]
`define src         [7:4]
`define Tsrc        [3:0]
`define regsize		[15:0]
`define memsize 	[65535:0]
`define width		16;
`define aluc		[2:0]
`define regc		[1:0]
`define regsel		[3:0]
`define immed		[15:0]

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
module processor(halt, reset, clk);
	output halt;
	input reset, clk;
	wire halt_o;
	wire `aluc alu_c;//control signals
	wire `regc A_c, Y_c, mdr_c,pc_c, ir_c, mar_c, cond_c;
	wire `regc reg_c;
	wire `regsel reg_sel;
	wire `word  reg_out, reg_in, alu_out, alu_a, alu_b, Y_out, mdr_d, mar_out, mdr_out, mem_out, pc_out, ir_out;
	wire `halfword alu_cond, cond_out;
	wire regRead, YtoBus, MDRtoBus, MARtoBus, PCtoBus, IRtoBus, memWrite, memRead;
	wire `word databus;
	wire [47:0] controls;
	//reg `word regfile `regsize;
	//reg `word mainMem `memsize;
	assign halt = halt_o;
	
//	always@ (reset) begin
//		databus <= 16'b0;
//		pc_c <= 1;
//		halt <= 0;
//		start <= 1;
//	end
	
	//control
			// 	[47] [45]  [43:41]   [40]	[39:38] [37:36] [35:34] [33:32] [31:30] [29:26] [25:24] [23]    [22]      [21]      [20]     [19]     [18]     [17]     [16]      [15:0]
			//{ir_c ,pc_c,alu_c, CONSTtoBus ,A_c,   Y_c,    mdr_c,  mar_c,  cond_c, regsel,  regc,  YtoBus, MDRtoBus, MARtoBus, PCtoBus, IRtoBus, regRead, memRead, memWrite, constant}
	assign databus = controls[40] ? controls[15:0] : (controls[18] ? reg_out : (controls[23] ? Y_out: (controls[22] ?  mdr_out : (controls[21] ? mar_out : (controls[20] ? pc_out : (controls[19] ? ir_out : 16'bz))))));
	assign mdr_d = controls[17] ? mem_out : databus;
	oracle oracle(clk, reset, halt_o, pc_out, ir_out, cond_out, controls);//(clk, start, reset, halt, pc, ir, cond, controlOut)
	//bufif1(databus, controls[15:0], controls[40]);
	
	//regfile
	regfile regile(clk, reg_out, reg_in, controls[25:24], controls[29:26]);
	//bufif1 regfileBus(databus, reg_out, controls[18]);
	
	//alu
	alu alu(alu_out, alu_a, databus, controls[43:41], alu_cond);
	
	//aux registers
	lss_reg A(clk, alu_a, databus, controls[39:38]);
	lss_reg Y(clk, Y_out, alu_out, controls[37:36]);
	//bufif1 yBus(databus, Y_out, YtoBus);
	lss_reg cond(clk, cond_out, alu_cond, controls[31:30]);//condition register
	
	lss_reg	mdr(clk, mdr_out, mdr_d, controls[35:34]);//(clk, out, in, c)
	//bufif1 mdrreg(databus, mdr_out, MDRtoBus);
	
	lss_reg	mar(clk, mar_out, databus, controls[33:32]);//(clk, out, in, c)
	//bufif1 marreg(mar_out, mar_d, MARtoBus);
	
	
	mainMem mainMem(clk, controls[16], mdr_out, mem_out, mar_out);//(clk, write, datain, dataout, addr);
	//bufif1(databus, mem_out, memRead);
	//bufif1(mem_out, databus, ~memRead);
	
	lss_reg pc(clk, pc_out, databus, controls[45:44]);
	//bufif1(databus, pc_out, PCtoBus);
	
	lss_reg ir(clk, ir_out, databus, controls[47:46]);
	//bufif1(databus, ir_out, IRtoBus);
	
	
endmodule

module oracle(clk, reset, halt, pc, ir, cond, controlOut);
	input clk, reset;
	input `word pc, ir;
	input `halfword cond;
	reg `word constant;
	reg [2:0] alu_c;
	reg [1:0] A_c, Y_c, mdr_c, mar_c, pc_c, ir_c, cond_c, regc;
	reg [3:0] regsel;
	reg regRead, YtoBus, MDRtoBus, MARtoBus, PCtoBus, IRtoBus, memRead, memWrite, CONSTtoBus;
	output reg [47:0] controlOut;
	output reg halt;
	reg [7:0] state, nextstate;
	//					 		   [43:41]   [40]	 [39:38] [37:36] [35:34] [33:32] [31:30] [29:26] [25:24] [23]    [22]      [21]      [20]     [19]     [18]     [17]     [16]      [15:0]
	
	localparam	[7:0]
			   start0 	= 	8'd50   ,
               start1 	= 	8'd51   ,
               start2 	= 	8'd52   ,
               start3 	= 	8'd53   ,
			   ad0      =	8'd0   ,
			   ad1      =	8'd54   ,
			   ad2      =	8'd55  ,
			   an0      =	8'd1   ,
			   an1      =	8'd56   ,
			   an2      =	8'd57 ,
			   or0      =	8'd2  ,
			   or1      =	8'd58  ,
			   or2      =	8'd59  ,
			   no0      =	8'd3  ,
			   no1      =	8'd60  ,
			   eo0      =	8'd4  ,
			   eo1      =	8'd61  ,
			   eo2      =	8'd62  ,
			   mi0      =	8'd5  ,
			   mi1      =	8'd63  ,
			   mi2      =	8'd64  ,
			   mi3      =	8'd65  ,
			   sr0      =	8'd6  ,
			   sr1      =	8'd66  ,
			   sr2      =	8'd67  ,
			   sr3      =	8'd68  ,
			   br0      =	8'd10  ,
			   br1      =	8'd69  ,
			   br2      =	8'd70  ,
			   br3      =	8'd71  ,
			   br4      =	8'd72  ,
			   br5      =	8'd73  ,
			   br6      =	8'd74  ,
			   br7      =	8'd75  ,
			   br8      =	8'd76  ,
			   br9      =	8'd77  ,
			  // br10     =	8'd78  ,
			  // br11     =	8'd79  ,
			   jr0      =	8'd11  ,
			   jr1      =	8'd80  ,
			   li0      =	8'd12  ,
			   si0      =	8'd13  ,
			   si1      =	8'd81  ,
			   si2      =	8'd82  ,
			   si3      =	8'd83  ,
			   si4	 	=	8'd89	,
			   si5   	=	8'd90	,
			   si6   	=	8'd91	,
			   co0      =	8'd14  ,
			   co1      =	8'd84  ,
			   lo0      =	8'h2e  ,
			   lo1      =	8'd85  ,
			   lo2      =	8'd86  ,
			   st0      =	8'h4e  ,
			   st1      =	8'd87  ,
			   st2      =	8'd88  ,
			   sy		=	8'd15;
			   
			   
   always@ (posedge clk)begin
	if 	(reset) state <= start0;
	else 		state <= nextstate;
	end
	
	always@ (posedge clk)begin
		case(state)
			start0: nextstate <= start1;
			start1: nextstate <= start2;
			start2: nextstate <= start3;
			start3: begin
						case(ir`opcode)
						0 : nextstate <= `OPad;
						1 : nextstate <= `OPan;
						2 : nextstate <= `OPor;
						3 : nextstate <= `OPno;
						4 : nextstate <= `OPeo;
						5 : nextstate <= `OPmi;
						6 : nextstate <= `OPsy;
						7 : nextstate <= `OPsy;
						8 : nextstate <= `OPsy;
						9 : nextstate <= `OPsr;
						10: nextstate <= `OPbr;
						11: nextstate <= `OPjr;
						12: nextstate <= `OPli;
						13: nextstate <= `OPsi;
						14:	begin
								if(ir`Tsrc == 1) nextstate <= `OPco;
								else if(ir`Tsrc == 2) nextstate <= 8'h2e; //{Tsrc, op} in hex
								else if(ir`Tsrc == 4) nextstate <= 8'h4e; //{Tsrc, op}
								else nextstate <= `OPsy;
							end
						default: nextstate <= `OPsy;
						endcase
						
					end
			ad0   :  nextstate <= ad1; 
			an0   :  nextstate <= an1; 
			or0   :  nextstate <= or1;
			no0   :  nextstate <= no1;
			no1   :  nextstate <= start1;
			eo0   :  nextstate <= eo1;
			eo2   :  nextstate <= start1;
			mi0   :  nextstate <= mi1;
			mi3   :  nextstate <= start1;
			sr0   :  nextstate <= sr1;
			sr1   :  nextstate <= sr2;
			sr2   :  nextstate <= sr3;
			sr3   : begin
						if (cond[3]) nextstate <= sr1;
						else nextstate <= start1;
					end
			br0   : begin
						if (cond[ir[10:8]]) nextstate <= br1;
						else nextstate <= start1;
					end
			br9  :  nextstate <= start1;
			jr0   : begin
						if (cond[ir[2:0]]) nextstate <= br1;
						else nextstate <= start1;
					end
			jr1   :  nextstate <= start1;
			li0   :  nextstate <= start1;
			si0   :  nextstate <= si1; 
			si3   :  begin if (cond[3]) nextstate <= si4; else nextstate <= start1;end
			si4	  :	nextstate <= si5;
			si5   :	nextstate <= si6;
			si6   : nextstate <= si0;
			co0   :  nextstate <= co1;
			co1   :  nextstate <= start1;
			lo0   :  nextstate <= lo1; 
			lo2   :  nextstate <= start1;
			st0   :  nextstate <= st1;
			st2   :  nextstate <= start1;
			ad1, ad2, an1, an2, or1, or2, eo1, mi1, mi2,
			br1, br2, br3, br4, br5, br6, br7, br8,
			si1, si2, lo1, st1: nextstate <= nextstate + 1;
			default: nextstate <= `OPsy;
		endcase
	end
	
	always@ (posedge clk) begin
		controlOut <= 0;// add resets to each state for unused control lines (done)
		MARtoBus <= 0; //not used
		case(state)
			start0:  begin 	regsel <= 0; PCtoBus <= 1; A_c <= 1; mar_c <= 1; end
			start1:  begin 	regsel <= 0;constant <= 1; alu_c <= 1; memRead <= 1; Y_c <= 1; end
			start2:  begin 	regsel <= 0;MDRtoBus <= 1; ir_c <= 1; end
			start3:  begin 	regsel <= 0;YtoBus <= 1; pc_c <= 1; end
			//				regsel				regc		regRead			A_c			Y_c			cond_c		YtoBus			alu_c
			ad0   :  begin 	regsel <= ir`src; 	regc <= 0;	regRead <= 1;  	A_c <= 1;																				end
			ad1   :  begin 	regsel <= ir`Tsrc; 				regRead <= 1; 				Y_c <= 1;	cond_c <= 0; 				alu_c <= 0; 						end
			ad2   :  begin 	regsel <= ir`dest;	regc <= 1;														YtoBus <= 1; 										end
			an0   :  begin 	regsel <= ir`src;  	regc <= 0; 	regRead <= 1;	A_c <= 1; 																				end
			an1   :  begin 	regsel <= ir`Tsrc; 				regRead <= 1; 				Y_c <= 1;	cond_c <= 1; 				alu_c <= 1; 						end
			an2   :  begin 	regsel <= ir`dest; 	regc <= 1; 			 											YtoBus <= 1; 										end
			or0   :  begin 	regsel <= ir`src; 	regc <= 0; 	regRead <= 1; 	A_c <= 1; 	 																			end
			or1   :  begin 	regsel <= ir`Tsrc; 				regRead <= 1; 				Y_c <= 1; 	cond_c <= 2; 				alu_c <= 2; 						end
			or2   :  begin 	regsel <= ir`dest; 	regc <= 1; 											 			YtoBus <= 1;										end
			no0   :  begin 	regsel <= ir`src; 				regRead <= 1; 				Y_c <= 1;	cond_c <= 3;  				alu_c <= 3; 						end
			no1   :  begin 	regsel <= ir`dest; 	regc <= 1;				 										YtoBus <= 1;										end
			eo0   :  begin 	regsel <= ir`src; 	regc <= 0;  regRead <= 1; 	A_c <= 1; 	 																			end
			eo1   :  begin 	regsel <= ir`Tsrc; 				regRead <= 1; 				Y_c <= 1; 	cond_c <= 4; 				alu_c <= 4; 						end
			eo2   :  begin 	regsel <= ir`dest; 	regc <= 1;														YtoBus <= 1;										end
			mi0   :  begin	regsel <= 0;									A_c <= 1; 																				end
			mi1   :  begin 	regsel <= ir`src; 				regRead <= 1;				Y_c <= 1;	cond_c <= 3; 				alu_c <= 3; 						end
			mi2   :  begin 	regsel <= 0;												Y_c <= 1; 	cond_c <= 0;YtoBus <= 1;	alu_c <= 0; 						end
			mi3   :  begin 	regsel <= ir`dest; 	regc <= 1;														YtoBus <= 1;										end
						
			sr0   :  begin	regsel <= ir`Tsrc;				regRead <= 1;	A_c <= 1;																				end
			sr1   :  begin	regsel <= ir`src;	regc <= 3;								Y_c <= 1;								alu_c <= 0;		constant <= -16'b1;	end
			sr2   :  begin	regsel <= 0;									A_c <= 1;							YtoBus <= 1;										end
			sr3   :  begin	regsel <= 0;															cond_c <= 1; 								constant <= 44'b0;	end
			br0   :  begin	regsel <= 0;																															end//dummy control assignmnt (this state does nothing while we wait for the condition to checked
			br1   :  begin	regsel <= 0;PCtoBus <= 1;						A_c <= 1;																				end// remove this state
			br2   :  begin	regsel <= 0;												Y_c <= 1;								alu_c <= 0;		constant <= 1; 		end
			br3   :  begin	regsel <= 0;																		YtoBus <= 1;	alu_c <= 3;		; 					end
			br4   :  begin	regsel <= 0;																		mdr_c <= 1;						YtoBus <= 1;		end
			br5   :  begin	regsel <= 0;IRtoBus	<= 1;						A_c <= 1;	Y_c <= 1;																			end
			br6   :  begin	regsel <= 0;constant <= 16'hff; 							Y_c <= 1; 								alu_c <= 1; 						end
			br7   :  begin	regsel <= 0;						A_c <= 1;																		YtoBus <= 1;		end
			br8   :  begin	regsel <= 0;MDRtoBus <= 1;									Y_c <= 1;								alu_c <= 0;		constant <= 16'hff;	end
			br9   :  begin	regsel <= 0;												pc_c <= 1;												YtoBus <= 1;		end
			//br10  :  begin	regsel <= 0;MDRtoBus <= 1; 												Y_c <= 1;								alu_c <= 0;		end
			//br11  :  begin	regsel <= 0;pc_c <= 1;																			YtoBus <= 1;  end
			jr0   :  begin	regsel <= 0;																															end
			jr1   :  begin	regsel <= ir`dest;														pc_c <= 1;									regRead <= 1;  		end
			li0   :  begin	regsel <= ir`dest;														regc <= 1;					constant <= {{8{ir[7]}},ir`immed}; 	end
			si0   :  begin	regsel <= ir`dest;A_c <= 1;  											regc <= 2; 									constant <= 16'b1; 	end	//change to shift by 8 instead of 1(done)
			si1   :  begin	regsel <= 0;												Y_c <= 1;								alu_c <= 0; 	constant <= 16'b1;	end
			si2   :  begin	regsel <= 0;constant <= 16'd8;											cond_c <= 1;													end
			si3   :  begin	regsel <= 0; 																															end
			si4	  :	 begin	regsel <= ir`dest; 				regRead <= 1; 	A_c <= 1;																				end
			si5	  :  begin  regsel <= 0;constant <= (ir`immed & 16'hff);														alu_c <= 4;		Y_c <= 1; 			end
			si6	  :	 begin	regsel <= ir`dest; YtoBus <= 1;  																										end
			co0   :  begin	regsel <= ir`src;				regRead <= 1;	A_c <= 1; 																				end
			co1   :  begin	regsel <= ir`Tsrc;				regRead <= 1;							cond_c <= 1;													end
			lo0   :  begin 	regsel <= ir`src; 				regRead <= 1; 							mar_c <= 1; 													end
			lo1   :  begin 	regsel <= 0;					memRead <= 1; 							mdr_c <= 1; mar_c <= 0;  										end
			lo2   :  begin 	regsel <= ir`dest; 				memRead <= 0; 							mdr_c <= 0; regc <= 1; 						MDRtoBus <= 1;  	end
			st0   :  begin 	regsel <= ir`dest; 				regRead <= 1; 							mdr_c <= 1;  													end
			st1   :  begin 	regsel <= 0;					memWrite <= 1; 							mar_c <= 1; mdr_c <= 0;  										end
			st2   :  begin 	regsel <= 0;					memWrite <= 0; 							mar_c <= 0;  													end
			default: halt <= 1;	// falls here if system call is made or if default is reached
			
		endcase
		controlOut <= {ir_c, pc_c, alu_c, CONSTtoBus ,A_c,   Y_c,    mdr_c,  mar_c,  cond_c, regsel,  regc,  YtoBus, MDRtoBus, MARtoBus, PCtoBus, IRtoBus, regRead, memRead, memWrite, constant};
	end
	
endmodule

//--------------------regfile------------------------
module regfile(clk, out, in, c, sel);
	input clk;
	input `word in; //write data if reg control is set to write
	input [3:0] c, sel; 
	output `word out; 
	reg `word r `regsize;
	
	initial begin
	r[0 ] <= 0;
	r[1 ] <= 0;
	r[2 ] <= 0;
	r[3 ] <= 0;
	r[4 ] <= 0;
	r[5 ] <= 0;
	r[6 ] <= 0;
	r[7 ] <= 0;
	r[8 ] <= 0;
	r[9 ] <= 0;
	r[10] <= 0;
	r[11] <= 0;
	r[12] <= 0;
	r[13] <= 0;
	r[14] <= 0;
	r[15] <= 0;
	end
	
	always@ (posedge clk)begin
		if (sel == 0) r[sel] <=0;
		else begin
			case(c)
			0: r[sel] <= r[sel];
			1: r[sel] <= in;
			2: r[sel] <= r[sel] << 1;
			3: r[sel] <= r[sel] >> 1;
			endcase
		end
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
	output reg `word dataout;
	
	reg `word memory `memsize;
	
	initial $readmemh("C:\Users\CJ\Documents\Seige\stuff\test1.vmem.txt",memory);
	
	always@ (posedge clk) begin
		if (write) memory[addr] <= datain;
		dataout <= memory[addr];
	end
	
endmodule

//-----------------------------------------------------lss_reg---------------------------------

module lss_reg(clk, out, in, c);
	input clk;
	input [1:0] c;
	input `word in;
	output reg `word out;
	
	always@ (posedge clk)begin
		case(c)
		0: out <= out;
		1: out <= in;
		2: out <= out << 1;
		3: out <= out >> 1;
		endcase
	end
endmodule

//module lss_reg_tb;
//  reg [15:0] in;
//  wire [15:0] out;
//  reg reset, clk;
//  reg [1:0] c;
  
//  lss_reg uut(clk, out, in, c, reset);
  
//  always begin
//    clk <= ~clk;
//    #5;
//  end
  
//  initial begin
//    clk <= 0;
//    reset <= 1;
//    c <= 0;
//    #50;
//    reset <= 0;
//    #50;
//    $display("r = %b", out);
//    #50;
    
//    in <= 16'h5555;
//    c <= 1;
//    #50;
//    in <= 0;
//    c <= 0;
//    $display("r = %b", out);
//    #50;
    
//    c <= 2;
//    #50;
//    c <= 0;
//    $display("r = %b", out);
    
//    c <= 3;
//    #50;
//    c <= 0;
//    $display("r = %b", out);
      
//  end
  
//endmodule

//module alu_tb;
//  reg [15:0] a, b;
//  wire [15:0] out;
//  reg [2:0] c;
//  wire [7:0] cond;
  
//  integer i;
  
//  alu uut(out, a, b, c, cond);
  
//  initial begin
//  	a <= 8; b <= 128;
//    for(i = 0; i != 5; i = i + 1) begin
//      c<=i; #10;
//      $display("case %d : out = %b  cond = %b", i, out, cond);
//    end
      
//  end
  
//endmodule

module testbench;
	reg clk, reset;
	wire halt;
	
	processor p(halt, reset, clk);
	
	always begin
		#5 clk <= ~clk;
	end
	
	initial begin
		reset = 0;
		clk = 0;
		#100;
		reset = 1;
		#10;
		reset = 0;
		$dumpfile("results.txt");
		$dumpvars(0, processor);
	end
endmodule

