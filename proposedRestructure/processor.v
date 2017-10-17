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

//condition codes
`define fPos [0]
`define ltPos [1]
`define lePos [2]
`define eqPos [3]
`define nePos [4]
`define gePos [5]
`define gtPos [6]
`define tPos [7]

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

module processor(halt, reset, clk);
	output reg halt;
	input reset, clk;
	reg `word pc = 0;
	reg `word ir;
	wire `word ALUOut = 0;
	wire IorD = 0;
	
	//mainmem comm busses
	//input
	reg `word memAddress;
	wire `word memDataIn; //data to write to main mem
	wire memRead = 1;
	wire memWrite = 0;
	//output
	wire `word memDataOut;
	
	//instruction register comm busses
    //input
    //see memDataOut
    //output
    wire [3:0] currentOpcode, currentDst, currentSrc, currentTsrc;
	
	//reg file comm busses
	wire RegDst;
	//input
	wire `word readReg1;
	wire `word readReg2;
	wire `word writeReg;
	wire writeToRegCommand;
	reg `word writeData;
    //output
    wire `word readRegData1;	
	wire `word readRegData2;
	
	//ALU comm busses
	//input
	reg `word a, b;
	wire `opcode ALUControl;
	//output
	wire `word ALUOut;
	
    
	always@(reset) begin
       halt = 0;
       pc = 0;
       memAddress = pc;
       //$readmemh0(regfile);
       //$readmemh1(mainmem);
     end
	
	always @(IorD or ALUOut or pc)
	begin
	   if(IorD == 1)
	       memAddress <= ALUOut;
	    else
	       memAddress <= pc;
	end
	
	controller controller(IorD, memRead, memWrite, MemtoReg, RegDst, RegWriteCommand, ALUSrcA, ALUSrcB, ALUOp, PCSource, clk, ir`opcode);

    always@(PCSource)
    begin
        case(PCSource)
            0: ;//??????????
            1: pc <= ALUOut; //increment PC by 1
            2: ;//?????????
            default: halt = 1;
        endcase  
    end
	
	mainMem mainMem(memDataOut, clk, memAddress, memRead, memWrite, memDataIn);
	IR IR(currentOpcode, currentDst, currentSrc, currentTsrc, clk, memDataOut);
	
	always@(currentOpcode)
	begin
	   if(currentOpcode == `OPsy)
	   begin
	       halt = 1;
	       
	   end
	end
	
	assign readRegData1 = currentSrc;
	assign readRegData2 = currentTsrc;
	assign writeReg = currentDst;
	assign MDR_out = memDataOut; //MDR
	
	//???????????????
//	always@(RegDst)
//	begin
//	   if(RegDst == 1)
//           memAddress = ALUOut;
//        else
//           memAddress = pc;
//	   writeReg
//	end

    always@(MemtoReg)
	begin
	   if(MemtoReg == 1)
           writeData <= MDR_out;
        else
           writeData <= ALUOut;
	end
	
	regFile regfile(readRegData1, readRegData2, reset, clk, readReg1, readReg2, writeReg, RegWriteCommand, writeData);
	
	always@(ALUSrcA)
	begin
	   if(ALUSrcA == 1)
	       a <= readRegData1;
	   else
	       a <= pc;
	end
	
	always@(ALUSrcB)
    begin
       case(ALUSrcB)
            0: b <= readRegData2;
            1: b <= 1; //increment PC by 1
            2: ;//?????
            3: ;//???????
            default: halt = 1;
       endcase
    end
        
        
	alu ALU(ALUOut, clk, a, b, currentOpcode);
	
		
endmodule

//-----------------------------------------------------control logic---------------------------
module controller(IorD, memRead, memWrite, MemtoReg, RegDst, RegWrite, ALUSrcA, ALUSrcB, ALUOp, PCSource, clk, ir`opcode);

    
    
endmodule

//----------------------ALU---------------------------
module alu(ALUOut, clk, a, b, currentOpcode);
	input `word a, b;
	input clk;
	input [3:0] currentOpcode;
	
	output reg `word ALUOut;
	
	reg [7:0] cond = 8'b00000001; // f lt le eq ne ge gt t
	
	always@(clk) begin
	   case(currentOpcode)
	       `OPad: ;
	       `OPan: ;
	       `OPco:
	           begin
	           
	               cond = 8'b00000001;
	               
	               if (a == b)
	               begin
	                   cond`eqPos = 1;
	                   cond`lePos = 1;
	                   cond`gePos = 1;
	               end
	               else
	               begin
	                   if(a > b)
	                   begin
	                       cond`gtPos = 1;
	                       cond`nePos = 1;
	                       cond`gePos = 1;
	                   end
	                   else
	                   begin
	                       if(a < b)
	                       begin
	                           cond`ltPos = 1;
                               cond`nePos = 1;
                               cond`lePos = 1;
	                       end
	                   end
	               end
	           end
	       `OPeo: ;
	       `OPli: ;
	       `OPlo: ;
	       `OPmi: ;
	       `OPno: ;
	       `OPor: ;
	       `OPsi: ;
	       `OPst: ;
	   endcase
	end
	
	always@ (a, b)begin
		if (a>b) cond = 8'b00001111; // f lt le eq ne ge gt t
		else if (b>a) cond = 8'b01101001;
		else	cond = 00010001;
	end
endmodule

//--------------------regfile------------------------
module regfile(readRegData1, readRegData2, reset, clk, readReg1, readReg2, writeReg, RegWriteCommand, writeData);
    input clk, reset;
    input [3:0] readReg1, readReg2, writeReg;
    input RegWriteCommand;
    input `word writeData;
    
    output reg `word readRegData1, readRegData2;
     
    reg `word regfileArray `regsize; //register file
	
	always@(reset) begin
	   //$readmemh0(regfileArray);
    end
	
	always@(clk)
	begin
	   readRegData1 <= regfileArray[readReg1];
	   readRegData2 <= regfileArray[readReg2];
	   if(RegWriteCommand == 1)
            regfileArray[writeReg] <= writeData;
	end
        
    always@(writeReg)
    begin
        if(RegWriteCommand == 1)
            regfileArray[writeReg] = writeData;
    end
	
	
endmodule

//-----------------------------------------------------main mem--------------------------------
module mainMem(memDataOut, reset, clk, memAddress, memRead, memWrite, memDataIn);
	input clk, reset;
	input memRead, memWrite;
	input `word memDataIn;
	input `word memAddress;
	
	output reg `word memDataOut;
	
	reg `word mainmemory `memsize;
	
	always@(reset) begin
           //$readmemh1(mainmemory);
    end
        
	always@(clk)
	begin
	   if(memWrite == 1)
	       mainmemory[memAddress] <= memDataIn;
	   else
	       begin
	           if(memRead == 1)
	               memDataOut <= mainmemory[memAddress];
	       end
	end
	
endmodule

//instruction register
module IR(currentOpcode, currentDst, currentSrc, currentTsrc, clk, memDataOut);
    input `word memDataOut;
    input clk;
    
    output reg [3:0] currentOpcode, currentDst, currentSrc, currentTsrc;
    
    always@(clk)
    begin
        currentOpcode <= memDataOut`opcode;
        currentDst <= memDataOut`dest;
        currentSrc <= memDataOut`src ;
        currentTsrc <= memDataOut`Tsrc;
    end
endmodule
