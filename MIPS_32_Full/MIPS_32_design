module InstructionMemory
#(parameter AWIDTH = 32, DWIDTH = 32)(
input wire [AWIDTH-1:0] PC,
output reg [DWIDTH-1:0] instruction
);

//instruction memory 4KB , and every instruction 4 bytes --> memory has 1024 instruction each 32 bits
  reg [DWIDTH-1:0] mem[0 : 1023];
  always@(*)begin  
  instruction =  mem[PC/4];
  end
endmodule

//======================================
module ALU(
  input wire[31:0] op1,
  input wire[31:0] op2,
  input wire[3:0] ALU_operation,
  output reg[31:0] ALU_result,
  output reg Zero
);
  always@(*)begin

    case(ALU_operation)
      4'b0010: ALU_result <= op1 + op2;
      4'b0110: ALU_result <= op1 - op2;
      default: ALU_result <= 33'bx;    
    endcase

    if((op1 == op2)) Zero <=  1'b1;
    else Zero <=  1'b0;
    
  end
endmodule

//========================================
  module Add_PC (
  input  wire [31:0] PC_in,
  output reg [31:0] PC_out
   );
   
   wire clk;
   wire start; 
   reg [31:0] PC_initial;

   always@(posedge start)begin
    PC_out = PC_initial;
    end

   always@(posedge clk)begin
   PC_out = PC_in + 4 ;
   end
endmodule
//========================================
module RegisterFile(
  input wire [4:0] readRegister1,
  input wire [4:0] readRegister2,
  input wire [4:0] writeRegister,
  input wire [31 : 0] writeData,
  input wire regWrite,
  output wire [31 : 0] readData1,
  output wire [31 : 0] readData2
);
  wire clk;
  reg [31:0] readRegister1values;
  reg [31:0] readRegister2values;
  reg [31:0] register [31:0];
  always@(readRegister1 or readRegister2)begin
    register[0] <=  32'h00000000;
  end
    
  assign readData1 = readRegister1values;
  assign readData2 = readRegister2values;
  

    always@(*)begin
  readRegister1values <=  register[readRegister1];
  readRegister2values <=  register[readRegister2];
  end


  always@(posedge clk)begin
    register[writeRegister] =  writeData;
  end 

endmodule
//========================================
module Control(
    input wire[5:0] opCode,
    output reg [1:0] ALUOp,
    output reg RegDst,
    output reg branch,
    output reg branchN,
    output reg MemRead,
    output reg MemtoReg,
    output reg MemWrite,
    output reg ALUSrc,
    output reg RegWrite
  );
  parameter R_format = 6'd0;
  parameter lw = 6'd35;
  parameter sw = 6'd43;
  parameter beq = 6'd4;
  parameter bne = 6'd5;
  parameter addi = 6'd8;



  always@(*)
  begin
    case(opCode)
      R_format:begin
        RegDst <= 1'b1;
        ALUSrc <= 1'b0;
        MemtoReg <= 1'b0;
        RegWrite <= 1'b1;
        MemRead <=  1'b0;
        MemWrite <= 1'b0;
        branch <= 1'b0;
        branchN <= 1'b0;
        ALUOp <= 2'b10;
      end
      lw: begin
        RegDst <= 1'b0;
        ALUSrc <= 1'b1;
        MemtoReg <= 1'b1;
        RegWrite <= 1'b1;
        MemRead <= 1'b1;
        MemWrite <= 1'b0;
        branch <= 1'b0;
        branchN <= 1'b0;
       ALUOp <= 2'b00;
      end
      sw: begin
        RegDst <= 1'bx;
        ALUSrc <= 1'b1;
        MemtoReg <= 1'bx;
        RegWrite <= 1'b0;
        MemRead <= 1'b0;
        MemWrite <= 1'b1;
        branch <= 1'b0;
        branchN <= 1'b0;
        ALUOp <= 2'b00;
      end
      beq: begin
        RegDst <= 1'bx;
        ALUSrc <= 1'b0;
        MemtoReg <= 1'bx;
        RegWrite <= 1'b0;
        MemRead <= 1'b0;
        MemWrite <= 1'b0;
        branch <= 1'b1;
        branchN <= 1'b0;
        ALUOp <= 2'b01;
      end
     bne: begin
       RegDst <= 1'bx;
       ALUSrc <= 1'b0;
       MemtoReg <= 1'bx;
       RegWrite <= 1'b0;
       MemRead <= 1'b0;
       MemWrite <= 1'b0;
       branch <= 1'b0;
       branchN <= 1'b1;
       ALUOp <= 2'b01;
     end
    addi: begin
        RegDst <= 1'b0;
        ALUSrc <= 1'b1;
        MemtoReg <= 1'b0;
        RegWrite <= 1'b1;
        MemRead <= 1'b0;
        MemWrite <= 1'b0;
        branch <= 1'b0;
        branchN <= 1'b0;
        ALUOp <= 2'b00;

    end
      endcase
  end

endmodule
//========================================
module DataMemory
#(parameter AWIDTH = 32, DWIDTH = 32)(
input wire [AWIDTH-1:0] addr,
input wire [AWIDTH-1:0] writeData,
input wire memWrite,
input wire memRead,  
output reg [DWIDTH-1:0] readData
);

//Data memory 16KB , and every address is 4 bytes --> memory has 4096 address each 32 bits
  reg [DWIDTH-1:0] mem[0 : 4095];
  reg dataWriteAddr[DWIDTH-1 : 0];
  
  
   always@(*)begin
     if(memRead)
       readData =  mem[addr/4];
   end
  
   always@(*)begin
     if(memWrite)
       mem[addr/4] =  writeData;
    end
  

endmodule
//========================================
module Mux2to1_32bit(
  input wire[31:0] in0,
  input wire[31:0] in1,
  input wire sel,
  output wire[31:0]out
);
  assign out = (sel == 0)? in0 : in1;
endmodule
//========================================
module SignExtend(
  input wire [15:0] in,
  output wire [31:0] out
);
  assign out = { {16{in[15]}} , in};
endmodule
//========================================
module BranchAdder(
input wire[31 : 0] PC,
input wire[31 : 0] addr, // Number of instructions we want to jump to
output wire[31 : 0] new_PC
);

assign new_PC = PC + (addr << 2);

endmodule
//========================================
module ALUcontrol(
input wire[1:0] ALUOp,
 input wire [31:0]instruction,  
output reg [3:0] ALU_operation
);

  parameter add = 4'b0010;
  parameter sub = 4'b0110;
  
  always@(*)begin
   case(ALUOp)
    2'b00: ALU_operation = add;
    2'b01: ALU_operation = sub; 
     2'b10: ALU_operation = (instruction[5:0] == 6'b100000)? add : sub;
   endcase
   end

endmodule
//========================================
module Mux2to1_5bit(
  input wire[4:0] in0,
  input wire[4:0] in1,
  input wire sel,
  output wire[4:0]out
);
  assign out = (sel == 0)? in0 : in1;
endmodule
//========================================



















//========================================
module MIPS_32_bit();
  
  wire[31:0] instruction;
  wire RegDst;
  wire regWrite;
  wire [4:0] mux_inst_regFile_out;
  wire [31:0] mux_MemtoReg_out;
  wire [31 : 0] readData1_out; 
  wire [31 : 0] readData2_out; 
  wire [3:0] ALUcontrol_out;
  wire [1:0] ALUOp;
  wire [31 : 0] ALU_result_out;
  wire zeroFlag;
  wire branch;
  wire branchN;
  wire memRead;
  wire memWrite;
  wire MemtoReg;
  wire ALUSrc;
  wire[31:0] readDataMem;
  wire[31:0] signExtend_out;
  wire[31:0] mux_regFile_alu_out;
  wire[31:0] branchAdder_out;
  wire[31:0] PC_in;
  wire[31:0] PC_out;
  wire[31:0] PC_mux_out;
  wire[31:0] PC_PLUS_4;
  wire mux_branch_PCAdder_sel;

  assign mux_branch_PCAdder_sel = (zeroFlag && branch) | ((!zeroFlag) && branchN);
  
  Add_PC add_pc(.PC_in(PC_mux_out), .PC_out(PC_out));
  InstructionMemory instMem(.PC(PC_out) , .instruction(instruction));
  RegisterFile registerFile(.regWrite(regWrite) , .readRegister1(instruction[25:21]) , .readRegister2(instruction[20:16]) , .writeRegister(mux_inst_regFile_out), .writeData(mux_MemtoReg_out) , .readData1(readData1_out) , .readData2(readData2_out));
  ALU alu(.op1(readData1_out) , .op2(mux_regFile_alu_out) , .ALU_operation(ALUcontrol_out) , .ALU_result(ALU_result_out) , .Zero(zeroFlag));
  ALUcontrol aluControl(.ALUOp(ALUOp) , .ALU_operation(ALUcontrol_out) , .instruction(instruction));
  DataMemory dataMemory(.addr(ALU_result_out) , .writeData(readData2_out) , .memRead(memRead) , .memWrite(memWrite) , .readData(readDataMem));
  Control control(.opCode(instruction[31:26]) , .ALUOp(ALUOp) , .RegDst(RegDst) , .branch(branch) , .branchN(branchN) , .MemRead(memRead) , .MemWrite(memWrite) , .MemtoReg(MemtoReg) , .ALUSrc(ALUSrc) , .RegWrite(regWrite));
  BranchAdder branchAdder(.PC(PC_out) , .addr(signExtend_out) , .new_PC(branchAdder_out));
  Mux2to1_32bit mux_regFile_alu(.in0(readData2_out) , .in1(signExtend_out) , .sel(ALUSrc) ,.out(mux_regFile_alu_out));
  Mux2to1_32bit mux_MemtoReg(.in0(ALU_result_out) , .in1(readDataMem) , .sel(MemtoReg) , .out(mux_MemtoReg_out));
  Mux2to1_32bit mux_branch_PCAdder(.in0(PC_out) , .in1(branchAdder_out) , .sel(mux_branch_PCAdder_sel) , .out(PC_mux_out));
  Mux2to1_5bit mux_inst_regFile(.in0(instruction[20:16]) , .in1(instruction[15:11]) , .sel(RegDst) , .out(mux_inst_regFile_out));
  SignExtend signExtend(.in(instruction[15:0]) , .out(signExtend_out));


endmodule

