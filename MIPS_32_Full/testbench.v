//Testbench for the whole design

`timescale 1ns/100ps

module MIPS_32_tb();

  
reg clk;
reg start;

assign   DUT.add_pc.clk = clk;
assign   DUT.add_pc.start = start;
assign   DUT.registerFile.clk = clk;

MIPS_32_bit DUT();

initial begin
  clk <= 1'b0;
  forever begin 
   #4 clk = ~clk;
end
end

initial begin
 DUT.add_pc.PC_initial <= 32'b0;
 start <= 1'b0;
 #4
 start <= 1'b1;
end

initial begin

  DUT.instMem.mem[0] <= 32'h00004020 ; //add $t0 , $0 , $0 
  DUT.instMem.mem[1] <= 32'h20090020 ; //addi $t1 , $0 , 32
  DUT.instMem.mem[2] <= 32'h200b0004 ; //addi $t3 , $0 , 4
  DUT.instMem.mem[3] <= 32'h2010000c ; //addi $s0 , $0 , 12
  DUT.instMem.mem[4] <= 32'h00008820 ; //add  $s1 , $0 , $0
  DUT.instMem.mem[5] <= 32'had300004 ; //sw $s0 , 4($t1)
  DUT.instMem.mem[6] <= 32'h020b8022 ; //sub $s0 , $s0 , $t3
  DUT.instMem.mem[7] <= 32'h022b8820 ; //add $s1 , $s1 , $t3
  DUT.instMem.mem[8] <= 32'h1600fffd ; //bne $s0 , $0 , -3 
  DUT.instMem.mem[9] <= 32'h2130000c ; //addi $s0 , $t1 , 12
  DUT.instMem.mem[10] <= 32'h200c002c ; //addi $t4 , $0 , 44
  DUT.instMem.mem[11] <= 32'h120c0002 ; //beq $s0 , $t4 , 2
  DUT.instMem.mem[12] <= 32'h018b6022 ; //sub $t4 , $t4 , $t3
  DUT.instMem.mem[13] <= 32'h162cfffe ; //bne $s1 , $t4 , L3
  DUT.instMem.mem[14] <= 32'h8d320004 ; //lw $s2 , 4($t1)
  DUT.instMem.mem[15] <= 32'had920000 ; //sw $s2 , 0($t4)

end


  
  initial begin 
#170 $finish;
end
  
  initial begin
    #12.1 $display("$t0 = 0x%8.0h  |  time = %0t " , DUT.registerFile.register[8] , $time);
    #8 $display("$t1 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[9] ,$time);
    #8 $display("$t3 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[11] ,$time);    
    #8 $display("$s0 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[16],$time);    
    #8 $display("$s1 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[17] ,$time);    
    #8 $display("memory[9] = 0x%8.0h  |  time = %0t" , DUT.dataMemory.mem[9] ,$time);//
    #8 $display("$s0 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[16] ,$time);    
    #8 $display("$s1 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[17] ,$time);    
    #8 $display("instruction : 0x%8.0h  |  time = %0t" , DUT.instruction ,$time); //bne this displays the instruction we branched to 
    #8 $display("$s0 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[16] ,$time);    
    #8 $display("$s1 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[17] ,$time);
    #8 $display("instruction : 0x%8.0h  |  time = %0t" , DUT.instruction ,$time); //bne this displays the instruction we branched to   
    #8 $display("$s0 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[16],$time);    
    #8 $display("$s1 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[17] ,$time); 
    #8 $display("instruction : 0x%8.0h  |  time = %0t" , DUT.instruction ,$time);//bne this displays the next instruction as the registers were equal
    #8 $display("$s0 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[16] ,$time);    
    #8 $display("$t4 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[12] ,$time); 
    #8 $display("instruction : 0x%8.0h  |  time = %0t" , DUT.instruction ,$time);//beq this displays the instruction we branched to
    #8 $display("$s2 = 0x%8.0h  |  time = %0t" , DUT.registerFile.register[18] ,$time);    
    #8 $display("memory[11] = 0x%8.0h  |  time = %0t" , DUT.dataMemory.mem[11] ,$time);//    

  end
  
  

endmodule


//there is an assumption that PC was 0 at the beggining of simulation, we are not taking into consideration page address we just start counting from the first instruction
//we write on registers @posedge clk , so we see the registers change value exactly at the end of the cycle it should be changed in and the beginning of the next cycle
//all instructions that we write on a register in : add , sub, addi , lw --> will do as i described above , while sw will be executed immediately as it's combinational 
//instructions gets fetched with the posedge for clk , so we don't see what beq and bne until the end of the cycle they're executed in, like: add , sub addi and lw
//we start execution at signal start which is raised #4 from the begining of simulation
