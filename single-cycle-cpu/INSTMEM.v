
`timescale 1ns / 1ps

module INSTMEM(Addr,Inst);//?????
input[31:0]Addr;
//input InsMemRW;//???'0'?????????????????
output[31:0]Inst;
wire[31:0]Rom[31:0];
assign Rom[5'h00]=32'h20010008;//addi $1,$0,8 $1=8
assign Rom[5'h01]=32'h3402000C;//ori $2,$0,12 $2=12
assign Rom[5'h02]=32'h00221820;//add $3,$1,$2 $3=20
assign Rom[5'h03]=32'h00412022;//sub $4,$2,$1 $4=4
assign Rom[5'h04]=32'h00222824;//and $5,$1,$2
assign Rom[5'h05]=32'h00223025;//or $6,$1,$2
assign Rom[5'h06]=32'h14220002;//bne $1,$2,2
assign Rom[5'h07]=32'hXXXXXXXX;
assign Rom[5'h08]=32'hXXXXXXXX;
assign Rom[5'h09]=32'h10220002;// beq $1,$2,2
assign Rom[5'h0A]=32'h0800000D;// J 0D 
assign Rom[5'h0B]=32'hXXXXXXXX;
assign Rom[5'h0C]=32'hXXXXXXXX;
assign Rom[5'h0D]=32'hAD02000A;// sw $2 10($8) memory[$8+10]=12
assign Rom[5'h0E]=32'h8D04000A;//lw $4 10($8) $4=12
assign Rom[5'h0F]=32'h10440003;//beq $2,$4,3
assign Rom[5'h10]=32'hXXXXXXXX;
assign Rom[5'h11]=32'hXXXXXXXX;
assign Rom[5'h12]=32'hXXXXXXXX;
assign Rom[5'h13]=32'h30470009;//andi $2,9,$7
assign Rom[5'h14]=32'hXXXXXXXX;
assign Rom[5'h15]=32'hXXXXXXXX;
assign Rom[5'h16]=32'hXXXXXXXX;
assign Rom[5'h17]=32'hXXXXXXXX;
assign Rom[5'h18]=32'hXXXXXXXX;
assign Rom[5'h19]=32'hXXXXXXXX;
assign Rom[5'h1A]=32'hXXXXXXXX;
assign Rom[5'h1B]=32'hXXXXXXXX;
assign Rom[5'h1C]=32'hXXXXXXXX;
assign Rom[5'h1D]=32'hXXXXXXXX;
assign Rom[5'h1E]=32'hXXXXXXXX;
assign Rom[5'h1F]=32'hXXXXXXXX;
assign Inst=Rom[Addr[6:2]];
endmodule