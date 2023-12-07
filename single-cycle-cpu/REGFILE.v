
`timescale 1ns / 1ps

module REGFILE(Ra,Rb,D,Wr,We,Clk,Clrn,Qa,Qb);
input [4:0]Ra,Rb,Wr;
input [31:0]D;
input We,Clk,Clrn;
output [31:0]Qa,Qb;
wire [31:0]Y_mux,Q31_reg32,Q30_reg32,Q29_reg32,Q28_reg32,Q27_reg32,Q26_reg32,Q25_reg32,Q24_reg32,Q23_reg32,Q22_reg32,Q21_reg32,Q20_reg32,Q19_reg32,Q18_reg32,Q17_reg32,Q16_reg32,Q15_reg32,Q14_reg32,Q13_reg32,Q12_reg32,Q11_reg32,Q10_reg32,Q9_reg32,Q8_reg32,Q7_reg32,Q6_reg32,Q5_reg32,Q4_reg32,Q3_reg32,Q2_reg32,Q1_reg32,Q0_reg32;

DEC5T32E dec(Wr,We,Y_mux);

REG32  A(D,Y_mux,Clk,Clrn,Q31_reg32,Q30_reg32,Q29_reg32,Q28_reg32,Q27_reg32,Q26_reg32,Q25_reg32,Q24_reg32,Q23_reg32,Q22_reg32,Q21_reg32,Q20_reg32,Q19_reg32,Q18_reg32,Q17_reg32,Q16_reg32,Q15_reg32,Q14_reg32,Q13_reg32,Q12_reg32,Q11_reg32,Q10_reg32,Q9_reg32,Q8_reg32,Q7_reg32,Q6_reg32,Q5_reg32,Q4_reg32,Q3_reg32,Q2_reg32,Q1_reg32,Q0_reg32);

MUX32X32 select1(Q0_reg32,Q1_reg32,Q2_reg32,Q3_reg32,Q4_reg32,Q5_reg32,Q6_reg32,Q7_reg32,Q8_reg32,Q9_reg32,Q10_reg32,Q11_reg32,Q12_reg32,Q13_reg32,Q14_reg32,Q15_reg32,Q16_reg32,Q17_reg32,Q18_reg32,Q19_reg32,Q20_reg32,Q21_reg32,Q22_reg32,Q23_reg32,Q24_reg32,Q25_reg32,Q26_reg32,Q27_reg32,Q28_reg32,Q29_reg32,Q30_reg32,Q31_reg32,Ra,Qa);
MUX32X32 select2(Q0_reg32,Q1_reg32,Q2_reg32,Q3_reg32,Q4_reg32,Q5_reg32,Q6_reg32,Q7_reg32,Q8_reg32,Q9_reg32,Q10_reg32,Q11_reg32,Q12_reg32,Q13_reg32,Q14_reg32,Q15_reg32,Q16_reg32,Q17_reg32,Q18_reg32,Q19_reg32,Q20_reg32,Q21_reg32,Q22_reg32,Q23_reg32,Q24_reg32,Q25_reg32,Q26_reg32,Q27_reg32,Q28_reg32,Q29_reg32,Q30_reg32,Q31_reg32,Rb,Qb);

endmodule

module DEC5T32E(I,En,Y);
input [4:0]I;
input En;
output[31:0]Y;

not not1(en,En);
not not2(i0,I[0]);
not not3(i1,I[1]);
not not4(i2,I[2]);
not not5(i3,I[3]);
not not6(i4,I[4]);
nor nor0(Y[0],I[0],I[1],I[2],I[3],I[4],en);
nor nor1(Y[1],i0,I[1],I[2],I[3],I[4],en);
nor nor2(Y[2],I[0],i1,I[2],I[3],I[4],en);
nor nor3(Y[3],i0,i1,I[2],I[3],I[4],en);
nor nor4(Y[4],I[0],I[1],i2,I[3],I[4],en);
nor nor5(Y[5],i0,I[1],i2,I[3],I[4],en);
nor nor6(Y[6],I[0],i1,i2,I[3],I[4],en);
nor nor7(Y[7],i0,i1,i2,I[3],I[4],en);
nor nor8(Y[8],I[0],I[1],I[2],i3,I[4],en);
nor nor9(Y[9],i0,I[1],I[2],i3,I[4],en);
nor nor10(Y[10],I[0],i1,I[2],i3,I[4],en);
nor nor11(Y[11],i0,i1,I[2],i3,I[4],en);
nor nor12(Y[12],I[0],I[1],i2,i3,I[4],en);
nor nor13(Y[13],i0,I[1],i2,i3,I[4],en);
nor nor14(Y[14],I[0],i1,i2,i3,I[4],en);
nor nor15(Y[15],i0,i1,i2,i3,I[4],en);
nor nor16(Y[16],I[0],I[1],I[2],I[3],i4,en);
nor nor17(Y[17],i0,I[1],I[2],I[3],i4,en);
nor nor18(Y[18],I[0],i1,I[2],I[3],i4,en);
nor nor19(Y[19],i0,i1,I[2],I[3],i4,en);
nor nor20(Y[20],I[0],I[1],i2,I[3],i4,en);
nor nor21(Y[21],i0,I[1],i2,I[3],i4,en);
nor nor22(Y[22],I[0],i1,i2,I[3],i4,en);
nor nor23(Y[23],i0,i1,i2,I[3],i4,en);
nor nor24(Y[24],I[0],I[1],I[2],i3,i4,en);
nor nor25(Y[25],i0,I[1],I[2],i3,i4,en);
nor nor26(Y[26],I[0],i1,I[2],i3,i4,en);
nor nor27(Y[27],i0,i1,I[2],i3,i4,en);
nor nor28(Y[28],I[0],I[1],i2,i3,i4,en);
nor nor29(Y[29],i0,I[1],i2,i3,i4,en);
nor nor30(Y[30],I[0],i1,i2,i3,i4,en);
nor nor31(Y[31],i0,i1,i2,i3,i4,en);

endmodule

module REG32(D,En,Clk,Clrn,Q31,Q30,Q29,Q28,Q27,Q26,Q25,Q24,Q23,Q22,Q21,Q20,Q19,Q18,Q17,Q16,Q15,Q14,Q13,Q12,Q11,Q10,Q9,Q8,Q7,Q6,Q5,Q4,Q3,Q2,Q1,Q0);
input [31:0]D,En;
input Clk,Clrn;
output [31:0]Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,Q17,Q18,Q19,Q20,Q21,Q22,Q23,Q24,Q25,Q26,Q27,Q28,Q29,Q30,Q31;

D_FFEC32 q1(D,Clk,En[1],Clrn,Q1);
D_FFEC32 q2(D,Clk,En[2],Clrn,Q2);
D_FFEC32 q3(D,Clk,En[3],Clrn,Q3);
D_FFEC32 q4(D,Clk,En[4],Clrn,Q4);
D_FFEC32 q5(D,Clk,En[5],Clrn,Q5);
D_FFEC32 q6(D,Clk,En[6],Clrn,Q6);
D_FFEC32 q7(D,Clk,En[7],Clrn,Q7);
D_FFEC32 q8(D,Clk,En[8],Clrn,Q8);
D_FFEC32 q9(D,Clk,En[9],Clrn,Q9);
D_FFEC32 q10(D,Clk,En[10],Clrn,Q10);
D_FFEC32 q11(D,Clk,En[11],Clrn,Q11);
D_FFEC32 q12(D,Clk,En[12],Clrn,Q12);
D_FFEC32 q13(D,Clk,En[13],Clrn,Q13);
D_FFEC32 q14(D,Clk,En[14],Clrn,Q14);
D_FFEC32 q15(D,Clk,En[15],Clrn,Q15);
D_FFEC32 q16(D,Clk,En[16],Clrn,Q16);
D_FFEC32 q17(D,Clk,En[17],Clrn,Q17);
D_FFEC32 q18(D,Clk,En[18],Clrn,Q18);
D_FFEC32 q19(D,Clk,En[19],Clrn,Q19);
D_FFEC32 q20(D,Clk,En[20],Clrn,Q20);
D_FFEC32 q21(D,Clk,En[21],Clrn,Q21);
D_FFEC32 q22(D,Clk,En[22],Clrn,Q22);
D_FFEC32 q23(D,Clk,En[23],Clrn,Q23);
D_FFEC32 q24(D,Clk,En[24],Clrn,Q24);
D_FFEC32 q25(D,Clk,En[25],Clrn,Q25);
D_FFEC32 q26(D,Clk,En[26],Clrn,Q26);
D_FFEC32 q27(D,Clk,En[27],Clrn,Q27);
D_FFEC32 q28(D,Clk,En[28],Clrn,Q28);
D_FFEC32 q29(D,Clk,En[29],Clrn,Q29);
D_FFEC32 q30(D,Clk,En[30],Clrn,Q30);
D_FFEC32 q31(D,Clk,En[31],Clrn,Q31);

assign Q0=0;
endmodule


module MUX32X32(Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,Q17,Q18,Q19,Q20,Q21,Q22,Q23,Q24,Q25,Q26,Q27,Q28,Q29,Q30,Q31,R,Q);//select one register from regfile
input [4:0]R;
input [31:0]Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,Q17,Q18,Q19,Q20,Q21,Q22,Q23,Q24,Q25,Q26,Q27,Q28,Q29,Q30,Q31;
output [31:0]Q;
function [31:0] select;

    input [31:0]Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,Q17,Q18,Q19,Q20,Q21,Q22,Q23,Q24,Q25,Q26,Q27,Q28,Q29,Q30,Q31;
    input [31:0]R;
    case(R)
      5'b00000:select=Q0;  
      5'b00001:select=Q1;   
      5'b00010:select=Q2;   
      5'b00011:select=Q3;    
      5'b00100:select=Q4;  
      5'b00101:select=Q5;   
      5'b00110:select=Q6;   
      5'b00111:select=Q7;     
      5'b01000:select=Q8;  
      5'b01001:select=Q9;  
      5'b01010:select=Q10;  
      5'b01011:select=Q11;  
      5'b01100:select=Q12;  
      5'b01101:select=Q13;  
      5'b01110:select=Q14;  
      5'b01111:select=Q15;  
      5'b10000:select=Q16;  
      5'b10001:select=Q17;   
      5'b10010:select=Q18;   
      5'b10011:select=Q19;   
      5'b10100:select=Q20;  
      5'b10101:select=Q21;   
      5'b10110:select=Q22;   
      5'b10111:select=Q23;     
      5'b11000:select=Q24;  
      5'b11001:select=Q25;  
      5'b11010:select=Q26;  
      5'b11011:select=Q27;  
      5'b11100:select=Q28;  
      5'b11101:select=Q29;  
      5'b11110:select=Q30;  
      5'b11111:select=Q31;      
    endcase
  endfunction   //select one of the registers
  assign Q=select(Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,Q17,Q18,Q19,Q20,Q21,Q22,Q23,Q24,Q25,Q26,Q27,Q28,Q29,Q30,Q31,R);
endmodule

module D_Latch(D,En,Q,Qn);
input D,En;
output Q,Qn;
wire Sn,Rn,Dn;
not i0(Dn,D);
nand i1(Sn,D,En);
nand i2(Rn,En,Dn);
nand i3(Q,Sn,Qn);
nand i4(Qn,Q,Rn);
endmodule
 
module D_FF(D,Clk,Q,Qn);
input D,Clk;
output Q,Qn;
wire Clkn,Q0,Qn0;
not i0(Clkn,Clk);
D_Latch d0(D,Clkn,Q0,Qn0);
D_Latch d1(Q0,Clk,Q,Qn);
endmodule

module D_FFEC(D, Clk, En, Clrn, Q, Qn);
  // D Flip-Flop with Enable and Asynchronous Clear module
  input D, Clk, En, Clrn;   // Data input, Clock input, Enable input, Asynchronous Clear input
  output Q, Qn;             // Q (output) and Qn (complementary output)
  wire Y0, Y_C;             // Intermediate wires
  // Instantiate a 2-to-1 MUX (Multiplexer)
  MUX2X1 m0(Q, D, En, Y0);
  // Use an AND gate to combine the output of the MUX with the asynchronous clear signal
  and i0(Y_C, Y0, Clrn);
  // Instantiate a D Flip-Flop with an asynchronous clear
  D_FF d0(Y_C, Clk, Q, Qn);
endmodule


module D_FFEC32(D,Clk,En,Clrn,Q,Qn);
 input [31:0]D;
 input Clk,En,Clrn;
 output [31:0]Q,Qn;
 D_FFEC d0(D[0],Clk,En,Clrn,Q[0],Qn[0]);
 D_FFEC d1(D[1],Clk,En,Clrn,Q[1],Qn[1]);
 D_FFEC d2(D[2],Clk,En,Clrn,Q[2],Qn[2]);
 D_FFEC d3(D[3],Clk,En,Clrn,Q[3],Qn[3]);
 D_FFEC d4(D[4],Clk,En,Clrn,Q[4],Qn[4]);
 D_FFEC d5(D[5],Clk,En,Clrn,Q[5],Qn[5]);
 D_FFEC d6(D[6],Clk,En,Clrn,Q[6],Qn[6]);
 D_FFEC d7(D[7],Clk,En,Clrn,Q[7],Qn[7]);
 D_FFEC d8(D[8],Clk,En,Clrn,Q[8],Qn[8]);
 D_FFEC d9(D[9],Clk,En,Clrn,Q[9],Qn[9]);
 D_FFEC d10(D[10],Clk,En,Clrn,Q[10],Qn[10]);
 D_FFEC d11(D[11],Clk,En,Clrn,Q[11],Qn[11]);
 D_FFEC d12(D[12],Clk,En,Clrn,Q[12],Qn[12]);
 D_FFEC d13(D[13],Clk,En,Clrn,Q[13],Qn[13]);
 D_FFEC d14(D[14],Clk,En,Clrn,Q[14],Qn[14]);
 D_FFEC d15(D[15],Clk,En,Clrn,Q[15],Qn[15]);
 D_FFEC d16(D[16],Clk,En,Clrn,Q[16],Qn[16]);
 D_FFEC d17(D[17],Clk,En,Clrn,Q[17],Qn[17]);
 D_FFEC d18(D[18],Clk,En,Clrn,Q[18],Qn[18]);
 D_FFEC d19(D[19],Clk,En,Clrn,Q[19],Qn[19]);
 D_FFEC d20(D[20],Clk,En,Clrn,Q[20],Qn[20]);
 D_FFEC d21(D[21],Clk,En,Clrn,Q[21],Qn[21]);
 D_FFEC d22(D[22],Clk,En,Clrn,Q[22],Qn[22]);
 D_FFEC d23(D[23],Clk,En,Clrn,Q[23],Qn[23]);
 D_FFEC d24(D[24],Clk,En,Clrn,Q[24],Qn[24]);
 D_FFEC d25(D[25],Clk,En,Clrn,Q[25],Qn[25]);
 D_FFEC d26(D[26],Clk,En,Clrn,Q[26],Qn[26]);
 D_FFEC d27(D[27],Clk,En,Clrn,Q[27],Qn[27]);
 D_FFEC d28(D[28],Clk,En,Clrn,Q[28],Qn[28]);
 D_FFEC d29(D[29],Clk,En,Clrn,Q[29],Qn[29]);
 D_FFEC d30(D[30],Clk,En,Clrn,Q[30],Qn[30]);
 D_FFEC d31(D[31],Clk,En,Clrn,Q[31],Qn[31]);
 endmodule

module MUX2X1(A0,A1,S,Y);
 input A0,A1,S;
 output Y;
 not i0(S_n,S);
 nand i1(A0_S,A0,S_n);
 nand i2(A1_S,A1,S);
 nand i3(Y,A0_S,A1_S);
 endmodule