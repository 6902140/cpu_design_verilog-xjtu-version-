
`timescale 1ns / 1ps

//X????1???
//Y????2???????
//Aluc??????
//R????????D???????????
//Z????1????????????????0????????????????

module ALU(X, Y, Aluc, R, Z);
  // ALU module for performing arithmetic and logic operations
  input [31:0] X, Y;   // Input data X and Y, each 32 bits wide
  input [1:0] Aluc;    // Selector for determining the operation to be performed
  output [31:0] R;     // Output result 1
  output Z;            // Output result 2

  wire [31:0] d_as, d_and, d_or, d_and_or;

  // Call the ADDSUB_32 module to perform addition or subtraction, and store the result in d_as
  ADDSUB_32 as(X, Y, Aluc[0], d_as);

  // Use logical gates to implement the AND and OR operations on X and Y, and store the results in d_and and d_or
  assign d_and = X & Y;
  assign d_or = X | Y;

  // Use the MUX2X32 module to select between d_and and d_or based on Aluc[0], and store the result in d_and_or
  MUX2X32 select1(d_and, d_or, Aluc[0], d_and_or);

  // Use the MUX2X32 module to select between d_as and d_and_or based on Aluc[1], and store the result in R
  MUX2X32 selected(d_as, d_and_or, Aluc[1], R);

  // Negate R and check if all bits are 0 using the | operator, and store the result in Z
  assign Z = ~|R;

endmodule


module MUX2X32(A0,A1,S,Y);
input [31:0] A0,A1;
input S;
output [31:0] Y;
function [31:0] select;
input [31:0] A0,A1;
input S;
case(S)
0:select=A0;
1:select=A1;
endcase
endfunction
assign Y=select(A0,A1,S);
endmodule
