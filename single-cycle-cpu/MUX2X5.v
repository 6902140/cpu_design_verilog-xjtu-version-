
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/11/28 21:35:10
// Design Name: 
// Module Name: MUX2X5
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module MUX2X5(A0,A1,S,Y);
input [4:0] A0,A1;
input S;
output [4:0] Y;
function [4:0] select;
input [4:0] A0,A1;
input S;
case(S)
0:select=A0;
1:select=A1;
endcase
endfunction
assign Y=select(A0,A1,S);
endmodule

