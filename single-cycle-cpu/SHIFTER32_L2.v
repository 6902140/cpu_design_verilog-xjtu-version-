
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/11/28 21:38:14
// Design Name: 
// Module Name: SHIFTER32_L2
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


module SHIFTER32_L2(X,Sh);
input [31:0] X;
output [31:0] Sh;
parameter z=2'b00;
assign Sh={X[29:0],z};
endmodule

module SHIFTER_COMBINATION(X,PCADD4,Sh);
input [26:0] X;
input [31:0] PCADD4;
output [31:0] Sh;
parameter z=2'b00;
assign Sh={PCADD4[3:0],X[26:0],z};
endmodule

