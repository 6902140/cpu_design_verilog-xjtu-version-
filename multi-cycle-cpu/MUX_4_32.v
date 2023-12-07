
module MUX_3_32(in0,in1,in2,Sel,Out);

input [31:0] in0,in1,in2;
input [1:0] Sel;
output reg[31:0] Out;

always@(in0,in1,in2,Sel)
case (Sel)
	0: Out<=in0;
	1: Out<=in1;
	2: Out<=in2;
	default: Out<=in0;
endcase

endmodule

module MUX_4_32_1x(in0,in2,in3,Sel,Out);

input [31:0] in0,in2,in3;
input [1:0] Sel;
output reg[31:0] Out;

always@(in0,in2,in3,Sel)
case (Sel)
	0: Out<=in0;
	1: Out<=4;
	2: Out<=in2;
	default: Out<=in3;
endcase

endmodule
