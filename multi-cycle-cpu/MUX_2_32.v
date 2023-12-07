module MUX_2_32(in0,in1,Sel,Out);

input [31:0] in0,in1;
input Sel;
output reg[31:0] Out;

always@(in0,in1,Sel)
case (Sel)
	0: Out<=in0;
	default: Out<=in1;
endcase

endmodule

module MUX_2_5(in0,in1,Sel,Out);

input [4:0] in0,in1;
input Sel;
output reg[4:0] Out;

always@(in0,in1,Sel)
case (Sel)
	0: Out<=in0;
	default: Out<=in1;
endcase

endmodule
