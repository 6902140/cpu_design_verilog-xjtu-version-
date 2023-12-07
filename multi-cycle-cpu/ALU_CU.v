//根据ALU的opcode输出ALUctl信号
module ALU_CU(ALUop,FuncCode,Out);

input [1:0] ALUop;
input [5:0] FuncCode;
output reg [2:0] Out;
/*
对应编号为：
add	000
sub	001
addu	010
and	011
or	100
nor	101
*/
always @*
case(ALUop)
	2'b00:Out=3'b010;	//addu
	2'b01:Out=3'b001;	//sub
	2'b10:
	begin
		case(FuncCode)
			6'b100000:Out=3'b000;	//add
			6'b100010:Out=3'b001;	//sub
			6'b100001:Out=3'b010;	//addu
			6'b100100:Out=3'b011;	//and
			6'b100101:Out=3'b100;	//or
			6'b100111:Out=3'b101;	//nor	
			default:Out=3'b111;
		endcase
	end
	default:Out=3'b111;
endcase

endmodule
