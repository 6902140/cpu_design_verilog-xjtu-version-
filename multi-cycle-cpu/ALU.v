//根据ALUctl信号对输入的AB进行计算，如果结果为0,zero输出高电平
module ALU (ALUctl,A,B,ALUout,Zero);

input [2:0] ALUctl;
input [31:0] A,B;
output reg [31:0] ALUout;
output Zero;

assign Zero=(ALUout==0);

always @(ALUctl,A,B)
begin
	case(ALUctl)// ALU实现六种指令
		3'b000:ALUout<=A+B;	//add
		3'b001:ALUout<=A-B;	//sub
		3'b010:ALUout<=A+B;	//addu
		3'b011:ALUout<=A&B;	//and
		3'b100:ALUout<=A|B;	//or
		3'b101:ALUout<=~(A|B);	//nor
		default:ALUout<=0;
	endcase
end

endmodule
