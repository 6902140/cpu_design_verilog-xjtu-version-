//使用有限状态自动机来实现，根据当前状态和输入判断下一个一个输出的信号
module CU(
input [3:0] S, [5:0] OP,
output reg PCWr,reg PCWrCond,reg IorD,reg MemRd,reg MemWr,reg IRWr,reg MemtoReg,
output reg [1:0] PCSrc,reg [1:0] ALUOp,reg [1:0] ALUSrcB,reg ALUSrcA,reg RegWr,reg RegDst,
output reg [3:0] NS //NextState 
);

always@(S)
begin
  case(S)
  0:NS<=1;//参考图示state0的下一个状态只有可能是state1
  1:
  begin
  	case(OP)//根据opcode来决策
  	6'b100011:NS<=2;
  	6'b101011:NS<=2;
  	6'b000000:NS<=6;
  	6'b000100:NS<=8;
  	6'b000010:NS<=9;
  	default:NS<=4'b1xx1;
  	endcase
  end
  2:
  begin
  	case(OP)
  	6'b100011:NS<=3;
  	6'b101011:NS<=5;
  	default:NS<=0;
  	endcase
  end
  3:NS<=4;
  4:NS<=0;
  5:NS<=0;
  6:NS<=7;
  7:NS<=0;
  8:NS<=0;
  9:NS<=0;
  default:NS<=0;
  endcase
end


//根据State来改变控制信号
always@(S)
case(S)
0:{PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg, PCSrc , ALUOp , ALUSrcB ,ALUSrcA,RegWr,RegDst}={16'b1001010_00_00_01_000};
1:{PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg, PCSrc , ALUOp , ALUSrcB ,ALUSrcA,RegWr,RegDst}={16'b0000000_00_00_11_000};
2:{PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg, PCSrc , ALUOp , ALUSrcB ,ALUSrcA,RegWr,RegDst}={16'b0000000_00_00_10_100};
3:{PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg, PCSrc , ALUOp , ALUSrcB ,ALUSrcA,RegWr,RegDst}={16'b0011000_00_00_00_000};
4:{PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg, PCSrc , ALUOp , ALUSrcB ,ALUSrcA,RegWr,RegDst}={16'b0000001_00_00_00_010};
5:{PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg, PCSrc , ALUOp , ALUSrcB ,ALUSrcA,RegWr,RegDst}={16'b0010100_00_00_00_000};
6:{PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg, PCSrc , ALUOp , ALUSrcB ,ALUSrcA,RegWr,RegDst}={16'b0000000_00_10_00_100};
7:{PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg, PCSrc , ALUOp , ALUSrcB ,ALUSrcA,RegWr,RegDst}={16'b0000000_00_00_00_011};
8:{PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg, PCSrc , ALUOp , ALUSrcB ,ALUSrcA,RegWr,RegDst}={16'b0100000_01_01_00_100};
9:{PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg, PCSrc , ALUOp , ALUSrcB ,ALUSrcA,RegWr,RegDst}={16'b1000000_10_00_00_000};
default:{PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg, PCSrc , ALUOp , ALUSrcB ,ALUSrcA,RegWr,RegDst}={16'b0000000_00_00_00_000};
endcase

endmodule
