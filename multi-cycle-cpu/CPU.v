
`include "MUX_2_32.v"//ok
`include "ALU_CU.v"//ok
`include "MUX_4_32.v"
`include "SHL2.v"//ok
`include "ALU.v"//ok
`include "reg_file.v"//ok
`include "SigExt16_32.v"//ok
`include "reg_PC.v"//ok
`include "reg_32.v"//ok
`include "ANDgate.v"//ok
`include "ORgate.v"//ok
`include "mem.v"//ok
`include "CU.v"//ok

module CPU(clock,rst);

input clock,rst;

wire [31:0] Inst,PC_in,PC_out,mem_addr,mem_data_R,A_in,B_in,A_out,B_out,ALU_A,ALU_B,ALU_out,ALUOut_out,MDR_out;
wire [31:0] SigExt_out,SHL2_32_out,WriteData_RF;
wire [27:0] SHL2_26_out;
wire PCWr,PCWrCond,IorD,MemRd,MemWr,IRWr,MemtoReg,ALUSrcA,RegWr,RegDst,ena_W_PC,Zero,ANDgate_out;
wire [1:0] PCSrc,ALUOp,ALUSrcB;
wire [3:0] SR_out,SR_in;
wire [2:0] ALUctl;
wire [4:0] WriteReg;

CU CU(.S(SR_out),.OP(Inst[31:26]),
.PCWr(PCWr),.PCWrCond(PCWrCond),.IorD(IorD),.MemRd(MemRd),.MemWr(MemWr),.IRWr(IRWr),.MemtoReg(MemtoReg),
.PCSrc(PCSrc),.ALUOp(ALUOp),.ALUSrcB(ALUSrcB),.ALUSrcA(ALUSrcA),.RegWr(RegWr),.RegDst(RegDst),
.NS(SR_in));

reg_SR SR(.clock(clock),.rst(rst),.In(SR_in),.Out(SR_out));

reg_PC PC(.clock(clock),.rst(rst),.ena_W(ena_W_PC),.In(PC_in),.Out(PC_out));

mem mem(.ena_W(MemWr),.ena_R(MemRd),.addr(mem_addr),.data_W(B_out),.data_R(mem_data_R),.clock(clock));

reg_IR IR(.clock(clock),.ena_W(IRWr),.In(mem_data_R),.Out(Inst));

reg_32 A(.clock(clock),.In(A_in),.Out(A_out));

reg_32 B(.clock(clock),.In(B_in),.Out(B_out));

ALU ALU(.ALUctl(ALUctl),.A(ALU_A),.B(ALU_B),.ALUout(ALU_out),.Zero(Zero));

ALU_CU ALU_CU(.ALUop(ALUOp),.FuncCode(Inst[5:0]),.Out(ALUctl));

reg_32 ALUOut(.clock(clock),.In(ALU_out),.Out(ALUOut_out));

reg_32 MDR(.clock(clock),.In(mem_data_R),.Out(MDR_out));

SigExt16_32 SigExt16_32(.In(Inst[15:0]),.Out(SigExt_out));

SHL2_32 SHL2_32(.In(SigExt_out),.Out(SHL2_32_out));

SHL2_26 SHL2_26(.In(Inst[25:0]),.Out(SHL2_26_out));

reg_file RF(.Read1(Inst[25:21]),.Read2(Inst[20:16]),.WriteReg(WriteReg),.WriteData(WriteData_RF),.RegWr(RegWr),.Data1(A_in),.Data2(B_in),.clock(clock));

ANDgate ANDgate(.in1(PCWrCond),.in2(Zero),.Out(ANDgate_out));

ORgate ORgate(.in1(PCWr),.in2(ANDgate_out),.Out(ena_W_PC));

MUX_2_32 MUX_2_32_1(.in0(PC_out),.in1(ALUOut_out),.Sel(IorD),.Out(mem_addr));

MUX_2_32 MUX_2_32_2(.in0(ALUOut_out),.in1(MDR_out),.Sel(MemtoReg),.Out(WriteData_RF));

MUX_2_32 MUX_2_32_3(.in0(PC_out),.in1(A_out),.Sel(ALUSrcA),.Out(ALU_A));

MUX_2_5 MUX_2_5(.in0(Inst[20:16]),.in1(Inst[15:11]),.Sel(RegDst),.Out(WriteReg));

MUX_4_32_1x MUX_4_32_1x(.in0(B_out),.in2(SigExt_out),.in3(SHL2_32_out),.Sel(ALUSrcB),.Out(ALU_B));

MUX_3_32 MUX_3_32(.in0(ALU_out),.in1(ALUOut_out),.in2({PC_out[31:28],SHL2_26_out[27:0]}),.Sel(PCSrc),.Out(PC_in));

endmodule
