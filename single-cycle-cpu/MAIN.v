
`timescale 1ns / 1ps

module MAIN(Clk,Reset,Addr,Inst,Qa,Qb,R,Result,D);
input Clk,Reset;
output [31:0] Inst,Result,R,Qb,Qa,Addr,D;

wire [31:0]Result,PCadd4,EXTIMM,InstL2,EXTIMML2,D,Y,Dout,mux4x32_2,R;
wire Z,Regrt,Se,Wreg,Aluqb,Reg2reg,Cout,Wmem;
wire [1:0]Aluc,Pcsrc;
wire [4:0]Wr;

PC pc(Clk,Reset,Result,Addr);
PCadd4 pcadd4(Addr,PCadd4);
INSTMEM instmem(Addr,Inst);

CONUNIT conunit(Inst[31:26],Inst[5:0],Z,Regrt,Se,Wreg,Aluqb,Aluc,Wmem,Pcsrc,Reg2reg);
MUX2X5 mux2x5(Inst[15:11],Inst[20:16],Regrt,Wr);
EXT16T32 ext16t32(Inst[15:0],Se,EXTIMM);
SHIFTER_COMBINATION shifter1(Inst[26:0],PCadd4,InstL2);
SHIFTER32_L2 shifter2(EXTIMM,EXTIMML2);
REGFILE regfile(Inst[25:21],Inst[20:16],D,Wr,Wreg,Clk,Reset,Qa,Qb);
MUX2X32 mux2x321(EXTIMM,Qb,Aluqb,Y);
ALU alu(Qa,Y,Aluc,R,Z);
DATAMEM datamem(R,Qb,Clk,Wmem,Dout);
MUX2X32 mux2x322(Dout,R,Reg2reg,D);
CLA_32 cla_32(PCadd4,EXTIMML2,0,mux4x32_2, Cout);
MUX4X32 mux4x32(PCadd4,0,mux4x32_2,InstL2,Pcsrc,Result);
//assign NEXTADDR=Result;
//assign ALU_R=R;
endmodule

//input the op and func to ouput control messege
module CONUNIT(Op,Func,Z,Regrt,Se,Wreg,Aluqb,Aluc,Wmem,Pcsrc,Reg2reg);
input[5:0]Op,Func;
input Z;
output Regrt,Se,Wreg,Aluqb,Wmem,Reg2reg;
output[1:0]Pcsrc,Aluc;
wire R_type=~|Op; //op all 0 then rtype=1
wire I_add=R_type&Func[5]&~Func[4]&~Func[3]&~Func[2]&~Func[1]&~Func[0];
wire I_sub=R_type&Func[5]&~Func[4]&~Func[3]&~Func[2]&Func[1]&~Func[0];
wire I_and=R_type&Func[5]&~Func[4]&~Func[3]&Func[2]&~Func[1]&~Func[0];
wire I_or=R_type&Func[5]&~Func[4]&~Func[3]&Func[2]&~Func[1]&Func[0];

wire I_addi=~Op[5]&~Op[4]&Op[3]&~Op[2]&~Op[1]&~Op[0];
wire I_andi=~Op[5]&~Op[4]&Op[3]&Op[2]&~Op[1]&~Op[0];
wire I_ori=~Op[5]&~Op[4]&Op[3]&Op[2]&~Op[1]&Op[0];
wire I_lw=Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0];
wire I_sw=Op[5]&~Op[4]&Op[3]&~Op[2]&Op[1]&Op[0];
wire I_beq=~Op[5]&~Op[4]&~Op[3]&Op[2]&~Op[1]&~Op[0];
wire I_bne=~Op[5]&~Op[4]&~Op[3]&Op[2]&~Op[1]&Op[0];
wire I_J=~Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&~Op[0];

//make the control messege
assign Regrt=I_addi|I_andi|I_ori|I_lw|I_sw|I_beq|I_bne|I_J;
assign Se=I_addi|I_lw|I_sw|I_beq|I_bne;
assign Wreg=I_add|I_sub|I_and|I_or|I_addi|I_andi|I_ori|I_lw;
assign Aluqb=I_add|I_sub|I_and|I_or|I_beq|I_bne|I_J;
assign Aluc[1]=I_and|I_or|I_andi|I_ori;
assign Aluc[0]=I_sub|I_or|I_ori|I_beq|I_bne;
assign Wmem=I_sw;
assign Pcsrc[1]=I_beq&Z|I_bne&~Z|I_J;
assign Pcsrc[0]=I_J;
assign Reg2reg=I_add|I_sub|I_and|I_or|I_addi|I_andi|I_ori|I_sw|I_beq|I_bne|I_J;


endmodule

module TEST;
reg Clk;
reg Reset;
wire [31:0] Addr,Inst,Qa,Qb,R,Result,D;
MAIN uut(
.Clk(Clk),
.Reset(Reset),
.Addr(Addr),
.Inst(Inst),
.Qa(Qa),
.Qb(Qb),
.R(R),
.Result(Result),
.D(D)
);

initial begin
Clk=0;Reset=0;
#10;
Clk=1;Reset=0;
#10;
Reset=1;
Clk=0;
forever #20 Clk=~Clk;
end
endmodule



