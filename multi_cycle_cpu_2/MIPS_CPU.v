//select 1 from 2 ,32 bit/二选一多路选择器
module mux2_to_1_32bit(out,i0,i1,s0);
input [31:0]i0,i1;
input s0;
output reg[31:0]out;
always@(*)
begin
  if(s0==0)
  out<=i0;
  else
  out<=i1;
end
endmodule

//3选1多路选择器
module mux3_to_1_32bit(out,i0,i1,i2,s0,s1);
input [31:0]i0,i1,i2;
input s0,s1;
output reg[31:0]out;
always@(*)
begin
  case({s1,s0})
  2'b00:out<=i0;
  2'b01:out<=i1;
  2'b10:out<=i2;
  default:out<=32'bx;
  endcase
end
endmodule

//register select mux,select 1 from 2 ,5 bit
module mux2_to_1_5bit(out,i0,i1,s0);
input [4:0]i0,i1;
input s0;
output reg[4:0]out;
always@(*)
begin
  if(s0==0)
  out<=i0;
  else
  out<=i1;
end
endmodule

//use for multicycle CPU
//0扩展
module SigExt5_32(IN,OUT);
      input [4:0] IN;
      output [31:0] OUT;
      assign OUT={{27{1'b0}},IN};
endmodule


//use for multicycle CPU
module MSigExt16_32(IN,OUT,EXP);
      input [15:0] IN;
      input EXP;
      output reg[31:0] OUT;
      always@(*)
      begin
      if(EXP==0)//符号扩展
      OUT<={{16{IN[15]}},IN};
      else//无符号扩展
      OUT<={{16{1'b0}},IN};
      end
endmodule

//条件跳转指令
module IFBRANCH(Con,ZF,SF,Branch);
input [2:0]Con;
input ZF,SF;
output reg Branch;
parameter BEQ=3'b000,
          BNE=3'b001,
          BGEZ=3'b010,
          BGTZ=3'b011,
          BLEZ=3'b100,
          BLTZ=3'b101;

initial begin
Branch=1'b0;
end


always@(*)
begin
case(Con)
   BEQ:begin
       if(ZF==1) Branch=1'b1;
       else Branch=1'b0;
       end
   BNE:begin
       if(ZF==0) Branch=1'b1;
       else Branch=1'b0;
       end
   BGEZ:begin
        if(SF==0) Branch=1'b1;
        else Branch=1'b0;
        end
   BGTZ:begin
        if(SF==0&&ZF==0) Branch=1'b1;
        else Branch=1'b0;
        end
   BLEZ:begin
        if(SF==1||ZF==1) Branch=1'b1;
        else Branch=1'b0;
        end
   BLTZ:begin
        if(SF==1) Branch=1'b1;
        else Branch=1'b0;
        end
   default:Branch=1'b0;

endcase
end
endmodule

//combine pc and Imm for address;
module COMBINE(PC,IMM,ADDR);
input [31:0] PC;
input [25:0] IMM;
output [31:0] ADDR;
assign ADDR={PC[31:28],IMM,1'b0,1'b0};
endmodule

//use for 32 bit data move left 2 bit
module SHL2_32(IN,OUT);
input [31:0] IN;
output [31:0] OUT;
assign OUT=IN<<2;
endmodule

module mux3_to_1_A(out,i0,i1,i2,s0,s1);
input [31:0]i0,i1,i2;
input s0,s1;
output reg[31:0]out;
always@(*)
begin
  case({s1,s0})
  2'b00:out<=i0;
  2'b01:out<=i1;
  2'b10:out<=i2;
  2'b11:out<=i2;
  default:out<=32'bx;
  endcase
end
endmodule

//ALUSrcB selector
module mux5_to_1_32bit(out,i0,i2,i3,s0,s1,s2);
input [31:0]i0,i2,i3;
input s0,s1,s2;
output reg[31:0]out;
always@(*)
begin
  case({s2,s1,s0})
  3'b000:out<=i0;
  3'b001:out<=4;
  3'b010:out<=i2;
  3'b011:out<=i3;
  3'b100:out<=0;
  3'b101:out<=0;
  3'b110:out<=0;
  3'b111:out<=0;
  default:out<=32'bx;
  endcase
end
endmodule

module Dtrigger(D,Q,CLK);
input [31:0]D;
input CLK;
output reg [31:0]Q;
always@(posedge CLK)
  Q <= D;    
endmodule

module MALU_CU(INST,OP0,OP1,OP2,CTRL,ALUSrc1);
input [5:0]INST;
input OP0,OP1,OP2;
output reg [3:0]CTRL;
output reg ALUSrc1;
parameter ADD=4'b0000,
            SUB=4'b0001,
            AND=4'b0010,
            OR=4'b0011,
            XOR=4'b0100,
            SLL=4'b0101,
            SRL=4'b0111,
	    SRA=4'b1001,
            SLT=4'b0110,
            NOR=4'b1000,
            FADD=6'b100000,
            FSUB=6'b100010,
            FAND=6'b100100,
            FOR=6'b100101,
            FXOR=6'b100110,
            FSLL=6'b000000,
            FSLLV=6'b000100,
            FSRL=6'b000010,
            FSRLV=6'b000110,
            FSRA=6'b000011,
            FSRAV=6'b000111,
            FSLT=6'b101010,
            FNOR=6'b100111;
initial begin
CTRL=4'bx;
ALUSrc1=1'b0;
end
always@(INST,OP0,OP1,OP2)
begin
ALUSrc1=1'b0;
if({OP2,OP1,OP0}==3'b000)
CTRL=ADD;
else if({OP2,OP1,OP0}==3'b001)
CTRL=SUB;
else if({OP2,OP1,OP0}==3'b010)
CTRL=AND;
else if({OP2,OP1,OP0}==3'b011)
CTRL=OR;
else if({OP2,OP1,OP0}==3'b100)
CTRL=SLT;
else if({OP2,OP1,OP0}==3'b101)
CTRL=XOR;
else if({OP2,OP1}==2'b11)
begin
if(INST[5:2]==4'b0000) ALUSrc1=1'b1;
else ALUSrc1=1'b0;
case(INST)
  FADD:CTRL=ADD;
  FSUB:CTRL=SUB;
  FAND:CTRL=AND;
  FOR:CTRL=OR;
  FXOR:CTRL=XOR;
  FSLL:CTRL=SLL;
  FSLLV:CTRL=SLL;
  FSRL:CTRL=SRL;
  FSRLV:CTRL=SRL;
  FSRA:CTRL=SRA;
  FSRAV:CTRL=SRA;
  FSLT:CTRL=SLT;
  FNOR:CTRL=NOR;
  default:CTRL=4'bx;
endcase
end

else
CTRL=4'bx;
end
endmodule

//use for more instructions
module MALU(
  ALUCtrl,X,Y,ZF,OUT,COUT,SF,OF
      );

input [31:0] X,Y;
input [3:0]ALUCtrl;
output  reg ZF;
output  reg COUT;
output  reg SF;
output reg OF;
output reg [31:0] OUT; 
  parameter ADD=4'b0000,
            SUB=4'b0001,
            AND=4'b0010,
            OR=4'b0011,
            XOR=4'b0100,
            SLL=4'b0101,
            SRL=4'b0111,
	    SRA=4'b1001,
            SLT=4'b0110,
	    NOR=4'b1000;
initial
begin
  assign COUT=0;
  assign ZF=0;
  assign SF=0;
end  
  always@(*)
  begin
  case(ALUCtrl)
    ADD:{COUT,OUT}<=X+Y;//add
    SUB:{COUT,OUT}<=X-Y;//sub
    AND:OUT<=X&Y;//and
    OR:OUT<=X|Y;//or
    XOR:OUT<=X^Y;//xor
    SLL:OUT<=(Y<<X);//sll
    SRL:OUT<=(Y>>X);//srl
    SRA:OUT<=($signed(Y)>>>X);//sra
    SLT:begin
        if($signed(X)<$signed(Y)) OUT<=1;
        else OUT<=0;
        end
    NOR:OUT<=~(X^Y);
    default:OUT<=X;
  endcase
  assign ZF=~|OUT;
  assign SF=OUT[31];
  end
  endmodule




module REGGROUP(W_data,R_Reg1,R_Reg2,W_Reg,W,R_data1,R_data2,CLK);
input[31:0] W_data;
input [4:0] R_Reg1,R_Reg2,W_Reg;
input W,CLK;
output reg [31:0]R_data1,R_data2;
integer i;
reg [31:0]regs[31:0];
initial 
begin
  regs[1]=65;
  regs[0]=2;
  for ( i = 2 ; i <= 31 ; i = i + 1) 
    regs [i] = 0;
end
always@(*)
begin
    R_data1=regs[R_Reg1];
    R_data2=regs[R_Reg2];
end

always@(posedge CLK)
begin
  if(W==1) 
    regs[W_Reg]<=W_data;
end

endmodule

module INSDATAMEM(Addr,R_data,W_data,R,W);
input [31:0]Addr,W_data;
input R,W;
output reg[31:0]R_data;
reg [31:0]datamem[2047:0];
integer i;
initial 
begin
  datamem[0]=128;
  for ( i = 1; i <= 1023 ; i = i + 1) 
    datamem [i] = i;
  R_data=32'bx;
  //initial instructions
  datamem[1024]=32'b000000_00000_00001_00010_00000_100000;//r2=r0+r1
  datamem[1025]=32'b000000_00000_00000_00001_00010_000011;//r1=r0>>>2
  datamem[1026]=32'b000000_00000_00001_00010_00000_000111;//r2=r1>>>r0
  datamem[1027]=32'b001000_00000_00001_0000000000000111;//r1=r0+7
  datamem[1028]=32'b101011_00001_00001_0000000000000111;//store r1 M[r1+7] 
  datamem[1029]=32'b100011_00001_00010_0000000000000111;//load M[r1+7] to r2
  
  //datamem[1030]=32'b000101_00000_00001_0000000000000011;//r1!=r0?PC+12:PC+4
  datamem[1030]=32'b000001_00000_00001_0000000000000010;//r0>=0?PC+12:PC+4
  //datamem[1030]=32'b000111_00001_00001_0000000000000011;//r1>0?PC+4+12:PC+4
  datamem[1031]=32'b000010_00000000000000010000000001;
  datamem[1032]=32'b000010_00000000000000010000000010;
  datamem[1033]=32'b000010_00000000000000010000000000;//jump target 
  datamem[1034]=32'b000010_00000000000000010000000100;//jump to datamem[1024]
end
always@(*)
begin
  if(W==0&&R==1)//read data or instruction;
    begin
     #1 R_data<=datamem[Addr[31:2]%2048];
    end

  else if(W==1&&R==0)
    begin
     #1 datamem[Addr[31:2]%1024]=W_data;//write data
    end
end
endmodule 
module IR(D,W,CLK,INST);
input [31:0]D;
input W,CLK;
output reg [31:0]INST;
always@(posedge CLK)
begin
if(W==1)
  INST <= D; 
end   
endmodule
//combine instruction memory and data memory,insruction is from 1024 to 2047
module MDtrigger_pc(CLK,D,Q,RESET,W);
input CLK,RESET,W;
input [31:0] D;
output reg [31:0] Q;
initial begin
Q<=1024;
end
always@(posedge CLK or posedge RESET)
if (RESET)
  Q <= 4096;  
else if(W==1)  
   Q=D;  
endmodule


//use for multicycle CPU
module MCU(OP,BG,CLK,RESET,IorD,IRWr,PCWr,PCWrCond,Con,
RegDst,RegWr,ALUSrcA,PCSrc,ALUOp,
ALUSrcB,MemtoReg,MemRd,MemWr,BSrc,Exp);
input [5:0]OP;
input CLK,RESET;
input BG;//identify bgez/bltz,Inst[16]
output IorD,IRWr,PCWr,PCWrCond,RegDst,ALUSrcA,RegWr,MemtoReg,MemRd,MemWr,BSrc,Exp;
output [1:0] PCSrc,ALUSrcB;
output [2:0] ALUOp,Con;
reg [4:0] CurrentState,NextState;
reg [21:0] controls;
parameter IFetch=5'b00000,
          IDecode=5'b00001,
          MemAddr=5'b00011,
          ALCalcu=5'b00100,
          BeqBranch=5'b00101,
          BneBranch=5'b00110,
          BgezBranch=5'b00111,
          BgtzBranch=5'b01000,
          BlezBranch=5'b01001,
          BltzBranch=5'b01010,
          JumpAdd=5'b01011,
          Addiexe=5'b01100,
          Andiexe=5'b01101,
          Oriexe=5'b01110,
          Sltiexe=5'b01111,
          Xoriexe=5'b10000,
          Memread=5'b10001,
          Memwrite=5'b10010,
          Regwrite_R=5'b10011,
          Regwrite_LW=5'b10100,
          Regwrite_RI=5'b10101,
          //op_code
          addi=6'b001000,
          andi=6'b001100,
          ori=6'b001101,
          xori=6'b001010,
          slti=6'b001110,
          beq=6'b000100,
          bne=6'b000101,
          bgeltz=6'b000001,
          bgtz=6'b000111,
          blez=6'b000110,
          rr=6'b000000,
          j=6'b000010,
          lw=6'b100011,
          sw=6'b101011;

always@(posedge CLK or posedge RESET)
if (RESET) CurrentState <= IFetch;
else       CurrentState <= NextState;

always@(OP,BG,CurrentState)
begin
  case(CurrentState)
  IFetch:NextState=IDecode;  
  IDecode: begin
           case(OP)
           lw:NextState=MemAddr;
           sw:NextState=MemAddr;           
           rr:NextState=ALCalcu;
           beq:NextState=BeqBranch;
           bne:NextState=BneBranch;
           bgeltz:begin 
                  if(BG==1) NextState=BgezBranch;
                  else NextState=BltzBranch;
                  end
           bgtz:NextState=BgtzBranch;
           blez:NextState=BlezBranch;
           j:NextState=JumpAdd;
           addi:NextState=Addiexe;
           andi:NextState=Andiexe;
           ori:NextState=Oriexe;
           xori:NextState=Xoriexe;
           slti:NextState=Sltiexe;
           default:NextState=5'bx;
           endcase
           end
  MemAddr:begin
          case(OP)
          lw:NextState=Memread;
          sw:NextState=Memwrite;
          default:NextState=5'bx;
          endcase
          end
  ALCalcu:NextState=Regwrite_R;
  Addiexe:NextState=Regwrite_RI;
  Oriexe:NextState=Regwrite_RI;  
  Xoriexe:NextState=Regwrite_RI;
  Andiexe:NextState=Regwrite_RI;
  Sltiexe:NextState=Regwrite_RI;
  BeqBranch:NextState=IFetch;
  BneBranch:NextState=IFetch;
  BgezBranch:NextState=IFetch;
  BgtzBranch:NextState=IFetch;
  BlezBranch:NextState=IFetch;
  BltzBranch:NextState=IFetch;
  JumpAdd:NextState=IFetch;
  Regwrite_R:NextState=IFetch;
  Regwrite_RI:NextState=IFetch;
  Memwrite:NextState=IFetch;
  Memread:NextState=Regwrite_LW;
  Regwrite_LW:NextState=IFetch;
  default:NextState=5'bx;
endcase    
end
assign{IorD,IRWr,PCWr,PCWrCond,Con,
RegDst,RegWr,ALUSrcA,PCSrc,
ALUOp,ALUSrcB,MemtoReg,MemRd,MemWr,BSrc,Exp}=controls;

always@(CurrentState)
begin
case(CurrentState)
  IFetch:controls=22'b0110000000000000101000;
  IDecode:controls=22'b0000000000000001100000;
  MemAddr:controls=22'b0000000001000001000000;
  ALCalcu:controls=22'b0000000001001100000000;
  Addiexe:controls=22'b0000000001000001000000;
  Andiexe:controls=22'b0000000001000101000001;
  Oriexe:controls=22'b0000000001000111000001;
  Sltiexe:controls=22'b0000000001001001000000;
  Xoriexe:controls=22'b0000000001001011000001;
  BeqBranch:controls=22'b0001000001010010000000;
  BneBranch:controls=22'b0001001001010010000000;
  BgezBranch:controls=22'b0001010001010010000010;
  BgtzBranch:controls=22'b0001011001010010000010;
  BlezBranch:controls=22'b0001100001010010000010;
  BltzBranch:controls=22'b0001101001010010000010;
  JumpAdd:controls=22'b0010000000100000000000;
  Memread:controls=22'b1000000000000000001000;
  Memwrite:controls=22'b1000000000000000000100;
  Regwrite_R:controls=22'b0000000110000000000000;
  Regwrite_LW:controls=22'b0000000010000000010000;
  Regwrite_RI:controls=22'b0000000010000000000000;
  default:controls=22'bx;
endcase
end
endmodule


module tb_mymcpu();
reg clk;
reg reset;//pc and cu reset
wire [31:0]pcq;//pc out
wire pcw;//pc write control
wire [3:0]aluctrl;
wire IorD,IRWr,PCWr,PCWrCond,RegDst,ALUSrcA,ALUSrc1,RegWr,MemtoReg,MemRd,MemWr,BSrc,Exp;
wire [1:0] PCSrc,ALUSrcB;
wire [2:0] ALUOp,Con;
reg [22:0] controls;
wire [31:0]s;//ALU output
wire cout;//overflow bit
wire zf;//zero sign
wire sf;//signal sign
wire [4:0] sec1;//register select
wire [31:0] sec2;//address select 
wire [31:0] sec3;//wdata select
//wire [31:0] sec4;//bsrc select
wire [31:0] sec5;//PCSrc select
wire [31:0] seca;//ALUSrcA select
wire [31:0] secb;//ALUSrcB select
wire [31:0] rdata;//memory
wire [31:0] rdata1;//RF
wire [31:0] rdata2;//RF
wire [31:0] inst;//IR output
wire [31:0] mdrq;//MDR output
wire [31:0] aluout;
wire [31:0] Aq;//ALUSrcA output
wire [31:0] Bq;//ALUSrcB output
wire [31:0] temp1;//SHL2 output,32bit
wire [31:0] temp2;//sigext16/32 output
wire [31:0] temp3;//sigext5/32 output
wire [31:0] temp4;//combine pc address
wire  temp5;//ifbranch output
parameter t=20;
MDtrigger_pc PC(.CLK(clk),.D(sec5),.Q(pcq),.RESET(reset),.W((temp5&PCWrCond)|PCWr));
IR myIR(.D(rdata),.W(IRWr),.CLK(clk),.INST(inst));

INSDATAMEM memory(.Addr(sec2),.R_data(rdata),.W_data(Bq),.R(MemRd),.W(MemWr));
Dtrigger MDR(.D(rdata),.Q(mdrq),.CLK(clk));

REGGROUP RF(.W_data(sec3),.R_Reg1(inst[25:21]),.R_Reg2(inst[20:16]),
.W_Reg(sec1),.W(RegWr),.R_data1(rdata1),.R_data2(rdata2),.CLK(clk));



MALU alu(.ALUCtrl(aluctrl),.X(seca),.Y(secb),.ZF(zf),.OUT(s),.COUT(cout),.SF(sf));
MALU_CU alucu(.INST(inst[5:0]),.OP0(ALUOp[0]),.OP1(ALUOp[1]),.OP2(ALUOp[2]),.CTRL(aluctrl),.ALUSrc1(ALUSrc1));
Dtrigger da(.D(rdata1),.Q(Aq),.CLK(clk));
Dtrigger db(.D(rdata2),.Q(Bq),.CLK(clk));
Dtrigger ALUOut(.D(s),.Q(aluout),.CLK(clk));
//mux2_to_1_32B mux4(.out(sec4),.i0(rdata2),.s0(BSrc));
mux5_to_1_32bit srcB(.out(secb),.i0(Bq),.i2(temp2),.i3(temp1),.s0(ALUSrcB[0]),.s1(ALUSrcB[1]),.s2(BSrc));
//mux4_to_1_32bit srcB(.out(secb),.i0(Bq),.i2(temp2),.i3(temp1),.s0(ALUSrcB[0]),.s1(ALUSrcB[1]));
mux3_to_1_A srcA(.out(seca),.i0(pcq),.i1(Aq),.i2(temp3),.s0(ALUSrcA),.s1(ALUSrc1));
SHL2_32 shl2(.IN(temp2),.OUT(temp1));


COMBINE addcom(.PC(pcq),.IMM(inst[25:0]),.ADDR(temp4));
IFBRANCH ifb(.Con(Con),.ZF(zf),.SF(sf),.Branch(temp5));

MSigExt16_32 sig16_32(.IN(inst[15:0]),.OUT(temp2),.EXP(Exp));
SigExt5_32 sig5_32(.IN(inst[10:6]),.OUT(temp3));

MCU CU(.OP(inst[31:26]),.BG(inst[16]),.CLK(clk),.RESET(reset),
.IorD(IorD),.IRWr(IRWr),.PCWr(PCWr),.PCWrCond(PCWrCond),.Con(Con),
.RegDst(RegDst),.RegWr(RegWr),.ALUSrcA(ALUSrcA),.PCSrc(PCSrc),.ALUOp(ALUOp),
.ALUSrcB(ALUSrcB),.MemtoReg(MemtoReg),.MemRd(MemRd),.MemWr(MemWr),.BSrc(BSrc),.Exp(Exp));

mux2_to_1_5bit mux1(.out(sec1),.i0(inst[20:16]),.i1(inst[15:11]),.s0(RegDst));
mux2_to_1_32bit mux2(.out(sec2),.i0(pcq),.i1(aluout),.s0(IorD));
mux2_to_1_32bit mux3(.out(sec3),.i0(aluout),.i1(mdrq),.s0(MemtoReg));
mux3_to_1_32bit mux5(.out(sec5),.i0(s),.i1(aluout),.i2(temp4),.s0(PCSrc[0]),.s1(PCSrc[1]));
initial begin
#0
clk=1;
reset=1;
#1
reset=0;
end
always#(t/2)
begin
clk=~clk;
controls={IorD,IRWr,PCWr,PCWrCond,Con,
RegDst,RegWr,ALUSrc1,ALUSrcA,PCSrc,
ALUOp,ALUSrcB,MemtoReg,MemRd,MemWr,BSrc,Exp};
end
endmodule