//RegWr为写使能信号，当其有效时clocl上升沿将数据写入寄存器

module reg_file(Read1,Read2,WriteReg,WriteData,RegWr,Data1,Data2,clock);

input [4:0] Read1,Read2,WriteReg;	//reg number
input [31:0] WriteData;	//data to write
input RegWr;	//write control
input clock;	//clock for write
output [31:0] Data1,Data2;	//data to read

reg [31:0] RF [31:0];	//32 reg(32bit)通过这种方式实现寄存器组件

//测试用
initial
begin
	RF[0]=0;
end

assign Data1=RF[Read1];
assign Data2=RF[Read2];

//时钟信号一旦到来并且写使能启动便写入数据
always
begin
	@(posedge clock) if(RegWr) RF[WriteReg]<=WriteData;
end

endmodule
