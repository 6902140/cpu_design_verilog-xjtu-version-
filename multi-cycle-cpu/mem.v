//合并了数据存储器和指令存储器，同样使用寄存器堆来模拟，并且初始化测试指令
module mem(ena_W,ena_R,addr,data_W,data_R,clock);

input ena_W,ena_R,clock;
input [31:0] addr;
input [31:0] data_W;
output reg [31:0] data_R;

reg [31:0] mem [1023:0];

//用作测试
integer i;
initial
begin
	for(i=0;i<512;i=i+1)
		mem[i]=i;
end

//用作测试
initial
begin
mem[512+0]=32'b100011_00000_00100_00000000_00000011;	//lw
mem[512+1]=32'b100011_00000_00001_00000000_00000001;	//lw
mem[512+2]=32'b100011_00000_00010_00000000_00000010;	//lw
mem[512+3]=32'b100011_00000_00011_00000000_00000100;	//lw
mem[512+4]=32'b000100_00010_00011_00000000_00000010;	//beq
mem[512+5]=32'b000000_00010_00001_00010_00000_100000;	//add
mem[512+6]=32'b000010_00_0001_0000_0000_0000_0000_0100;	//j
mem[512+7]=32'b101011_00000_00010_00000000_00000011;	//sw
mem[512+8]=32'b000010_00_0001_0000_0000_0000_0000_0000;	//j
end

always@(negedge clock)
begin
	if(ena_W) 
	begin
		if((addr)<512)//如果满足是数据存储区域
		begin 
			mem[(addr)]<=data_W;
		end
	end
end

always@(addr,ena_R,data_R)
begin
	if(ena_R)
	begin
        if((addr)<512) data_R<=mem[(addr)];	//读取数据
        else data_R<=mem[((addr-32'h00400000)>>2)+512];	//读取指令
	end
end 

endmodule
