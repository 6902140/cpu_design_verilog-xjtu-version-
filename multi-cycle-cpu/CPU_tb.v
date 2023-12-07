`include "CPU.v"

module CPU_TEST();

reg rst;
reg clock;
initial
begin
	rst=0;
	clock=0;
	#1 rst=1;
	#5 rst=0;
end
always #50 clock=~clock;

CPU CPU(.clock(clock),.rst(rst));

endmodule

