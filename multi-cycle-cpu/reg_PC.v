module reg_PC(clock,rst,ena_W,In,Out);

input clock;
input rst;
input ena_W;	
input [31:0] In;
output [31:0] Out;

reg [31:0] PC;

always @(posedge rst) PC<=32'h00400000;
    
always @(posedge clock) 
begin
	if(ena_W) PC<=In;        //enable input 
end

assign Out=PC;

endmodule

module reg_IR(clock,ena_W,In,Out);

input clock;
input ena_W;	
input [31:0] In;
output [31:0] Out;

reg [31:0] IR;
   
always @(posedge clock) 
begin
	if(ena_W) IR<=In;        //enable input 
end

assign Out=IR;

endmodule


module reg_SR(clock,rst,In,Out);

input clock;
input rst;
input [3:0] In;
output [3:0] Out;

reg [3:0] SR;

always @(posedge rst) SR<=0;
    
always @(posedge clock) 
begin
	SR<=In;        
end

assign Out=SR;

endmodule


