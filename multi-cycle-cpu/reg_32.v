module reg_32(clock,In,Out);

input clock;
input [31:0] In;
output [31:0] Out;

reg [31:0] reg32;
    
always @(posedge clock) 
begin
	reg32<=In;       
end

assign Out=reg32;

endmodule

