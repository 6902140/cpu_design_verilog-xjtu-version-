
module SHL2_32(In,Out);

input [31:0] In;
output [31:0] Out;

assign Out={In[29:0],2'b00};

endmodule

module SHL2_26(In,Out);

input [25:0] In;
output [27:0] Out;

assign Out={In[25:0],2'b00};

endmodule
