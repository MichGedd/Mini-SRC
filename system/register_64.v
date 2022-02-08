module register_64(input [63:0] D, output [63:0] Q, input clr, input clk, input write);

	register_32 low32(.D (D[31:0]),
		.Q (Q[31:0]),
		.clr (clr),
		.clk (clk),
		.write (write));
	
	register_32 high32(.D (D[63:32]),
		.Q (Q[63:32]),
		.clr (clr),
		.clk (clk),
		.write (write));

endmodule
