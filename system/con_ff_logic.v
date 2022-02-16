module con_ff_logic(input [1:0] in_condition, input [31:0] in_bus, input in_con_in, output out_branch);

	wire w_eq_zero;
	wire w_neq_zero;
	wire w_gte_zero;
	wire w_lt_zero;
	
	wire w_bus_reduce;
	
	wire w_con_ff_d;
	wire [31:0] w_con_ff_q;  // This is just here so we dont get a warning about mis-match size.
	
	assign w_eq_zero = ~in_condition[1] & ~in_condition[0];
	assign w_neq_zero = ~in_condition[1] & in_condition[0];
	assign w_gte_zero = in_condition[1] & ~in_condition[0];
	assign w_lt_zero = in_condition[1] & in_condition[0];
	assign w_bus_recudce = ~(|in_bus);
	assign w_con_ff_d = (w_bus_recudce & w_eq_zero) | (~w_bus_recudce & w_neq_zero) | (~in_bus[31] & w_gte_zero) | (in_bus[31] & w_lt_zero);
	assign out_branch = w_con_ff_q[0];
	
	register_32 CON(.D ({{31{1'b0}}, w_con_ff_d}), // This is just here so we dont get a warning about mis-match size.
		.Q (w_con_ff_q),
		.clr (1'b1),
		.clk (in_con_in),
		.write (1'b1));

endmodule
