module adder_32(input [31:0] in_x, input [31:0] in_y, input in_carry, output [31:0] out_sum, output out_carry);

	wire w_cla_low_carry_out;

	cla_16 cla_low(.in_x(in_x[15:0]),
		.in_y(in_y[15:0]),
		.in_carry(in_carry),
		.out_sum(out_sum[15:0]),
		.out_carry(w_cla_low_carry_out),
		.out_generate(),
		.out_propogate());
		
	cla_16 cla_high(.in_x(in_x[31:16]),
		.in_y(in_y[31:16]),
		.in_carry(w_cla_low_carry_out),
		.out_sum(out_sum[31:16]),
		.out_carry(out_carry),
		.out_generate(),
		.out_propogate());

endmodule

//-------------------------------

// TODO: Add adder_32_tb