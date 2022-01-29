module array_division_cell(input in_x, input in_y, input in_mode, input in_carry, output out_sum, output out_y, output out_mode, output out_carry);

	assign out_y = in_y;
	assign out_mode = in_mode;
	
	full_adder fa(.in_x (in_x),
		.in_y ((in_y ^ in_mode)),
		.in_carry (in_carry),
		.out_sum (out_sum),
		.out_carry (out_carry));

endmodule
