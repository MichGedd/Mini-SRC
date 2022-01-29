module full_adder (input in_x, input in_y, input in_carry, output out_sum, output out_carry);

	assign out_sum = in_x ^ in_y ^ in_carry;
	assign out_carry = (in_x & in_y) | ((in_x ^ in_y) & in_carry);

endmodule
