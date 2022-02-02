module reducer_4_to_2 (input in_w, input in_x, input in_y, input in_z, input in_cin, output out_sum, output out_carry, output out_cout);

	// These are derived from Truth Table in Week 1 Slides
	assign out_sum = in_w ^ in_x ^ in_y ^ in_z ^ in_cin;
	assign out_cout = (in_w & in_x) | (in_y & in_z) | (in_x & in_z) | (in_w & in_z) | (in_w & in_y) | (in_x & in_y);
	assign out_carry = (in_w & in_x & in_y & in_z) | (in_cin & ~out_sum);

endmodule
