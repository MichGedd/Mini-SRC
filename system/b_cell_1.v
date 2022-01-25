module b_cell_1(input in_x, input in_y, input in_carry, output out_sum, output out_generate, output out_propogate);

	assign out_generate = in_x & in_y;
	assign out_propogate = in_x ^ in_y;
	assign out_sum = out_propogate ^ in_carry;

endmodule
