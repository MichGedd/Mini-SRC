module array_division_block_8x32(input [38:0] in_x, input [31:0] in_y, input in_mode, output [31:0] out_result, output [7:0] out_quotient);

	wire [31:0] w_partial_quotient [0:7];
	
	assign out_result = w_partial_quotient[7];
	
	genvar i;
	generate
		for(i = 0; i < 8; i = i + 1) begin : gen_array_rows
			if(i == 0) begin
				array_division_row_32 row (.in_x (in_x[38:7]),
				.in_y (in_y),
				.in_mode (in_mode),
				.out_result (w_partial_quotient[i]),
				.out_carry (out_quotient[7-i]),
				.out_y ());
			end else begin
				array_division_row_32 row (.in_x ({w_partial_quotient[i-1][30:0], in_x[7-i]}),
				.in_y (in_y),
				.in_mode (out_quotient[7-(i-1)]),
				.out_result (w_partial_quotient[i]),
				.out_carry (out_quotient[7-i]),
				.out_y ());
			end
		end
	endgenerate

endmodule
