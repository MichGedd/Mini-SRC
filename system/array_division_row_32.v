module array_division_row_32 (input [31:0] in_x, input [31:0] in_y, input in_mode, output [31:0] out_result, output [31:0] out_y, output out_carry);

	wire [32:0] w_modes;
	wire [32:0] w_carries;
	
	assign w_carries[32] = w_modes[32];
	assign w_modes[0] = in_mode;
	assign out_carry = w_carries[0];
	
	genvar i;
	
	generate
		for (i = 0; i < 32; i = i + 1) begin : gen_array_cells
			array_division_cell adc (.in_x (in_x[31-i]),
				.in_y (in_y[31-i]),
				.in_mode (w_modes[i]),
				.in_carry (w_carries[i+1]),
				.out_sum (out_result[31-i]),
				.out_y (out_y[31-i]),
				.out_mode (w_modes[i+1]),
				.out_carry (w_carries[i]));
		end
	endgenerate

endmodule

//---------------------------

module array_division_row_32_tb;

	reg [31:0] in_x, in_y;
	reg in_mode;
	wire [31:0] out_result, out_y;
	wire out_carry;
	
	parameter delay = 10;
	
	array_division_row_32 DUT (.in_x (in_x),
		.in_y (in_y),
		.in_mode (in_mode),
		.out_result (out_result),
		.out_y (out_y),
		.out_carry (out_carry));
	
	initial begin
		// Testbench values taken from Problem Set 1 Q12
		in_x = 32'b01100; in_y = 32'b0111; in_mode = 1;  // out_result = 0101, out_carry = 1;
		#(delay) in_x = 32'b0101; in_y = 32'b0111; in_mode = 1; // out_result = 1110, out_carry = 1;
	end

endmodule

