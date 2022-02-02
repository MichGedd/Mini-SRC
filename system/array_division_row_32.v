module array_division_row_32 (input [31:0] in_x, input [31:0] in_y, input in_mode, output [31:0] out_result, output [31:0] out_y, output out_carry);

	// Currently this is essentially just a 32-bit RCA with with carry_in as in_mode, a_in as in_y ^ mode_in, and b_in as in_x.
	// In theory we could change this to a 32-bit CLA, reducing carry propogation from 32^2 to 32
	// Update: Changed to adder 32. See comments above on why.
	// Update 2: The divisor circuit takes forever to compile with these new changes, but I'm not sure why? Compiling divider_32 takes 4 min.
	
	wire [31:0] w_in_mode_extend = {32{in_mode}};
	
	assign out_y = in_y;
	
	
	adder_32 adder(.in_x (in_x),
		.in_y (in_y ^ w_in_mode_extend),
		.in_carry (in_mode),
		.out_sum (out_result),
		.out_carry (out_carry));

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

