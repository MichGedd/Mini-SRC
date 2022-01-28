module bit_pair_recoder_32(input [31:0] in_multiplier, input [2:0] in_bit_pair, output reg [63:0] out_recode);

	wire [31:0] w_neg_multiplier;  // Output for -1
	wire [31:0] w_pos_shift;  // Output for +2
	wire [31:0] w_neg_shift;  // Output for -2
	
	adder_32 adder(.in_x (~in_multiplier),
		.in_y (32'b0),
		.in_carry (1'b1),
		.out_sum (w_neg_multiplier),
		.out_carry ());
	
	assign w_pos_shift = {in_multiplier[30:0], 1'b0};
	assign w_neg_shift = {w_neg_multiplier[30:0], 1'b0};

	always @(*) begin
		case(in_bit_pair)
			3'b000,
			3'b111:  out_recode = 64'b0;  // 0 x M
			3'b001,
			3'b010:  out_recode = {{32{in_multiplier[31]}}, in_multiplier};  // +1 x M
			3'b011:  out_recode = {{32{w_pos_shift[31]}}, w_pos_shift}; // +2 x M
			3'b100:  out_recode = {{32{w_neg_shift[31]}}, w_neg_shift}; // -2 x M
			3'b101,
			3'b110:  out_recode = {{32{w_neg_multiplier[31]}}, w_neg_multiplier}; // -1 x M
			default: out_recode = 64'b0;
		endcase
	end
endmodule

//----------------------------------

module bit_pair_recoder_32_tb;

	reg [31:0] multiplier;
	reg [2:0] bit_pair;
	wire [63:0] recode;
	
	parameter delay = 10;
	
	bit_pair_recoder_32 DUT(.in_multiplier (multiplier),
		.in_bit_pair (bit_pair),
		.out_recode (recode));
	
	initial begin
		multiplier = 32'hd; bit_pair = 3'b100;  // Input of 13. Output should be decimal -26
		#(delay) bit_pair = 3'b101;  // Output should be decimal -13
		#(delay) bit_pair = 3'b111;  // Output should be decimal 0
		#(delay) bit_pair = 3'b010;  // Output should be decimal 13
		#(delay) bit_pair = 3'b011;  // Output should be decimal 26
	end

endmodule
