module divider_32(input [31:0] in_dividend, input [31:0] in_divisor, output reg [31:0] out_quotient, output reg [31:0] out_remainder);
	
	reg [31:0] w_pos_divisor;
	reg [31:0] w_pos_dividend;
	wire [31:0] w_inv_divisor;
	wire [31:0] w_inv_dividend;
	
	wire [31:0] w_pos_quotient;
	wire [31:0] w_neg_quotient;
	
	wire [63:0] w_dividend_sign_extend = {{32{1'b0}}, w_pos_dividend};
	wire [31:0] w_partial_quotient [0:31];
	wire [32:0] w_mode_carry;
	wire [31:0] w_sum;
	
	assign w_mode_carry[32] = 1'b1;
	assign w_pos_quotient = w_mode_carry[31:0];
	
	genvar i;
	
	adder_32 adder (.in_x (w_partial_quotient[31]),
		.in_y (w_pos_divisor),
		.in_carry (1'b0),
		.out_sum (w_sum),
		.out_carry ());
	
	// These two adders are for negating dividend and divisor
	adder_32 invert_dividend (.in_x (~in_dividend),
		.in_y (32'b0),
		.in_carry (1'b1),
		.out_sum (w_inv_dividend),
		.out_carry ());
	
	adder_32 invert_divisor (.in_x (~in_divisor),
		.in_y (32'b0),
		.in_carry (1'b1),
		.out_sum (w_inv_divisor),
		.out_carry ());
	
	// This adder is for inverting the output to match signs
	adder_32 invert_quotient (.in_x (~w_pos_quotient),
		.in_y (32'b0),
		.in_carry (1'b1),
		.out_sum (w_neg_quotient),
		.out_carry ());
	
	generate
		for(i = 0; i < 32; i = i + 1) begin : gen_division_rows
			if (i == 0) begin
				array_division_row_32 row ( .in_x (w_dividend_sign_extend[62:31]),
				.in_y (w_pos_divisor),
				.in_mode (w_mode_carry[32-i]),
				.out_result (w_partial_quotient[i]),
				.out_y (),
				.out_carry (w_mode_carry[31-i]));
			end else begin
				array_division_row_32 row ( .in_x ({w_partial_quotient[i-1][30:0], w_dividend_sign_extend[31-i]}),
				.in_y (w_pos_divisor),
				.in_mode (w_mode_carry[32-i]),
				.out_result (w_partial_quotient[i]),
				.out_y (),
				.out_carry (w_mode_carry[31-i]));
			end
		end
	
	endgenerate
	
	always @(*) begin
		case (w_mode_carry[0])
			1'b0: out_remainder = w_sum;
			1'b1:	out_remainder = w_partial_quotient[31];
			default: out_remainder = 32'b0;
		endcase
		
		case (in_divisor[31])
			1'b0: w_pos_divisor = in_divisor;
			1'b1: w_pos_divisor = w_inv_divisor;
			default: w_pos_divisor = 32'b0;
		endcase
		
		case (in_dividend[31])
			1'b0: w_pos_dividend = in_dividend;
			1'b1: w_pos_dividend = w_inv_dividend;
			default w_pos_dividend = 32'b0;
		endcase
		
		case (in_divisor[31] ^ in_dividend[31])
			1'b0: out_quotient = w_pos_quotient;
			1'b1: out_quotient = w_neg_quotient;
			default out_quotient = 32'b0;
		endcase
	end

endmodule

//------------------------------

module divider_32_tb;

	reg [31:0] dividend, divisor;
	wire [31:0] quotient, remainder;
	
	parameter delay = 10;
	
	divider_32 DUT (.in_dividend (dividend),
		.in_divisor (divisor),
		.out_quotient (quotient),
		.out_remainder (remainder));
		
	initial begin
		//dividend = 32'b1100111; divisor = 32'b0111;
		dividend = 32'ha; divisor = 32'h1;  // 10 div 1;  Output should be 10R1
		#(delay) dividend = 32'h1e; divisor = 32'h4;  // 30 div 4; Output should be 7R2 
		#(delay) dividend = 32'ha; divisor = 32'hFFFFFFFD;  // 10 div -3. Output should be -3R1
		#(delay) dividend = 32'hFFFFFE0C; divisor = 32'h3;  // -500 div 3. Output should be -166R2
		#(delay) dividend = 32'hFFFFFF9C; divisor = 32'hFFFFFFF7;  // -100 div -9. Output should be 11R1
	end

endmodule
