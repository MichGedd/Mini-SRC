module divider_32(input [31:0] in_dividend, input [31:0] in_divisor, output [31:0] out_quotient, output reg [31:0] out_remainder);

	wire [63:0] w_dividend_sign_extend = {{32{in_dividend[31]}}, in_dividend};
	wire [31:0] w_partial_quotient [0:32];
	wire [32:0] w_mode_carry;
	wire [31:0] w_sum;
	
	assign w_mode_carry[32] = 1'b1;
	assign out_quotient = w_mode_carry[31:0];
	assign w_partial_quotient[0] = w_dividend_sign_extend[63:32];
	
	genvar i;
	
	adder_32 adder (.in_x (w_partial_quotient[32]),
		.in_y (in_divisor),
		.in_carry (1'b0),
		.out_sum (w_sum),
		.out_carry ());
	
	generate
		for(i = 0; i < 32; i = i + 1) begin : gen_division_rows
			if (i == 0) begin
				array_division_row_32 row ( .in_x (w_partial_quotient[i]),
				.in_y (in_divisor),
				.in_mode (w_mode_carry[32-i]),
				.out_result (w_partial_quotient[i+1]),
				.out_y (),
				.out_carry (w_mode_carry[31-i]));
			end else  begin
				array_division_row_32 row ( .in_x ({w_partial_quotient[i][30:0], w_dividend_sign_extend[32-i]}),
				.in_y (in_divisor),
				.in_mode (w_mode_carry[32-i]),
				.out_result (w_partial_quotient[i+1]),
				.out_y (),
				.out_carry (w_mode_carry[31-i]));
			end
			
		end
	endgenerate
	
	always @(*) begin
		case (w_mode_carry[0])
			1'b0: out_remainder = w_sum;
			1'b1:	out_remainder = w_partial_quotient[32];
			default: out_remainder = 32'b0;
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
		dividend = 32'b1100111; divisor = 32'b0111;
		//dividend = 32'ha; divisor = 32'h1;  // 10 div 1;  Output should be 10R1
		//#(delay) dividend = 32'h1e; divisor = 32'h4;  // 30 div 4; Output should be 7R2 
	end

endmodule
