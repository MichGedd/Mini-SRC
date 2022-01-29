module reducer_64 (input [63:0] in_w, input [63:0] in_x, input [63:0] in_y, input [63:0] in_z, input in_cin, output [63:0] out_sum, output [63:0] out_carry, output out_cout);

		wire [64:0] w_carries;
		wire [63:0] w_out_carry_pre_shift;  // remember to shift carry left 1 (multiply by two)
		
		assign w_carries[0] = in_cin;
		assign out_cout = w_carries[64];
		assign out_carry = {w_out_carry_pre_shift[62:0], 1'b0};  // No values lost as 32 bit number is expected input

		genvar i;
		
		generate
			for(i = 0; i < 64; i = i + 1) begin : gen_reducer_4_to_2
				reducer_4_to_2 reducer(.in_w (in_w[i]),
					.in_x (in_x[i]),
					.in_y (in_y[i]),
					.in_z (in_z[i]),
					.in_cin (w_carries[i]),
					.out_sum (out_sum[i]),
					.out_carry (w_out_carry_pre_shift[i]),
					.out_cout (w_carries[i+1]));
			end
		endgenerate

endmodule

//-------------------------

module reducer_64_tb;

	reg [63:0] w, x, y, z;
	reg cin;
	
	wire [63:0] sum, carry;
	wire cout;
	
	parameter delay = 10;
	
	reducer_64 DUT (.in_w(w),
		.in_x (x),
		.in_y (y),
		.in_z (z),
		.in_cin (cin),
		.out_sum (sum),
		.out_carry (carry),
		.out_cout (cout));
	
	initial begin
		w = 32'h3B9ACA00; x = 32'h3B9ACA00; y = 32'h4C4B40; z = 32'h4C4B40; cin = 1'b0;  // w and x = 1 billion, y and z = 5 million. Output should be 2.01 billion
	end

endmodule 