module cla_4(input [3:0] in_x, input [3:0] in_y, input in_carry, output [3:0] out_sum, output out_generate, output out_propogate);

	wire [4:0] w_carry;
	wire [3:0] w_generate;
	wire [3:0] w_propogate;
	
	assign w_carry[0] = in_carry;
	assign out_carry = w_carry[4];
	
	genvar i;
	
	generate
		for(i = 0; i < 4; i = i + 1) begin: gen_b_cell_1
			b_cell_1 bcell(.in_x (in_x[i]),
				.in_y (in_y[i]),
				.in_carry (w_carry[i]),
				.out_sum (out_sum[i]),
				.out_generate (w_generate[i]),
				.out_propogate (w_propogate[i]));
		end
	endgenerate
	
	cla_logic cla(.in_carry (w_carry[0]),
		.in_generate (w_generate),
		.in_propogate (w_propogate),
		.out_carry (w_carry[4:1]),
		.out_generate (out_generate),
		.out_propogate (out_propogate));

endmodule

//--------------------------

module cla_4_tb;

	reg [3:0] in_x, in_y;
	reg in_carry;
	
	wire [3:0] out_sum;
	wire out_generate, out_propogate, out_carry;
	
	parameter delay = 10;
	
	cla_4 DUT(.in_x (in_x),
		.in_y (in_y),
		.in_carry (in_carry),
		.out_sum (out_sum),
		.out_generate (out_generate),
		.out_propogate (out_propogate));
		
	initial begin
		in_x = 4'b0000; in_y = 4'b0000; in_carry = 0;
		
		#(delay) in_x = 4'b0100; in_y = 4'b0010; in_carry = 0; // 4 + 2
		#(delay) in_x = 4'b0000; in_y = 4'b0000; in_carry = 0; // 0 + 0
		#(delay) in_x = 4'b0100; in_y = 4'b0010; in_carry = 1; // 4 + 2 + 1
		#(delay) in_x = 4'b0000; in_y = 4'b0000; in_carry = 0; // 0 + 0
		#(delay) in_x = 4'b1111; in_y = 4'b0010; in_carry = 0; // -1 + 2
		#(delay) in_x = 4'b0000; in_y = 4'b0000; in_carry = 0; // 0 + 0
		#(delay) in_x = 4'b1111; in_y = 4'b0010; in_carry = 1; // -1 + 2 + 1
		#100;
	end

endmodule
