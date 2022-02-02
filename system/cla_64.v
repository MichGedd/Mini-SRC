module cla_64(input [63:0] in_x, input [63:0] in_y, input in_carry, output [63:0] out_sum, output out_carry, output out_generate, output out_propogate);

	wire [4:0] w_carry;
	wire [3:0] w_generate;
	wire [3:0] w_propogate;
	
	assign w_carry[0] = in_carry;
	assign out_carry = w_carry[4];
	
	genvar i;
	
	generate
		for(i = 0; i < 64; i = i + 16) begin: gen_cla_4
			cla_16 cla(.in_x (in_x[i+15:i]),
				.in_y (in_y[i+15:i]),
				.in_carry (w_carry[i/16]),
				.out_sum (out_sum[i+15:i]),
				.out_carry (),
				.out_generate (w_generate[i/16]),
				.out_propogate (w_propogate[i/16]));
		end
	endgenerate
	
	cla_logic cla(.in_carry (w_carry[0]),
		.in_generate (w_generate),
		.in_propogate (w_propogate),
		.out_carry (w_carry[4:1]),
		.out_generate (out_generate),
		.out_propogate (out_propogate));

endmodule
