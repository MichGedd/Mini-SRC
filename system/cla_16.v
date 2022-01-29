module cla_16(input [15:0] in_x, input [15:0] in_y, input in_carry, output [15:0] out_sum, output out_carry, output out_generate, output out_propogate);

	wire [4:0] w_carry;
	wire [3:0] w_generate;
	wire [3:0] w_propogate;
	
	assign w_carry[0] = in_carry;
	assign out_carry = w_carry[4];
	
	genvar i;
	
	generate
		for(i = 0; i < 16; i = i + 4) begin: gen_cla_4
			cla_4 cla(.in_x (in_x[i+3:i]),
				.in_y (in_y[i+3:i]),
				.in_carry (w_carry[i/4]),
				.out_sum (out_sum[i+3:i]),
				.out_carry (),
				.out_generate (w_generate[i/4]),
				.out_propogate (w_propogate[i/4]));
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

module cla_16_tb;

	reg [15:0] in_x, in_y;
	reg in_carry;
	
	wire [15:0] out_sum;
	wire out_generate, out_propogate, out_carry;
	
	parameter delay = 10;
	
	cla_16 DUT(.in_x (in_x),
		.in_y (in_y),
		.in_carry (in_carry),
		.out_sum (out_sum),
		.out_carry (out_carry),
		.out_generate (out_generate),
		.out_propogate (out_propogate));
		
	initial begin
		in_x = 16'h0000; in_y = 16'h0000; in_carry = 0;
		
		#(delay) in_x = 16'h0001; in_y = 16'h000F; in_carry = 0; // 1 + 15
		#(delay) in_x = 16'h0000; in_y = 16'h0000; in_carry = 0; // 0 + 0
		#(delay) in_x = 16'h0001; in_y = 16'h000F; in_carry = 1; // 1 + 15 + 1
		#(delay) in_x = 16'h0000; in_y = 16'h0000; in_carry = 0; // 0 + 0
		#(delay) in_x = 16'hFFFF; in_y = 16'h0002; in_carry = 0; // -1 + 2
		#(delay) in_x = 16'h0000; in_y = 16'h0000; in_carry = 0; // 0 + 0
		#(delay) in_x = 16'hFFFF; in_y = 16'h0002; in_carry = 1; // -1 + 2 + 1
		#100;
	end

endmodule
