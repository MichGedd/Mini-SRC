module multiplier_32 (input [31:0] in_x, input [31:0] in_y, output [63:0] out_product);

	wire [63:0] w_partial_products [0:15];
	wire [63:0] w_partial_products_shift [0:15];
	wire [63:0] w_reduce_16_to_8 [0:7];
	wire [63:0] w_reduce_8_to_4 [0:3];
	wire [63:0] w_reduce_4_to_2 [0:1];
	wire [32:0] w_padded_multiplicand;
	
	assign w_padded_multiplicand = {in_y, 1'b0};
	
	genvar i;
	
	// This is SO MUCH Combinational Logic. Maximum Clock is ~88.7MHz. Might not be an issue as FPGA clock is 50MHz, but we can try to optimize this later.
	// I think the booth recode step might be the longest (due to two cla_16 in ripple)?
	generate
		for(i = 0; i < 16; i = i + 1) begin : gen_partial_products
			bit_pair_recoder_32 recoder(.in_multiplier (in_x),
				.in_bit_pair (w_padded_multiplicand[(i*2)+2:(i*2)]),
				.out_recode (w_partial_products[i]));
				
			assign w_partial_products_shift[i] = {w_partial_products[i][63-(2*i):0], {(2*i){1'b0}}};
		end
				
		for(i = 0; i < 4; i = i + 1) begin : gen_reduce_16_to_8
			reducer_64 reducer(.in_w (w_partial_products_shift[4*i]),
				.in_x (w_partial_products_shift[(4*i)+1]),
				.in_y (w_partial_products_shift[(4*i)+2]),
				.in_z (w_partial_products_shift[(4*i)+3]),
				.in_cin (1'b0),
				.out_sum (w_reduce_16_to_8[2*i]),
				.out_carry (w_reduce_16_to_8[(2*i)+1]),
				.out_cout ());
		end
		
		for(i = 0; i < 2; i = i + 1) begin : gen_reduce_8_to_4
			reducer_64 reducer(.in_w (w_reduce_16_to_8[4*i]),
				.in_x (w_reduce_16_to_8[(4*i)+1]),
				.in_y (w_reduce_16_to_8[(4*i)+2]),
				.in_z (w_reduce_16_to_8[(4*i)+3]),
				.in_cin (1'b0),
				.out_sum (w_reduce_8_to_4[2*i]),
				.out_carry (w_reduce_8_to_4[(2*i)+1]),
				.out_cout ());  
		end
		
		reducer_64 reducer(.in_w (w_reduce_8_to_4[0]),
			.in_x (w_reduce_8_to_4[1]),
			.in_y (w_reduce_8_to_4[2]),
			.in_z (w_reduce_8_to_4[3]),
			.in_cin (1'b0),
			.out_sum (w_reduce_4_to_2[0]),
			.out_carry (w_reduce_4_to_2[1]),
			.out_cout ());
		
		cla_64 adder(.in_x (w_reduce_4_to_2[0]),
			.in_y (w_reduce_4_to_2[1]),
			.in_carry (1'b0),
			.out_sum (out_product),
			.out_carry (),
			.out_propogate (),
			.out_generate ());
			
	endgenerate

endmodule

//------------------------
module multiplier_32_tb;

	reg[31:0] in_x, in_y;
	wire[63:0] out;
	
	parameter delay = 10;
	
	multiplier_32 DUT(.in_x (in_x),
		.in_y (in_y),
		.out_product (out));
	
	initial begin
		in_x = 32'h0; in_y = 32'h0;
		#(delay) in_x = 32'ha; in_y = 32'ha;  // 10 x 10; Output should be 100
		#(delay) in_x = 32'h61; in_y = 32'h56;  // 96 x 86; Output should be 8342
		#(delay) in_x = 32'hFFFFFFF3; in_y = 32'hB;  // -13 x 11; Output should be -143
	end

endmodule


