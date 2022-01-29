module alu_32(input [31:0] in_a, input [31:0] in_b, input [3:0] in_opcode, output reg [63:0] out_result);

	// ALU opcodes
	/*
	0000 = add
	0001 = sub
	0010 = rotate right
	0011 = rotate left
	0100 = shift right
	0101 = shift left
	0110 = and
	0111 = or
	1000 = mul
	1001 = div
	1010 = neg
	1011 = not
	*/

	wire [31:0] w_and_32;
	wire [31:0] w_or_32;
	wire [31:0] w_not_32;
	wire [31:0] w_adder_sum_out;
	wire [31:0] w_shift_rot_out;
	wire [63:0] w_multi_product_out;
	wire w_adder_carry_out;
	
	reg [31:0] r_a_pre_process;
	reg [31:0] r_b_pre_process;
	reg [31:0] r_a_inverted;
	reg r_adder_carry_in;
	
	assign w_and_32 = in_a & in_b;
	assign w_or_32 = in_a | in_b;
	assign w_not_32 = ~r_a_pre_process;
	
	adder_32 adder(.in_x (r_a_inverted),
		.in_y (r_b_pre_process),
		.in_carry (r_adder_carry_in),
		.out_sum (w_adder_sum_out),
		.out_carry (w_adder_carry_out));
	
	shift_rot_32 shifter (.in_x (in_a),
		.in_y (in_b),
		.in_left (in_opcode[0]),
		.in_rot (in_opcode[1]),
		.out (w_shift_rot_out));
	
	multiplier_32 multiplier (.in_x (in_a),
		.in_y (in_b),
		.out_product (w_multi_product_out));
	
	always @(*) begin
		case(in_opcode)  // Pre-process step before addition/not
			4'b1010,
			4'b1011 : begin
				r_b_pre_process = 0;  // If op code is NOT or INV set b to 0
				r_a_pre_process = in_a;
			end
			4'b0001 : begin  // If op code is SUB swap A and B
				r_a_pre_process = in_b;
				r_b_pre_process = in_a;
			end
			default : begin  // Default is pass values through
				r_a_pre_process = in_a;
				r_b_pre_process = in_b;
			end
		endcase
		
		case(in_opcode)  // Whether to invert A
			4'b0001,  // If op code is SUB, NOT, or INV, do a bitwise NOT to a
			4'b1010,
			4'b1011 : r_a_inverted = w_not_32;
			default : r_a_inverted = r_a_pre_process;  // Default is pass through values
		endcase
		
		case(in_opcode)
			4'b0001,
			4'b1010 : r_adder_carry_in = 1;
			default : r_adder_carry_in = 0;
		endcase
		
		case(in_opcode)  // Final ALU output
			4'b0000,
			4'b0001,
			4'b1010,
			4'b1011 : out_result = w_adder_sum_out;  // ADD, SUB, NOT, INV
			4'b0010,
			4'b0011,
			4'b0100,
			4'b0101 : out_result = w_shift_rot_out;  // ROR, ROL, SHR, SHL
			4'b0110 : out_result = w_and_32;  // AND
			4'b0111 : out_result = w_or_32;  // OR
			4'b1000 : out_result = w_multi_product_out;  // MUL
			default : out_result = 0;
		endcase
	end
	
endmodule

// --------------------------

module alu_32_tb;
	reg [31:0] in_a, in_b;
	reg [3:0] opcode;
	wire [63:0] alu_out;
	
	parameter delay = 10;
	
	alu_32 DUT(.in_a (in_a),
		.in_b (in_b),
		.in_opcode (opcode),
		.out_result (alu_out));
	
	initial begin
		in_a = 32'h0; in_b = 32'h0; opcode = 4'b0;
		#(delay) in_a = 32'hF0F0F0F0; in_b = 32'hABCDABCD; opcode = 4'b1011; // NOT
		#(delay) in_a = 32'h0; in_b = 32'h0; opcode = 4'b0;
		#(delay) in_a = 32'h00000001; in_b = 32'hFFFFFFFF; opcode = 4'b1010; // NEG
		#(delay) in_a = 32'h0; in_b = 32'h0; opcode = 4'b0;
		#(delay) in_a = 32'hFFFFFFFF; in_b = 32'h0F0F0F0F; opcode = 4'b0110;  // AND
		#(delay) in_a = 32'h0; in_b = 32'h0; opcode = 4'b0;
		#(delay) in_a = 32'hF0F0F0F0; in_b = 32'h0F0F0F0F; opcode = 4'b0111;  // OR
		#(delay) in_a = 32'h0; in_b = 32'h0; opcode = 4'b0;
		#(delay) in_a = 32'h0000FFFF; in_b = 32'h00000001; opcode = 4'b0000;  // ADD
		#(delay) in_a = 32'h0; in_b = 32'h0; opcode = 4'b0;
		#(delay) in_a = 32'h0000FFFF; in_b = 32'h000000FF; opcode = 4'b0001;  // SUB
		#(delay) in_a = 32'h0; in_b = 32'h0; opcode = 4'b0;
		#(delay) in_a = 32'hFFFFFFF3; in_b = 32'hB; opcode = 4'b1000;  // MUL
	end
	
endmodule

