module alu_32(input [31:0] in_a, input [31:0] in_b, input [31:0] in_opcode, output reg [31:0] out_result);

	// ALU opcodes
	/*
	0000 = add
	0001 = sub
	0010 = shift right
	0011 = shift left
	0100 = rotate right
	0101 = rotate left
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
	
	reg [31:0] r_a_pre_process;
	reg [31:0] r_b_pre_process;
	reg [31:0] r_a_inverted;
	
	assign w_and_32 = in_a & in_b;
	assign w_or_32 = in_a | in_b;
	assign w_not_32 = !r_a_pre_process;
	
	always @(*) begin
		case(in_opcode)
			4'b1010,
			4'b1011 : r_b_pre_process = 0;
			4'b0001 : begin
				r_a_pre_process = in_b;
				r_b_pre_process = in_a;
			end
			
			default : begin
				r_a_pre_process = in_a;
				r_b_pre_process = in_b;
			end
		endcase
		
		case(in_opcode)
			4'b0001,
			4'b1010,
			4'b1011 : r_a_inverted = w_not_32;
			default : r_a_inverted = r_a_pre_process;
		endcase
		
		case(in_opcode)
			4'b0110 : out_result = w_and_32;
			4'b0111 : out_result = w_or_32;
			default : out_result = 0;
		endcase
	end
	
	
	
endmodule
