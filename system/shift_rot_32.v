module shift_rot_32 (input [31:0] in_x, input[31:0] in_y, input in_left, in_rot, output reg [31:0] out);

	// in_left = 1 -> shift/rot left
	// in_left = 0 -> shift/rot right
	// in_rot = 1 -> rotate
	// in_rot = 0 -> shift
	
	wire [4:0] w_rot_ammount;
	wire [31:0] w_rot_left;
	wire [31:0] w_rot_right;
	wire [31:0] w_mask;
	reg [31:0] r_rot_output;
	
	assign w_rot_ammount = in_y [4:0];  // Equivilant to in_y mod 32
	
	rotate_left_32 rot_left(.in_x (in_x),
		.in_rot (w_rot_ammount),
		.out_rotated (w_rot_left));
	
	rotate_right_32 rot_right(.in_x (in_x),
		.in_rot (w_rot_ammount),
		.out_rotated (w_rot_right));
		
	shift_mask_32 shift_mask(.in_shift (w_rot_ammount),
		.in_left (in_left),
		.out_mask (w_mask));
		
	always @(*) begin
		case(in_left)  // Decide which rotate to use
			1'b0: r_rot_output = w_rot_right;
			1'b1: r_rot_output = w_rot_left;
			default: r_rot_output = 32'b0;
		endcase
		
		case(in_rot)  // Decide whether shift or rotate
			1'b0: begin  // Shift
				if (in_y >= 32) begin
					out = 32'b0;
				end else begin
					out = r_rot_output & w_mask;
				end
			end
			1'b1: out = r_rot_output;  // Rotate
			default: out = 32'b0;
		endcase
	end

endmodule

// -----------------------------------

module shift_rot_32_tb;

	reg [31:0] in_x, in_y;
	reg in_left, in_rot;
	wire [31:0] out;
	
	parameter delay = 10;
	
	shift_rot_32 DUT (.in_x (in_x),
		.in_y (in_y),
		.in_left (in_left),
		.in_rot (in_rot),
		.out (out));
	
	initial begin
		in_x = 32'hF000000F; in_y = 32'h0; in_left = 0; in_rot = 0;  // Dont rotate; Output should be f000000f;
		
		#(delay) in_y = 32'h3; in_left = 0; in_rot = 1; // rot right 3; Output should be FE000001
		#(delay) in_y = 32'h3; in_left = 0; in_rot = 0; // shift right 3; Output should be 1E000001
		#(delay) in_y = 32'h3; in_left = 1; in_rot = 1; // rot left 3; Output should be 8000007F;
		#(delay) in_y = 32'h3; in_left = 1; in_rot = 0; // shift left 3; Output should be 80000078;
		#(delay) in_y = 32'h23; in_left = 1; in_rot = 1; // Check if modulo function works; Output should be 8000007F;
		#(delay) in_y = 32'hFF; in_left = 1; in_rot = 0; // Check if shift > 31; Output should be 00000000;
		
	end

endmodule

