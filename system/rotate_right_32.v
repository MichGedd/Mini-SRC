module rotate_right_32(input [31:0] in_x, input[4:0] in_rot, output reg [31:0] out_rotated);

	always @(*) begin
		case(in_rot)
			5'b00000: out_rotated = in_x;
			5'b00001: out_rotated = {in_x[0], in_x[31:1]};
			5'b00010: out_rotated = {in_x[1:0], in_x[31:2]};
			5'b00011: out_rotated = {in_x[2:0], in_x[31:3]};
			5'b00100: out_rotated = {in_x[3:0], in_x[31:4]};
			5'b00101: out_rotated = {in_x[4:0], in_x[31:5]};
			5'b00110: out_rotated = {in_x[5:0], in_x[31:6]};
			5'b00111: out_rotated = {in_x[6:0], in_x[31:7]};
			5'b01000: out_rotated = {in_x[7:0], in_x[31:8]};
			5'b01001: out_rotated = {in_x[8:0], in_x[31:9]};
			5'b01010: out_rotated = {in_x[9:0], in_x[31:10]};
			5'b01011: out_rotated = {in_x[10:0], in_x[31:11]};
			5'b01100: out_rotated = {in_x[11:0], in_x[31:12]};
			5'b01101: out_rotated = {in_x[12:0], in_x[31:13]};
			5'b01110: out_rotated = {in_x[13:0], in_x[31:14]};
			5'b01111: out_rotated = {in_x[14:0], in_x[31:15]};
			5'b10000: out_rotated = {in_x[15:0], in_x[31:16]};
			5'b10001: out_rotated = {in_x[16:0], in_x[31:17]};
			5'b10010: out_rotated = {in_x[17:0], in_x[31:18]};
			5'b10011: out_rotated = {in_x[18:0], in_x[31:19]};
			5'b10100: out_rotated = {in_x[19:0], in_x[31:20]};
			5'b10101: out_rotated = {in_x[20:0], in_x[31:21]};
			5'b10110: out_rotated = {in_x[21:0], in_x[31:22]};
			5'b10111: out_rotated = {in_x[22:0], in_x[31:23]};
			5'b11000: out_rotated = {in_x[23:0], in_x[31:24]};
			5'b11001: out_rotated = {in_x[24:0], in_x[31:25]};
			5'b11010: out_rotated = {in_x[25:0], in_x[31:26]};
			5'b11011: out_rotated = {in_x[26:0], in_x[31:27]};
			5'b11100: out_rotated = {in_x[27:0], in_x[31:28]};
			5'b11101: out_rotated = {in_x[28:0], in_x[31:29]};
			5'b11110: out_rotated = {in_x[29:0], in_x[31:30]};
			5'b11111: out_rotated = {in_x[30], in_x[31]};
			default: out_rotated = in_x;
		endcase
	end

endmodule

//------------------------
// TODO: Add testbench