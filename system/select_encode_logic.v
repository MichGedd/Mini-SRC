module select_encode_logic(input in_gra,
	input in_grb,
	input in_grc,
	input in_read,
	input in_write,
	input in_base_addr_read,
	input [31:0] in_ir,
	output [3:0] out_regfile_location,
	output out_regfile_read,
	output out_regfile_write);

	assign out_regfile_read = in_read | in_base_addr_read;
	assign out_regfile_write = in_write;
	assign out_regfile_location = (in_ir[26:23] & {4{in_gra}}) | (in_ir[22:19] & {4{in_grb}}) | (in_ir[18:15] & {4{in_grc}});
	

endmodule
