module InPort_32(
	input in_clk, //clock in
	input in_clr,//clear in
	output [31:0] out_BusMuxIn, //the input data
	input [31:0] in_FromPort, //the output data
);