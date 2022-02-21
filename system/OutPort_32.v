module OutPort_32(
	input in_clk, //clock in
	input in_clr,
	input in_enable, //This is the OutPortIn signal, just their naming is bad
	input [31:0] in_BusMuxOut, //the input data
	output[31:0] out_ToPort, //the output data
);