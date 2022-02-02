module decoder_4_to_16 (
	input [3:0] in_4,
	output [15:0] out_16,
	input in_enable
);

reg [15:0] decode_out;

always @ (in_enable or in_4)
begin
	decode_out = 0;
	if(in_enable) begin
		case (in_4 [3:0])
			4'h0 : decode_out <= 16'h0001;
			4'h1 : decode_out <= 16'h0002;
			4'h2 : decode_out <= 16'h0004;
			4'h3 : decode_out <= 16'h0008;
			4'h4 : decode_out <= 16'h0010;
			4'h5 : decode_out <= 16'h0020;
			4'h6 : decode_out <= 16'h0040;
			4'h7 : decode_out <= 16'h0080;
			4'h8 : decode_out <= 16'h0100;
			4'h9 : decode_out <= 16'h0200;
			4'hA : decode_out <= 16'h0400;
			4'hB : decode_out <= 16'h0800;
			4'hC : decode_out <= 16'h1000;
			4'hD : decode_out <= 16'h2000;
			4'hE : decode_out <= 16'h4000;
			4'hF : decode_out <= 16'h8000;
		endcase
	end
	
end

assign out_16 [15:0] = decode_out [15:0];

endmodule

//-----------------------------

module decoder_4_to_16_tb;

reg enable;
reg [3:0] in_4;
wire [15:0] out_16;

localparam period = 20;

decoder_4_to_16 DUT (
	.in_4(in_4),
	.out_16(out_16),
	.in_enable(enable)
);

initial begin
	enable = 'b1;
	in_4 = 'b0000;
	#period
	in_4 = 'b0001;
	#period
	in_4 = 'b0010;
	#period
	in_4 = 'b0011;
	#period
	in_4 = 'b0100;
	#period
	in_4 = 'b0101;
	#period
	in_4 = 'b0110;
	#period
	in_4 = 'b0111;
	#period
	in_4 = 'b1000;
	#period
	in_4 = 'b1001;
	#period
	in_4 = 'b1010;
	#period
	in_4 = 'b1011;
	#period
	in_4 = 'b1100;
	#period
	in_4 = 'b1101;
	#period
	in_4 = 'b1111;
end

endmodule


