module register_32(input [31:0] D, output [31:0] Q, input clr, input clk, input write);

	reg [31:0] value_32;
	initial begin
		value_32 [31:0] = 'h00000000;
	end
	
	//clr is active low
	//active high clock signal
	//active high write
	
	always @ (posedge clk, negedge clr) //need to make this also triggered by clear
	if(clr == 0) begin
		value_32 [31:0] <= 'h00000000;
	end else if(write == 1) begin
		value_32 [31:0] <= D [31:0];
	end
	
	assign Q [31:0] = value_32 [31:0];
	
endmodule

//----------------------------

module register_32_tb;

	reg clockin, clearin, writein;
	reg [31:0] datain;
	wire [31:0] dataout;
	
	localparam period = 20;
	
	register_32 DUT (.D(datain), .Q(dataout), .clr(clearin), .clk(clockin), .write(writein));
	
	
	//Something is messed up right now
	
	
	initial
		begin
			
			//set the initial values:
			clockin = 0;
			clearin = 0;
			datain = 'h00000000;
			writein = 0;
			
			#period;
			
			//setting out clock cycles
			clockin = 1;
			#period;
			
			clockin = 0;
			writein = 1;
			datain = 'h11111111;
			#period;
			
			clockin = 1;
			#period;
			
			clockin = 0;
			clearin = 1;
			#period;
			
			clockin = 1;
			#period;
			
			clockin = 0;
			datain = 'h11110000;
			writein = 0;
			#period;
			
			clockin = 1;
			#period;
			
			clockin = 0;
			writein = 1;
			#period;
			
			clockin = 1;
			#period;
			
			clockin = 0;
			clearin = 0;
			#period;
			
			clockin = 1;
			#period;
		end
endmodule



