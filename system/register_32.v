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
	end else if(write == 1 && clk == 1) begin
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
	
	parameter Default = 4'b0000;
	parameter s_clear = 4'b0001;
	parameter s_write_1 = 4'b0010;
	parameter s_write_2 = 4'b0011;
	
	reg [3:0] Present_state = Default;
	
	initial
		begin
			clockin = 0;
			forever #10 clockin = ~clockin;
	end
	
	always @ (posedge clockin)
	begin
		case (Present_state)
			Default: Present_state = s_write_1;
			s_write_1: Present_state = s_write_2;
			s_write_2: Present_state = s_clear;
			s_clear : Present_state = s_write_1;
		endcase
	end
	
	always @ (Present_state)
	begin
		case(Present_state)
			Default: begin
				datain = 'h00000000;
				writein = 0;
				clearin = 1;
			end
			s_write_1: begin
				clearin = 1;
				datain = 'h11111111;
				writein = 1;
			end
			s_write_2: begin
				clearin = 1;
				writein = 1;
				datain = 'h11110000;
			end
			s_clear: begin
				#10; //this is just to show the change in the previous state. 
				clearin = 0;
			end
		endcase
	end
	
	/*
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
			*/
endmodule



