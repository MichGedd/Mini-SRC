// NOTE: This Verilog Code was taken from https://www.fpga4student.com/2017/08/verilog-code-for-clock-divider-on-fpga.html

// fpga4student.com: FPGA projects, VHDL projects, Verilog projects
// Verilog project: Verilog code for clock divider on FPGA
// Top level Verilog code for clock divider on FPGA

module clock_divider(input in_clock, output reg out_clock);

	initial begin
		out_clock = 0;
	end

	reg [27:0] counter= 28'd0;
	
	//parameter DIVISOR = 28'd10000000;
	parameter DIVISOR = 28'd3;
	
	always @(posedge in_clock) begin
		counter <= counter + 28'd1;
		if(counter == (DIVISOR - 1)) begin
			counter <= 28'd0;
		end
		
		out_clock <= (counter<DIVISOR/2)?1'b1:1'b0;
	end
endmodule

//-------------------------------

module clock_divider_tb;
	
	reg in;
	wire out;
	
	clock_divider DUT(.in_clock (in),
		.out_clock (out));
		
	initial begin
		in = 0;
		forever #10 in = ~in;
	end

endmodule
