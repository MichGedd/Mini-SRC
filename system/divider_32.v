module divider_32(input clk, input [31:0] in_dividend, input [31:0] in_divisor, output reg [31:0] out_quotient, output reg [31:0] out_remainder);

	// We have been hoodwinked, bamboozled, led astray, run amuck, and flat out decieved by the ELEC 374 lecture slides.
	// Lecture Slides: Array Divisors are super fast y'all
	// Our Array Divisor: UwU 4.01 MHz max clock pweez 
	//----------------------------------------------------------------------
	// In all seriousness we really need to reduce the ammount of logic here.
	// Biggest issue is probably the fact that the carry/mode gets propogated between rows and
	// has 32^2 full adders to pass through until we get to the bottom.....
	
	// Update: Switching from RCA to two cla_16 in ripple managed to increase clock speed from 4.01 MHz to 6.0 MHz, a 50% speed up. Maybe see if we can use booth's to reduce to 16 layers rather than 32?
	
	wire [31:0] w_array_block_out [0:3];
	wire [31:0] w_quotients;
	wire [31:0] w_sum;
	
	reg [38:0] r_array_block_in [0:2];
	reg r_in_mode [0:2];
	
	integer i;
	
	array_division_block_8x32 block1 (.in_x ({31'b0, in_dividend[31:24]}),
		.in_y (in_divisor),
		.in_mode (1'b1),
		.out_result (w_array_block_out[0]),
		.out_quotient (w_quotients[31:24]));
		
	array_division_block_8x32 block2 (.in_x (r_array_block_in[0]),
		.in_y (in_divisor),
		.in_mode (r_in_mode[0]),
		.out_result (w_array_block_out[1]),
		.out_quotient (w_quotients[23:16]));
		
	array_division_block_8x32 block3 (.in_x (r_array_block_in[1]),
		.in_y (in_divisor),
		.in_mode (r_in_mode[1]),
		.out_result (w_array_block_out[2]),
		.out_quotient (w_quotients[15:8]));
		
	array_division_block_8x32 block4 (.in_x (r_array_block_in[2]),
		.in_y (in_divisor),
		.in_mode (r_in_mode[2]),
		.out_result (w_array_block_out[3]),
		.out_quotient (w_quotients[7:0]));
	
	adder_32 restore (.in_x (w_array_block_out[3]),
		.in_y (in_divisor),
		.in_carry (1'b0),
		.out_sum (w_sum),
		.out_carry ());
	
	always @(posedge clk) begin
		for (i = 0; i < 3; i = i + 1) begin
			r_in_mode[i] <= w_quotients[24 - (8*i)];
		end
		
		r_array_block_in[0] <= {w_array_block_out[0][30:0], in_dividend[23:16]};
		r_array_block_in[1] <= {w_array_block_out[1][30:0], in_dividend[15:8]};
		r_array_block_in[2] <= {w_array_block_out[2][30:0], in_dividend[7:0]};
		out_quotient <= w_quotients;
		
		if (w_quotients[0] == 0) begin 
			out_remainder <= w_sum;
		end else begin
			out_remainder <= w_array_block_out[3];
		end
	end

endmodule

//------------------------------

module divider_32_tb;

	/*reg [31:0] dividend, divisor;
	wire [31:0] quotient, remainder;
	
	parameter delay = 10;
	
	divider_32 DUT (.in_dividend (dividend),
		.in_divisor (divisor),
		.out_quotient (quotient),
		.out_remainder (remainder));
		
	initial begin
		//dividend = 32'b1100111; divisor = 32'b0111;
		dividend = 32'ha; divisor = 32'h1;  // 10 div 1;  Output should be 10R0
		#(delay) dividend = 32'h1e; divisor = 32'h4;  // 30 div 4; Output should be 7R2 
		#(delay) dividend = 32'ha; divisor = 32'hFFFFFFFD;  // 10 div -3. Output should be -3R1
		#(delay) dividend = 32'hFFFFFE0C; divisor = 32'h3;  // -500 div 3. Output should be -166R2
		#(delay) dividend = 32'hFFFFFF9C; divisor = 32'hFFFFFFF7;  // -100 div -9. Output should be 11R1
	end*/
	
	reg [31:0] dividend, divisor;
	wire [31:0] quotient, remainder;
	
	reg clk;
	
	divider_32 DUT ( .clk (clk),
		.in_dividend (dividend),
		.in_divisor (divisor),
		.out_quotient (quotient),
		.out_remainder (remainder));
	
	initial begin
		dividend = 32'h1e; divisor = 32'h4;
		
		clk = 0;
		forever #10 clk = ~clk;
	end
	

endmodule
