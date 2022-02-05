module divider_32(input clk, input [31:0] in_dividend, input [31:0] in_divisor, output reg [31:0] out_quotient, output reg [31:0] out_remainder);

	// We have been hoodwinked, bamboozled, led astray, run amuck, and flat out decieved by the ELEC 374 lecture slides.
	// Lecture Slides: Array Divisors are super fast y'all
	// Our Array Divisor: UwU 4.01 MHz max clock pweez 
	//----------------------------------------------------------------------
	// In all seriousness we really need to reduce the ammount of logic here.
	// Biggest issue is probably the fact that the carry/mode gets propogated between rows and
	// has 32^2 full adders to pass through until we get to the bottom.....
	
	// Update: Switching from RCA to two cla_16 in ripple managed to increase clock speed from 4.01 MHz to 6.0 MHz, a 50% speed up. Maybe see if we can use booth's to reduce to 16 layers rather than 32?
	
	reg [31:0] w_pos_divisor;
	reg [31:0] w_pos_dividend;
	wire [31:0] w_inv_divisor;
	wire [31:0] w_inv_dividend;
	wire [31:0] w_pos_quotient;
	wire [31:0] w_neg_quotient;
	
	wire [31:0] w_array_block_out [0:3];
	wire [31:0] w_sum;
	
	reg [38:0] r_array_block_in [0:2];
	reg r_in_mode [0:2];
	
	integer i;
	
	// Adders to normalize divisor/dividend/quotient to positive
	adder_32 invert_dividend (.in_x (~in_dividend),
		.in_y (32'b0),
		.in_carry (1'b1),
		.out_sum (w_inv_dividend),
		.out_carry ());
	
	adder_32 invert_divisor (.in_x (~in_divisor),
		.in_y (32'b0),
		.in_carry (1'b1),
		.out_sum (w_inv_divisor),
		.out_carry ());
		
	adder_32 invert_quotient (.in_x (~w_pos_quotient),
		.in_y (32'b0),
		.in_carry (1'b1),
		.out_sum (w_neg_quotient),
		.out_carry ());
	//------------------------------------------------
	
	array_division_block_8x32 block1 (.in_x ({31'b0, w_pos_dividend[31:24]}),
		.in_y (w_pos_divisor),
		.in_mode (1'b1),
		.out_result (w_array_block_out[0]),
		.out_quotient (w_pos_quotient[31:24]));
		
	array_division_block_8x32 block2 (.in_x (r_array_block_in[0]),
		.in_y (w_pos_divisor),
		.in_mode (r_in_mode[0]),
		.out_result (w_array_block_out[1]),
		.out_quotient (w_pos_quotient[23:16]));
		
	array_division_block_8x32 block3 (.in_x (r_array_block_in[1]),
		.in_y (w_pos_divisor),
		.in_mode (r_in_mode[1]),
		.out_result (w_array_block_out[2]),
		.out_quotient (w_pos_quotient[15:8]));
		
	array_division_block_8x32 block4 (.in_x (r_array_block_in[2]),
		.in_y (w_pos_divisor),
		.in_mode (r_in_mode[2]),
		.out_result (w_array_block_out[3]),
		.out_quotient (w_pos_quotient[7:0]));
	
	adder_32 restore (.in_x (w_array_block_out[3]),
		.in_y (w_pos_divisor),
		.in_carry (1'b0),
		.out_sum (w_sum),
		.out_carry ());
	
	always @(posedge clk) begin
		for (i = 0; i < 3; i = i + 1) begin
			r_in_mode[i] <= w_pos_quotient[24 - (8*i)];
		end
		
		r_array_block_in[0] <= {w_array_block_out[0][30:0], w_pos_dividend[23:16]};
		r_array_block_in[1] <= {w_array_block_out[1][30:0], w_pos_dividend[15:8]};
		r_array_block_in[2] <= {w_array_block_out[2][30:0], w_pos_dividend[7:0]};
		
				
		if (in_divisor[31] ^ in_dividend[31] == 0) begin
			out_quotient = w_pos_quotient;
		end else begin 
			out_quotient = w_neg_quotient;
		end
		
		if (w_pos_quotient[0] == 0) begin 
			out_remainder <= w_sum;
		end else begin
			out_remainder <= w_array_block_out[3];
		end
	end
	
	always @(*) begin
		if (in_divisor[31] == 0) begin
			w_pos_divisor = in_divisor;
		end else begin
			w_pos_divisor = w_inv_divisor;
		end
		
		if (in_dividend[31] == 0) begin
			w_pos_dividend = in_dividend;
		end else begin
			w_pos_dividend = w_inv_dividend;
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
	
	parameter pos_pos = 0;
	parameter pos_neg = 1;
	parameter neg_pos = 2;
	parameter neg_neg = 3;
	
	integer count = 0;
	integer state;
	
	reg clk;
	
	divider_32 DUT ( .clk (clk),
		.in_dividend (dividend),
		.in_divisor (divisor),
		.out_quotient (quotient),
		.out_remainder (remainder));
	
	initial begin
		state = 0;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		count = count + 1;
		
		if (count == 4) begin
			count = 0;
			state = state + 1;
		end
	end
	
	always @(state) begin
		case(state)
			pos_pos : begin
				dividend = 32'h1e; divisor = 32'h4;
			end
			pos_neg : begin
				dividend = 32'ha; divisor = 32'hFFFFFFFD;
			end
			neg_pos : begin
				dividend = 32'hFFFFFE0C; divisor = 32'h3;
			end
			neg_neg : begin
				dividend = 32'hFFFFFF9C; divisor = 32'hFFFFFFF7;
			end
		endcase
	end
	

endmodule
