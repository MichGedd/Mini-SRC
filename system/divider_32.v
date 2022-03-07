module divider_32(input clk, 
	input in_reset,
	input [31:0] in_dividend,
	input [31:0] in_divisor,
	output [31:0] out_quotient,
	output [31:0] out_remainder);

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
		
	reg [31:0] r_a;
	reg [31:0] r_m;
	reg [31:0] r_q;
	reg [63:0] r_shift;
	
	assign out_quotient = r_q;
	assign out_remainder = r_a;

	//------------------------------------------------

	
	always @(posedge clk, posedge in_reset) begin
		if(in_reset) begin 
			r_a = 32'b0;
			r_q = w_pos_dividend;
			r_m = w_pos_divisor;
		end else begin
			r_shift = {r_a, r_q} << 1;
			r_a = r_shift[63:32];
			r_q = r_q << 1;
			r_a = r_a - r_m;
			
			if(r_a[31]) begin	// r_a is negative
				r_q[0] = 1'b0;
				r_a = r_a + r_m;
			end else begin  // r_a is positive
				r_q[0] = 1'b1;
			end
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
	reg reset;
	wire [31:0] quotient, remainder;
	
	parameter pos_pos = 0;
	parameter pos_neg = 1;
	parameter neg_pos = 2;
	parameter neg_neg = 3;
	
	integer count = 0;
	integer state;
	
	reg clk;
	
	divider_32 DUT ( .clk (clk),
		.in_reset (reset),
		.in_dividend (dividend),
		.in_divisor (divisor),
		.out_quotient (quotient),
		.out_remainder (remainder));
	
	initial begin
		state = 0;
		clk = 0;
		reset = 1;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		count = count + 1;
		reset = 0;
		if (count == 32) begin
			count = 0;
			state = state + 1;
			reset = 1;
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
