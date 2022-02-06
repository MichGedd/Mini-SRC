module datapath(input clk,
	input [3:0] in_regfile_location,  // Location to read from
	input [3:0] in_alu_opcode, // ALU opcode
	input [31:0] in_mem_data,  // Emulated data from memory
	input in_reg_clear,  // Clear all registers
	input in_mdr_select, // This selects whether to read from bus or memory. 0 for bus, 1 for memory
	input in_inc_pc,
	// Read signals
	input in_regfile_read,
	input in_hi_read,
	input in_lo_read,
	input in_z_hi_read,
	input in_z_lo_read,
	input in_pc_read,
	input in_mdr_read,
	input in_inport_read,
	input in_c_read,
	// Write signals
	input in_regfile_write,
	input in_hi_write,
	input in_lo_write,
	input in_z_write,
	input in_pc_write,
	input in_mdr_write,
	input in_ir_write,
	input in_y_write,
	input in_mar_write,
	output [31:0] out_bus);  // Probably going to remove this one in the future. In the future datapath, control, and memory will all be parts of "system"

	reg [31:0] r_bus;  // This is the bus
	reg [31:0] w_pc_in;  // Input to PC
	
	wire [31:0] w_regfile_out;
	wire [31:0] w_PC_out;
	wire [31:0] w_IR_out;
	wire [31:0] w_Y_out;
	wire [63:0] w_Z_out;
	wire [31:0] w_mar_out;
	wire [31:0] w_HI_out;
	wire [31:0] w_LO_out;
	wire [31:0] w_MDR_out;
	
	wire [63:0] w_alu_out;
	
	wire [31:0] w_pc_adder_out;  // PC has a dedicated adder
	
	wire [8:0] w_bus_select_signals; // Make sure to asign your reg out signals
	
	assign w_bus_select_signals = {in_c_read, in_inport_read, in_mdr_read, in_pc_read, in_z_lo_read, in_z_hi_read, in_lo_read, in_hi_read, in_regfile_read};
	assign out_bus = r_bus;
	
	registerfile_16x32 regfile( .in_Cdata (r_bus),
		.in_Cselect (in_regfile_location),
		.in_Aselect (in_regfile_location),
		.out_Adata (w_regfile_out),
		.in_clr (in_reg_clear),  
		.in_write (in_regfile_write),
		.in_read (in_regfile_read),
		.in_clk (clk));
		
	register_32 PC (.D (w_pc_in),  // We are going to have to add our own adder for PC
		.Q (w_PC_out),
		.clk (clk),
		.clr (in_reg_clear),
		.write (in_pc_write));
	
	register_32 IR (.D (r_bus),
		.Q (w_IR_out),
		.clk (clk),
		.clr (in_reg_clear),
		.write (in_ir_write));
	
	register_32 Y (.D (r_bus),
		.Q (w_Y_out),
		.clk (clk),
		.clr (in_reg_clear),
		.write (in_y_write));
	
	register_64 Z (.D (w_alu_out),
		.Q (w_Z_out),
		.clk (clk),
		.clr (in_reg_clear),
		.write (in_z_write));
	
	register_32 MAR (.D (r_bus),
		.Q (w_mar_out),
		.clk (clk),
		.clr (in_reg_clear),
		.write (in_mar_write));
	
	register_32 HI (.D (r_bus),
		.Q (w_HI_out),
		.clk (clk),
		.clr (in_reg_clear),
		.write (in_hi_write));
	
	register_32 LO (.D (r_bus),
		.Q (w_LO_out),
		.clk (clk),
		.clr (in_reg_clear),
		.write (in_lo_write));
	
	memory_data_register_32 mdr(.in_bus (r_bus),
		.in_memory (in_mem_data),
		.in_read (in_mdr_select),
		.in_clr (in_reg_clear),
		.in_clk (clk),
		.in_write (in_mdr_write),
		.out (w_MDR_out));
	
	alu_32 alu(.in_a (w_Y_out),
		.in_b (r_bus),
		.in_opcode (in_alu_opcode),
		.out_result (w_alu_out));
	
	adder_32 pc_adder(.in_x (w_PC_out),
		.in_y (32'b1),
		.in_carry (1'b0),
		.out_sum (w_pc_adder_out),
		.out_carry ());
	
	always @(w_bus_select_signals) begin
		case (w_bus_select_signals) 
			9'b000000001: r_bus = w_regfile_out;  // Reg File Out
			9'b000000010: r_bus = w_HI_out;  // Reg HI
			9'b000000100: r_bus = w_LO_out;  // Reg LO
			9'b000001000: r_bus = w_Z_out[63:32];  // Reg Z HI
			9'b000010000: r_bus = w_Z_out[31:0];  // Reg Z LO
			9'b000100000: r_bus = w_PC_out;  // Reg PC
			9'b001000000: r_bus = w_MDR_out;  // Reg MDR
			9'b010000000: r_bus = 32'b0;  // Reg InPort
			9'b100000000: r_bus = 32'b0;  // C_sign extend
			default: r_bus = 32'hx;
		endcase
	end
	
	always @(*) begin
		if(in_inc_pc) begin
			w_pc_in <= w_pc_adder_out;
		end else begin
			w_pc_in <= r_bus;
		end
	end
	
endmodule

//------------------------------------------------

module datapath_tb_and;

	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				y_write <= 1;  // Write bus to Y
			end
			T4 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R4 onto bus
				regfile_location <= 4;  // Put R4 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b0110;
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5
				regfile_location <= 5;  // Write bus to R5
			end
		endcase
	end
endmodule // end of AND

module datapath_tb_or; //or R5, R2, R4 op code: b0111

	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				y_write <= 1;  // Write bus to Y
			end
			T4 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R4 onto bus
				regfile_location <= 4;  // Put R4 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b0111;
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5
				regfile_location <= 5;  // Write bus to R5
			end
		endcase
	end

endmodule //end of OR

module datapath_tb_add; //add R5, R2, R4

	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				y_write <= 1;  // Write bus to Y
			end
			T4 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R4 onto bus
				regfile_location <= 4;  // Put R4 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b0000;
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5
				regfile_location <= 5;  // Write bus to R5
			end
		endcase
	end

endmodule //end of ADD

module datapath_tb_sub; //sub R5, R2, R4

	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				y_write <= 1;  // Write bus to Y
			end
			T4 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R4 onto bus
				regfile_location <= 4;  // Put R4 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b0001;
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5
				regfile_location <= 5;  // Write bus to R5
			end
		endcase
	end

endmodule //end of SUB

module datapath_tb_mul; //mul R2, R4

	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
			T5 : state = T6;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				y_write <= 1;  // Write bus to Y
			end
			T4 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R4 onto bus
				regfile_location <= 4;  // Put R4 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b1000;
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5(Low bits)
				regfile_location <= 5;  // Write bus to R5
			end
			T6 : begin
				reset_read_write_signals();
				z_hi_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R6 (high bits)
				regfile_location <= 6;  // Write bus to R6
		endcase
	end

endmodule //end of mul

module datapath_tb_div; //div R2, R4

	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
			T5 : state = T6;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				y_write <= 1;  // Write bus to Y
			end
			T4 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R4 onto bus
				regfile_location <= 4;  // Put R4 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b1001;
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5(Low bits)
				regfile_location <= 5;  // Write bus to R5
			end
			T6 : begin
				reset_read_write_signals();
				z_hi_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R6 (high bits)
				regfile_location <= 6;  // Write bus to R6
		endcase
	end

endmodule

module datapath_tb_shr; //shr R5, R2, R4

	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				y_write <= 1;  // Write bus to Y
			end
			T4 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R4 onto bus
				regfile_location <= 4;  // Put R4 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b0100;
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5
				regfile_location <= 5;  // Write bus to R5
			end
		endcase
	end

end module

module datapath_tb_shl; //shl R5, R2, R4

	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				y_write <= 1;  // Write bus to Y
			end
			T4 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R4 onto bus
				regfile_location <= 4;  // Put R4 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b0101;
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5
				regfile_location <= 5;  // Write bus to R5
			end
		endcase
	end

endmodule

module datapath_tb_ror; //ror R5, R2, R4

	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				y_write <= 1;  // Write bus to Y
			end
			T4 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R4 onto bus
				regfile_location <= 4;  // Put R4 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b0010;
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5
				regfile_location <= 5;  // Write bus to R5
			end
		endcase
	end

endmodule

module datapath_tb_rol; //rol R5, R2, R4

	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				y_write <= 1;  // Write bus to Y
			end
			T4 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R4 onto bus
				regfile_location <= 4;  // Put R4 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b0011;
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5
				regfile_location <= 5;  // Write bus to R5
			end
		endcase
	end

endmodule

module datapath_tb_neg; //neg R5, R2

	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b1010;
			end
			T4 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5
				regfile_location <= 5;  // Write bus to R5
			end
		endcase
	end

endmodule 

module datapath_tb_not; //not R5, R2

	
	// Set up registers and DUR
	reg clk, reg_clear, mdr_select, inc_pc;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write;
	reg [3:0] regfile_location, alu_opcode;
	reg [31:0] mem_data;
	
	wire[31:0] bus;
	
	datapath DUT (.clk (clk),
		.in_regfile_location (regfile_location),
		.in_alu_opcode (alu_opcode),
		.in_mem_data (mem_data),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.out_bus (bus));
	
	// Set up States
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011;
	
	reg [3:0] state = Default;
	
	// Set up clock
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	always @(posedge clk) begin
		case(state)
			Default : state = Reg_load1a;
			Reg_load1a : state = Reg_load1b;
			Reg_load1b : state = Reg_load2a;
			Reg_load2a : state = Reg_load2b;
			Reg_load2b : state = Reg_load3a;
			Reg_load3a : state = Reg_load3b;
			Reg_load3b : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
		endcase
	end
	
	// Reset Variables Function
	task reset_read_write_signals();
		begin
		regfile_read = 0;
		hi_read = 0;
		lo_read = 0;
		z_hi_read = 0;
		z_lo_read = 0;
		pc_read = 0;
		mdr_read = 0;
		inport_read = 0;
		c_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				mem_data <= 0;
				alu_opcode <= 0;
				regfile_location <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				
				reset_read_write_signals();
			end
			Reg_load1a : begin
				reset_read_write_signals();
				mem_data <= 32'h22;
				mdr_write <= 1;
				mdr_select <= 1;
			end
			Reg_load1b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 2;
			end
			Reg_load2a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load2b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 4;
			end
			Reg_load3a : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mem_data <= 32'h24;
			end
			Reg_load3b : begin
				reset_read_write_signals();
				mdr_read <= 1;
				regfile_write <= 1;
				regfile_location <= 5;
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
			end
			T1 : begin
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
				mem_data <= 32'h4A920000;	// Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				regfile_read <= 1;  // Put R2 onto bus
				regfile_location <= 2;  // Put R2 onto bus
				z_write <= 1;  // Write ALU to Z
				alu_opcode <= 4'b1011;
			end
			T4 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Put Z[31:0] onto bus
				regfile_write <= 1;  // Write bus to R5
				regfile_location <= 5;  // Write bus to R5
			end
		endcase
	end
	
endmodule
