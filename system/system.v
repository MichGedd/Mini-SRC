module system(input clk,
	input [3:0] in_alu_opcode, // ALU opcode
	input in_reg_clear,  // Clear all registers
	input in_mdr_select, // This selects whether to read from bus or memory. 0 for bus, 1 for memory
	input in_inc_pc,
	input in_gra,
	input in_grb,
	input in_grc,
	input in_ba_read,
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
	input in_mem_read,
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
	input in_mem_write,
	input in_outport_write,
	input in_con_write,
	input [31:0] in_inport_data,
	output [31:0] out_bus,
	output [31:0] out_outport,
	output out_branch);

	wire [31:0] w_mem_address;
	wire [31:0] w_mdr_data;
	wire [31:0] w_mem_data;
	wire [31:0] w_ir_out;
	wire [31:0] w_bus_out;
	wire [3:0] w_regfile_location;
	
	wire w_regfile_read;
	wire w_regfile_write;
	
	assign out_bus = w_bus_out;

	datapath path (.clk (clk),
		.in_regfile_location (w_regfile_location),
		.in_alu_opcode (in_alu_opcode),
		.in_mem_data (w_mem_data),
		.in_reg_clear (in_reg_clear),
		.in_inc_pc (in_inc_pc),
		.in_mdr_select (in_mdr_select),
		.in_regfile_read (w_regfile_read),
		.in_hi_read (in_hi_read),
		.in_lo_read (in_lo_read),
		.in_z_hi_read (in_z_hi_read),
		.in_z_lo_read (in_z_lo_read),
		.in_pc_read (in_pc_read),
		.in_mdr_read (in_mdr_read),
		.in_inport_read (in_inport_read),
		.in_c_read (in_c_read),
		.in_regfile_write (w_regfile_write),
		.in_hi_write (in_hi_write),
		.in_lo_write (in_lo_write),
		.in_z_write (in_z_write),
		.in_pc_write (in_pc_write),
		.in_mdr_write (in_mdr_write),
		.in_ir_write (in_ir_write),
		.in_y_write (in_y_write),
		.in_mar_write (in_mar_write),
		.in_BAout (in_ba_read),
		.in_outport_write (in_outport_write),
		.in_inport_data (in_inport_data),
		.out_bus (w_bus_out),  // Maybe change this later
		.out_mdr (w_mdr_data),
		.out_mar (w_mem_address),
		.out_ir (w_ir_out),
		.out_outport(out_outport));
	
	memory RAM (.clock (clk),
		.address (w_mem_address[8:0]),
		.data (w_mdr_data),
		.rden (in_mem_read),
		.wren (in_mem_write),
		.q (w_mem_data));
	
	select_encode_logic sel_enc_logic(.in_gra (in_gra),
		.in_grb (in_grb),
		.in_grc (in_grc),
		.in_read (in_regfile_read),
		.in_write (in_regfile_write),
		.in_base_addr_read (in_ba_read),
		.in_ir (w_ir_out),
		.out_regfile_location (w_regfile_location),
		.out_regfile_read (w_regfile_read),
		.out_regfile_write (w_regfile_write));
	
	con_ff_logic CON_FF(.clk (clk),
		.in_condition (w_ir_out[20:19]),
		.in_bus (w_bus_out),
		.in_con_write (in_con_write),  // Part of Phase 3
		.out_branch (out_branch));  // Part of Phase 3

endmodule

//---------------------------
// Note: When compiling these in VSIM, you may need to run vsim <testbench file> -L altera_mf_ver

module system_tb_ld_case1;
	// Set M[0] to 00000000100000000000000001010101
	// Set M{85] to some 32 bit value (1010101)
	// Output -> R1 should be M[85]
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
			T5 : state = T6;
			T6 : state = T7;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				grb <= 1;  // Set the regfile location RB
				ba_read <= 1;  // Set write signal to true
				y_write <= 1;
			end
			T4 : begin
				reset_read_write_signals();
				c_read <= 1;  // Put C on bus
				alu_opcode <= 4'b0000;  // ADD 
				z_write <= 1;  // Put ALU output in Z
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Read bottom 32 of Z to bus
				mar_write <= 1;  // Set MAR to bus
				mem_read <= 1;  // Prepare to read from memory
			end
			T6 : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mdr_select <= 1;
			end
			T7 : begin
				reset_read_write_signals();
				gra <= 1;  // Write bus to Ra
				regfile_write <= 1;  // Write bus to Ra
				mdr_read <= 1;  // Put MDR onto bus
			end
		endcase
	end
endmodule

module system_tb_ld_case2;
	// Set M[0] to 00000000000010000000000000100011
	// Set M{85] to some 32 bit value
	// Output -> R0 should be M[85]
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[1].register_inst.value_32 = 32'd50;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
			T3 : state = T4;
			T4 : state = T5;
			T5 : state = T6;
			T6 : state = T7;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				grb <= 1;  // Set the regfile location RB
				ba_read <= 1;  // Set write signal to true
				y_write <= 1;
			end
			T4 : begin
				reset_read_write_signals();
				c_read <= 1;  // Put C on bus
				alu_opcode <= 4'b0000;  // ADD 
				z_write <= 1;  // Put ALU output in Z
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Read bottom 32 of Z to bus
				mar_write <= 1;  // Set MAR to bus
				mem_read <= 1;  // Prepare to read from memory
			end
			T6 : begin
				reset_read_write_signals();
				mdr_write <= 1;
				mdr_select <= 1;
			end
			T7 : begin
				reset_read_write_signals();
				gra <= 1;  // Write bus to Ra
				regfile_write <= 1;  // Write bus to Ra
				mdr_read <= 1;  // Put MDR onto bus
			end
		endcase
	end
endmodule

module system_tb_ld_case3;
	// Set M[0] to 00001000100000000000000001010101
	// Output -> R1 should be d85
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				grb <= 1;  // Set the regfile location RB
				ba_read <= 1;  // Set write signal to true
				y_write <= 1;
			end
			T4 : begin
				reset_read_write_signals();
				c_read <= 1;  // Put C on bus
				alu_opcode <= 4'b0000;  // ADD 
				z_write <= 1;  // Put ALU output in Z
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Read bottom 32 of Z to bus
				gra <= 1;  // Write bus to Ra
				regfile_write <= 1;  // Write bus to Ra
			end
		endcase
	end
endmodule

module system_tb_ld_case4;
	// Set M[0] to 00001000000010000000000000100011
	// Output -> R0 should be d45
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[1].register_inst.value_32 = 32'd10;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				grb <= 1;  // Set the regfile location RB
				ba_read <= 1;  // Set write signal to true
				y_write <= 1;
			end
			T4 : begin
				reset_read_write_signals();
				c_read <= 1;  // Put C on bus
				alu_opcode <= 4'b0000;  // ADD 
				z_write <= 1;  // Put ALU output in Z
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Read bottom 32 of Z to bus
				gra <= 1;  // Write bus to Ra
				regfile_write <= 1;  // Write bus to Ra
			end
		endcase
	end
endmodule

module system_tb_st_case1;
	// Set M[0] to 00010000100000000000000001011010
	// Output -> M[90] should be 85
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[1].register_inst.value_32 = 32'd85;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				grb <= 1;  // Set the regfile location RB
				ba_read <= 1;  // Set write signal to true
				y_write <= 1;
			end
			T4 : begin
				reset_read_write_signals();
				c_read <= 1;  // Put C on bus
				alu_opcode <= 4'b0000;  // ADD 
				z_write <= 1;  // Put ALU output in Z
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Read bottom 32 of Z to bus
				mar_write <= 1; // Write bus to MAR
			end
			T6 : begin
				reset_read_write_signals();
				gra <= 1;  // Write Ra to bus
				regfile_read <= 1;  // Write Ra to bus
				mdr_write <= 1;  // Write bus to MDR
				mdr_select <= 0;  // Write bus to MDR
				mem_write <= 1;  // Write to memory
			end
		endcase
	end
endmodule

module system_tb_st_case2;
	// Set M[0] to 00010000100010000000000001011010
	// Output -> M[175] should be 85
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[1].register_inst.value_32 = 32'd85;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				grb <= 1;  // Set the regfile location RB
				ba_read <= 1;  // Set write signal to true
				y_write <= 1;
			end
			T4 : begin
				reset_read_write_signals();
				c_read <= 1;  // Put C on bus
				alu_opcode <= 4'b0000;  // ADD 
				z_write <= 1;  // Put ALU output in Z
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Read bottom 32 of Z to bus
				mar_write <= 1; // Write bus to MAR
			end
			T6 : begin
				reset_read_write_signals();
				gra <= 1;  // Write Ra to bus
				regfile_read <= 1;  // Write Ra to bus
				mdr_write <= 1;  // Write bus to MDR
				mdr_select <= 0;  // Write bus to MDR
				mem_write <= 1;  // Write to memory
			end
		endcase
	end
endmodule

module system_tb_addi;
	// Set M[0] to 01011001000011111111111111111011
	// R1 is set to 85
	// Output -> R2 should be 80 (R1 - 5)
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[1].register_inst.value_32 = 32'd85;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				grb <= 1;  // Set the regfile location RB
				regfile_read <= 1;  // Read regfile to bus
				y_write <= 1;  // Write Y to bus
			end
			T4 : begin
				reset_read_write_signals();
				c_read <= 1;  // Put C on bus
				alu_opcode <= 4'b0000;  // ADD 
				z_write <= 1;  // Put ALU output in Z
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Read bottom 32 of Z to bus
				gra <= 1;  // Write bus to Ra
				regfile_write <= 1;  // Write bus to Ra
			end
		endcase
	end
endmodule

module system_tb_andi;
	// Set M[0] to 01100001000010000000000000011010
	// R1 is set to 85
	// Output -> R2 should be 1 (85 logical and 26 is 1)
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[1].register_inst.value_32 = 32'd85;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				grb <= 1;  // Set the regfile location RB
				regfile_read <= 1;  // Read regfile to bus
				y_write <= 1;  // Write Y to bus
			end
			T4 : begin
				reset_read_write_signals();
				c_read <= 1;  // Put C on bus
				alu_opcode <= 4'b0110;  // ADD 
				z_write <= 1;  // Put ALU output in Z
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Read bottom 32 of Z to bus
				gra <= 1;  // Write bus to Ra
				regfile_write <= 1;  // Write bus to Ra
			end
		endcase
	end
endmodule

module system_tb_ori;
	// Set M[0] to 01101001000010000000000000011010
	// R1 is set to 85
	// Output -> R2 should be 1 (85 logical or 26 is 1)
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[1].register_inst.value_32 = 32'd85;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				grb <= 1;  // Set the regfile location RB
				regfile_read <= 1;  // Read regfile to bus
				y_write <= 1;  // Write Y to bus
			end
			T4 : begin
				reset_read_write_signals();
				c_read <= 1;  // Put C on bus
				alu_opcode <= 4'b0111;  // ADD 
				z_write <= 1;  // Put ALU output in Z
			end
			T5 : begin
				reset_read_write_signals();
				z_lo_read <= 1;  // Read bottom 32 of Z to bus
				gra <= 1;  // Write bus to Ra
				regfile_write <= 1;  // Write bus to Ra
			end
		endcase
	end
endmodule

module system_tb_brzr;
	// Set M[0] to 10010001000000000000000000100011
	// Set R2 to a value
	// Output -> PC = PC + 1 + 35 = 36 (if R2 is set to 0)
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[2].register_inst.value_32 = 32'd0;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				gra <= 1;
				regfile_read <= 1;
				con_write <= 1;
			end
			T4 : begin
				reset_read_write_signals();
				pc_read <= 1;
				y_write <= 1;
			end
			T5 : begin
				reset_read_write_signals();
				c_read <= 1;
				alu_opcode <= 4'd0000;
				z_write <= 1;
			end
			T6 : begin
				reset_read_write_signals();
				z_lo_read <= 1;
				pc_write <= branch;
			end
		endcase
	end
endmodule

module system_tb_brnz;
	// Set M[0] to 10010001000010000000000000100011
	// Set R2 to a value
	// Output -> PC = PC + 1 + 35 = 36 (if R2 is not zero)
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[2].register_inst.value_32 = 32'd1;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				gra <= 1;
				regfile_read <= 1;
				con_write <= 1;
			end
			T4 : begin
				reset_read_write_signals();
				pc_read <= 1;
				y_write <= 1;
			end
			T5 : begin
				reset_read_write_signals();
				c_read <= 1;
				alu_opcode <= 4'd0000;
				z_write <= 1;
			end
			T6 : begin
				reset_read_write_signals();
				z_lo_read <= 1;
				pc_write <= branch;
			end
		endcase
	end
endmodule

module system_tb_brpl;
	// Set M[0] to 10010001000100000000000000100011
	// Set R2 to a value
	// Output -> PC = PC + 1 + 35 = 36 (if R2 is >= 0)
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[2].register_inst.value_32 = 32'h1;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				gra <= 1;
				regfile_read <= 1;
				con_write <= 1;
			end
			T4 : begin
				reset_read_write_signals();
				pc_read <= 1;
				y_write <= 1;
			end
			T5 : begin
				reset_read_write_signals();
				c_read <= 1;
				alu_opcode <= 4'd0000;
				z_write <= 1;
			end
			T6 : begin
				reset_read_write_signals();
				z_lo_read <= 1;
				pc_write <= branch;
			end
		endcase
	end
endmodule

module system_tb_brmi;
	// Set M[0] to 10010001000110000000000000100011
	// Set R2 to a value
	// Output -> PC = PC + 1 + 35 = 36 (if R2 is < 0)
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[2].register_inst.value_32 = 32'hFFFFFFFF;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				gra <= 1;
				regfile_read <= 1;
				con_write <= 1;
			end
			T4 : begin
				reset_read_write_signals();
				pc_read <= 1;
				y_write <= 1;
			end
			T5 : begin
				reset_read_write_signals();
				c_read <= 1;
				alu_opcode <= 4'd0000;
				z_write <= 1;
			end
			T6 : begin
				reset_read_write_signals();
				z_lo_read <= 1;
				pc_write <= branch;
			end
		endcase
	end
endmodule

module system_tb_jr;
	// Set M[0] to 10011000100000000000000000000000
	// Set R1 to a value
	// Output -> PC = R1 (50 in this case)
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[1].register_inst.value_32 = 32'd50;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				gra <= 1;
				regfile_read <= 1;
				pc_write <= 1;
			end
		endcase
	end
endmodule

module system_tb_jal;
	// Set M[0] to 10100000100000000000000000000000
	// Set R1 to a value
	// Output -> R15 = 1 (PC + 1), PC = R1 (50 in this case)
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[1].register_inst.value_32 = 32'd50;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Write PC to bus
				regfile_write <= 1;  // Write bus to R15
				grb <= 1;  // Write to R15
			end
			T4 : begin
				reset_read_write_signals();
				gra <= 1;  // Write RA to bus
				regfile_read <= 1;  // Write RA to bus
				pc_write <= 1;  // Write Bus to PC
			end
		endcase
	end
endmodule

module system_tb_mfhi;
	// Set M[0] to 10111001000000000000000000000000
	// Set HI to a value
	// Output -> R2 = HI
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.HI.value_32 = 32'd50;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				gra <= 1;
				hi_read <= 1;
				regfile_write <= 1;
			end
		endcase
	end
endmodule

module system_tb_mflo;
	// Set M[0] to 11000001000000000000000000000000
	// Set LO to a value
	// Output -> R2 = LO
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.LO.value_32 = 32'd50;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				gra <= 1;
				lo_read <= 1;
				regfile_write <= 1;
			end
		endcase
	end
endmodule

module system_tb_out;
	// Set M[0] to 10110000100000000000000000000000
	// Set R1 to a value
	// Output -> Out.Port = R1 (50 in this case)
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		DUT.path.regfile.gen_registers[1].register_inst.value_32 = 32'd50;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		outport_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
			end
			T3 : begin
				reset_read_write_signals();
				gra <= 1;
				outport_write <= 1;
				regfile_read <= 1;
			end
		endcase
	end
endmodule

module system_tb_in;
	// Set M[0] to 10101000100000000000000000000000
	// Output -> R1 = In.Port (50 in this case)
	
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read, con_write;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	reg [31:0] inport_data;
	reg outport_write;
	
	wire[31:0] outport;
	wire[31:0] bus;
	wire branch;
	
	system DUT (.clk (clk),
		.in_alu_opcode (alu_opcode),
		.in_reg_clear (reg_clear),
		.in_inc_pc (inc_pc),
		.in_mdr_select (mdr_select),
		.in_regfile_read (regfile_read),
		.in_gra (gra),
		.in_grb (grb),
		.in_grc (grc),
		.in_ba_read (ba_read),
		.in_con_write (con_write),
		.in_hi_read (hi_read),
		.in_lo_read (lo_read),
		.in_z_hi_read (z_hi_read),
		.in_z_lo_read (z_lo_read),
		.in_pc_read (pc_read),
		.in_mdr_read (mdr_read),
		.in_inport_read (inport_read),
		.in_c_read (c_read),
		.in_mem_read (mem_read),
		.in_regfile_write (regfile_write),
		.in_hi_write (hi_write),
		.in_lo_write (lo_write),
		.in_z_write (z_write),
		.in_pc_write (pc_write),
		.in_mdr_write (mdr_write),
		.in_ir_write (ir_write),
		.in_y_write (y_write),
		.in_mar_write (mar_write),
		.in_mem_write (mem_write),
		.out_bus (bus),
		.in_outport_write (outport_write),
		.in_inport_data (inport_data),
		.out_outport(outport),
		.out_branch(branch));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101, T7 = 4'b1110;
	
	reg[3:0] state = Default;
	
	always @(posedge clk) begin
		case(state)
			Default : state = T0;
			T0 : state = T1;
			T1 : state = T2;
			T2 : state = T3;
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
		mem_read = 0;
		regfile_write = 0;
		hi_write = 0;
		lo_write = 0;
		pc_write = 0;
		mdr_write = 0;
		ir_write = 0;
		y_write = 0;
		z_write = 0;
		mar_write = 0;
		mem_write = 0;
		outport_write = 0;
		
		gra = 0;
		grb = 0;
		grc = 0;
		ba_read = 0;
		
		con_write = 0;
		end
	endtask
	
	always @(state) begin
		case(state)
			Default : begin
				reg_clear <= 1;
				alu_opcode <= 0;
				mdr_select <= 0;
				inc_pc <= 0;
				reset_read_write_signals();
			end
			T0 : begin
				reset_read_write_signals();
				pc_read <= 1;  // Put PC on the bus
				mar_write <= 1;  // Write bus to MAR
				inc_pc <= 1;  // Increment PC
				pc_write <= 1;  // Increment PC
				mem_read <= 1;  // Read from memory
			end
			T1 : begin 
				reset_read_write_signals();
				inc_pc <= 0;  // Stop incrementing PC
				mdr_write <= 1;  // Write memory to MDR
				mdr_select <= 1;  // Write memory to MDR
			end
			T2 : begin
				reset_read_write_signals();
				mdr_read <= 1;  // Put MDR onto the bus
				ir_write <= 1;  // Write bus to IR
				inport_data = 32'd50; // Inport data has to arrive one clock cycle before reading from the inport
			end
			T3 : begin
				reset_read_write_signals();
				gra <= 1;
				inport_read <= 1;
				regfile_write <= 1;
			end
		endcase
	end
endmodule
