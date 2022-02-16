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
	output [31:0] out_bus);  // Maybe remove this later

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
		.out_bus (w_bus_out),  // Maybe change this later
		.out_mdr (w_mdr_data),
		.out_mar (w_mem_address),
		.out_ir (w_ir_out));
	
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
	
	con_ff_logic CON_FF(.in_condition (w_ir_out[20:19]),
		.in_bus (w_bus_out),
		.in_con_in (),  // Part of Phase 3
		.out_branch ());  // Part of Phase 3

endmodule

//---------------------------
// Note: When compiling these in VSIM, you may need to run vsim <testbench file> -L altera_mf_ver

module system_tb;
	reg clk, reg_clear, mdr_select, inc_pc, gra, grb, grc, ba_read;
	reg regfile_read, hi_read, lo_read, z_hi_read, z_lo_read, pc_read, mdr_read, inport_read, c_read, mem_read;
	reg regfile_write, hi_write, lo_write, z_write, pc_write, mdr_write, ir_write, y_write, mar_write, mem_write;
	reg [3:0] alu_opcode;
	
	wire[31:0] bus;
	
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
		.out_bus (bus));
	
	// Set up clock
	initial begin
		//Do this to instantiate the value in a register -> DUT.path.Y.value_32 = 32'b1;
		//Do this to acccess a specific register in the regfile (This is reg 0) -> DUT.path.regfile.gen_registers[0].register_inst.value_32 = 32'b1;
		clk = 0;
		forever #10 clk = ~clk;
	end
	
	parameter Default = 4'b0000, Reg_load1a = 4'b0001, Reg_load1b = 4'b0010, Reg_load2a = 4'b0011,
		Reg_load2b = 4'b0100, Reg_load3a = 4'b0101, Reg_load3b = 4'b0110, T0 = 4'b0111,
		T1 = 4'b1000, T2 = 4'b1001, T3 = 4'b1010, T4 = 4'b1011, T5 = 4'b1100, T6 = 4'b1101;
	
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
				gra <= 1;  // Set regfile location RA
				z_lo_read <= 1;  // Read bottom 32 of Z to bus
				regfile_write <= 1;  // Write bus to RA
			end
		endcase
	end

endmodule
