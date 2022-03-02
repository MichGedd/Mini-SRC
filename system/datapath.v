module datapath(input clk,
	input [3:0] in_regfile_location,  // Location to read from
	input [3:0] in_alu_opcode, // ALU opcode
	input [31:0] in_mem_data,  // Emulated data from memory
	input in_reg_clear,  // Clear all registers
	input in_mdr_select, // This selects whether to read from bus or memory. 0 for bus, 1 for memory
	input in_inc_pc,
	input in_BAout,
	input [31:0] in_inport_data,
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
	input in_outport_write,
	output [31:0] out_bus,
	output [31:0] out_mdr,
	output [31:0] out_mar,
	output [31:0] out_ir,
	output [31:0] out_outport);

	reg [31:0] r_bus;  // This is the bus
	reg [31:0] w_pc_in;  // Input to PC
	reg [31:0] r_mar_out;
	
	wire [31:0] w_regfile_out;
	wire [31:0] w_PC_out;
	wire [31:0] w_IR_out;
	wire [31:0] w_Y_out;
	wire [63:0] w_Z_out;
	wire [31:0] w_HI_out;
	wire [31:0] w_LO_out;
	wire [31:0] w_MDR_out;
	wire [31:0] w_c_sign_extend;
	
	wire [31:0] w_inport_out;
	
	wire [63:0] w_alu_out;
	
	wire [31:0] w_pc_adder_out;  // PC has a dedicated adder
	
	wire [8:0] w_bus_select_signals; // Make sure to asign your reg out signals
	
	assign w_bus_select_signals = {in_c_read, in_inport_read, in_mdr_read, in_pc_read, in_z_lo_read, in_z_hi_read, in_lo_read, in_hi_read, in_regfile_read};
	assign w_c_sign_extend = {{14{w_IR_out[18]}}, w_IR_out[17:0]};
	assign out_bus = r_bus;
	assign out_mar = r_mar_out;
	assign out_mdr = w_MDR_out;
	assign out_ir = w_IR_out;
	
	registerfile_16x32 regfile( .in_Cdata (r_bus),
		.in_Cselect (in_regfile_location),
		.in_Aselect (in_regfile_location),
		.out_Adata (w_regfile_out),
		.in_clr (in_reg_clear),  
		.in_write (in_regfile_write),
		.in_read (in_regfile_read),
		.in_clk (clk),
		.in_BAout (in_BAout)
	);
	
	register_32 OutPort (
		.D (r_bus),
		.Q (out_outport),
		.clk (clk),
		.clr (in_reg_clear),
		.write (in_outport_write));
	
	register_32 InPort (
		.D (in_inport_data),
		.Q (w_inport_out),
		.clk (clk),
		.clr (in_reg_clear),
		.write (1'b1));
		
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
	
	always @(*) begin
		case (w_bus_select_signals) 
			9'b000000001: r_bus = w_regfile_out;  // Reg File Out
			9'b000000010: r_bus = w_HI_out;  // Reg HI
			9'b000000100: r_bus = w_LO_out;  // Reg LO
			9'b000001000: r_bus = w_Z_out[63:32];  // Reg Z HI
			9'b000010000: r_bus = w_Z_out[31:0];  // Reg Z LO
			9'b000100000: r_bus = w_PC_out;  // Reg PC
			9'b001000000: r_bus = w_MDR_out;  // Reg MDR
			9'b010000000: r_bus = w_inport_out;  // Reg InPort
			9'b100000000: r_bus = w_c_sign_extend;  // C_sign extend
			default: r_bus = 32'hx;
		endcase
	end
	
	always @(*) begin
		if(in_inc_pc) begin
			w_pc_in <= w_pc_adder_out;
		end else begin
			w_pc_in <= r_bus;
		end
		
		if(in_mar_write) begin
			r_mar_out = r_bus;
		end
	end
	
endmodule
