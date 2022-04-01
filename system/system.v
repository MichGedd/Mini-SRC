module system(input clk,
	input reset,
	input stop,
	input [7:0] inport_data,
	output [13:0] outport_data,
	output run);

	wire [31:0] w_mem_address;
	wire [31:0] w_mdr_data;
	wire [31:0] w_mem_data;
	wire [31:0] w_ir_out;
	wire [31:0] w_bus_out;
	wire [3:0] w_regfile_location;
	wire [31:0] w_outport_data;
	
	// Select Encode Signals
	wire w_gra;
	wire w_grb;
	wire w_grc;
	wire w_ba_read;
	wire w_selenc_regfile_read;
	wire w_selenc_regfile_write;
	
	// Write Signals
	wire w_regfile_write;
	wire w_hi_write;
	wire w_lo_write;
	wire w_z_write;
	wire w_pc_write;
	wire w_mdr_write;
	wire w_ir_write;
	wire w_y_write;
	wire w_mar_write;
	wire w_mem_write;
	wire w_outport_write;
	wire w_conff_write;
	
	// Read Signals
	wire w_regfile_read;
	wire w_hi_read;
	wire w_lo_read;
	wire w_z_lo_read;
	wire w_z_hi_read;
	wire w_pc_read;
	wire w_mdr_read;
	wire w_inport_read;
	wire w_c_read;
	wire w_mem_read;
	
	wire [3:0] w_alu_opcode;
	wire w_div_reset;
	wire w_mdr_select;
	wire w_inc_pc;
	wire w_clr;
	wire w_branch;
	
	// DE0-CV
	wire w_reset = ~reset;
	wire w_stop = ~stop;
	wire w_clk;
	wire [31:0] w_out_R;
	wire [31:0] w_inport = {{24{1'b0}}, {inport_data[7:0]}};
		
	datapath path (.clk (w_clk),
		.in_regfile_location (w_regfile_location),
		.in_alu_opcode (w_alu_opcode),
		.in_mem_data (w_mem_data),
		.in_reg_clear (w_clr),
		.in_inc_pc (w_inc_pc),
		.in_mdr_select (w_mdr_select),
		.in_div_reset (w_div_reset),
		.in_regfile_read (w_selenc_regfile_read),
		.in_hi_read (w_hi_read),
		.in_lo_read (w_lo_read),
		.in_z_hi_read (w_z_hi_read),
		.in_z_lo_read (w_z_lo_read),
		.in_pc_read (w_pc_read),
		.in_mdr_read (w_mdr_read),
		.in_inport_read (w_inport_read),
		.in_c_read (w_c_read),
		.in_regfile_write (w_selenc_regfile_write),
		.in_hi_write (w_hi_write),
		.in_lo_write (w_lo_write),
		.in_z_write (w_z_write),
		.in_pc_write (w_pc_write),
		.in_mdr_write (w_mdr_write),
		.in_ir_write (w_ir_write),
		.in_y_write (w_y_write),
		.in_mar_write (w_mar_write),
		.in_BAout (w_ba_read),
		.in_outport_write (w_outport_write),
		.in_inport_data (w_inport),
		.out_bus (w_bus_out),
		.out_mdr (w_mdr_data),
		.out_mar (w_mem_address),
		.out_ir (w_ir_out),
		.out_outport(w_outport_data),
		.out_R (w_out_R));  // DE0-CV Testing
	
	memory RAM (.clock (w_clk),
		.address (w_mem_address[8:0]),
		.data (w_mdr_data),
		.rden (w_mem_read),
		.wren (w_mem_write),
		.q (w_mem_data));
	
	select_encode_logic sel_enc_logic(.in_gra (w_gra),
		.in_grb (w_grb),
		.in_grc (w_grc),
		.in_read (w_regfile_read),
		.in_write (w_regfile_write),
		.in_base_addr_read (w_ba_read),
		.in_ir (w_ir_out),
		.out_regfile_location (w_regfile_location),
		.out_regfile_read (w_selenc_regfile_read),
		.out_regfile_write (w_selenc_regfile_write));
	
	con_ff_logic CON_FF(.clk (w_clk),
		.in_condition (w_ir_out[20:19]),
		.in_bus (w_bus_out),
		.in_con_write (w_conff_write),
		.out_branch (w_branch));
		
	control_unit control (.clk (w_clk),
		.in_reset (w_reset),
		.in_stop (w_stop),
		.in_ir (w_ir_out),
		.in_branch (w_branch),
		.out_run (run),
		.out_clear (w_clr),
		.out_gra (w_gra),
		.out_grb (w_grb),
		.out_grc (w_grc),
		.out_ba_read (w_ba_read),
		.out_regfile_write (w_regfile_write),
		.out_hi_write (w_hi_write),
		.out_lo_write (w_lo_write),
		.out_z_write (w_z_write),
		.out_pc_write (w_pc_write),
		.out_mdr_write (w_mdr_write),
		.out_ir_write (w_ir_write),
		.out_y_write (w_y_write),
		.out_mar_write (w_mar_write),
		.out_mem_write (w_mem_write),
		.out_outport_write (w_outport_write),
		.out_conff_write (w_conff_write),
		.out_hi_read (w_hi_read),
		.out_lo_read (w_lo_read),
		.out_z_lo_read (w_z_lo_read),
		.out_z_hi_read (w_z_hi_read),
		.out_regfile_read (w_regfile_read),
		.out_pc_read (w_pc_read),
		.out_mdr_read (w_mdr_read),
		.out_inport_read (w_inport_read),
		.out_c_read (w_c_read),
		.out_mem_read (w_mem_read),
		.out_alu_opcode (w_alu_opcode),
		.out_div_reset (w_div_reset),
		.out_mdr_select (w_mdr_select),
		.out_inc_pc (w_inc_pc),
		.out_state (state));
	
	seven_segment_display ssd_0 (.clk (w_clk),
		.data (w_outport_data[3:0]),
		.out (outport_data[6:0]));
		
	seven_segment_display ssd_1 (.clk (w_clk),
		.data (w_outport_data[7:4]),
		.out (outport_data[13:7]));
		
	clock_divider clk_div(.in_clock (clk),
		.out_clock (w_clk));

endmodule

//---------------------------
// Note: When compiling these in VSIM, you may need to run vsim <testbench file> -L altera_mf_ver

module system_tb;
	reg clk, reset, stop;
	reg [31:0] inport_data;
	
	wire run;
	wire [31:0] outport_data;
	
	system DUT (.clk (clk),
		.reset (reset),
		.stop (stop),
		.inport_data (inport_data),
		.run (run),
		.outport_data (outport_data));
	
	initial begin
		clk = 0;
		reset = 0;
		stop = 1;
		inport_data = 32'h88;
		forever begin 
			#10 clk = ~clk;
		end
	end
	
	always @(posedge clk) begin
		reset = 1;
	end
endmodule
