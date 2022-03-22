module control_unit (input clk,
	input in_reset,
	input in_stop,
	input [31:0] in_ir,
	input in_branch,
	output reg out_run, // If the machine is running
	output reg out_clear,  // Global register clear
	// Select Encode Signals
	output reg out_gra,
	output reg out_grb,
	output reg out_grc,
	output reg out_ba_read,
	// Write Signals
	output reg out_regfile_write,
	output reg out_hi_write,
	output reg out_lo_write,
	output reg out_z_write,
	output reg out_pc_write,
	output reg out_mdr_write,
	output reg out_ir_write,
	output reg out_y_write,
	output reg out_mar_write,
	output reg out_mem_write,
	output reg out_outport_write,
	output reg out_conff_write,
	// Read Signals
	output reg out_hi_read,
	output reg out_lo_read,
	output reg out_z_lo_read,
	output reg out_z_hi_read,
	output reg out_regfile_read,
	output reg out_pc_read,
	output reg out_mdr_read,
	output reg out_inport_read,
	output reg out_c_read,
	output reg out_mem_read,
	// ALU Signals
	output reg [3:0] out_alu_opcode,
	output reg out_div_reset,
	// Memory Signals
	output reg out_mdr_select,
	// PC Signals
	output reg out_inc_pc);
	
	parameter reset = 32'd1, fetch0 = 32'd2, fetch1 = 32'd3, fetch2 = 32'd4,
	load3 = 32'd5, load4 = 32'd6, load5 = 32'd7, load6 = 32'd8, load7 = 32'd9,
	loadi3 = 32'd10, loadi4 = 32'd11, loadi5 = 32'd12,
	store3 = 32'd13, store4 = 32'd14, store5 = 32'd15, store6 = 32'd16,
	add3 = 32'd17, add4 = 32'd18, add5 = 32'd19,
	sub3 = 32'd20, sub4 = 32'd21, sub5 = 32'd22,
	shr3 = 32'd23, shr4 = 32'd24, shr5 = 32'd25,
	shl3 = 32'd26, shl4 = 32'd27, shl5 = 32'd28,
	ror3 = 32'd29, ror4 = 32'd30, ror5 = 32'd31,
	rol3 = 32'd32, rol4 = 32'd33, rol5 = 32'd34,
	and3 = 32'd35, and4 = 32'd36, and5 = 32'd37,
	or3 = 32'd38, or4 = 32'd39, or5 = 32'd40,
	andi3 = 32'd41, andi4 = 32'd42, andi5 = 32'd43,
	ori3 = 32'd44, ori4 = 32'd45, ori5 = 32'd46,
	mul3 = 32'd47, mul4 = 32'd48, mul5 = 32'd49, mul6 = 32'd50,
	div3 = 32'd51, div4 = 32'd52, div5 = 32'd53, div6 = 32'd54,
	neg3 = 32'd55, neg4 = 32'd56,
	not3 = 32'd57, not4 = 32'd58,
	branch3 = 32'd59, branch4 = 32'd60, branch5 = 32'd61, branch6 = 32'd62,
	jr3 = 32'd63,
	jal3 = 32'd64, jal4 = 32'd65,
	in3 = 32'd66,
	out3 = 32'd67,
	mfhi3 = 32'd68,
	mflo3 = 32'd69,
	nop3 = 32'd70,
	halt3 = 32'd71,
	addi3 = 32'd72, addi4 = 32'd73, addi5 = 32'd74; 
	
	reg [31:0] state = reset;
	reg fetch_buffer = 0;
	reg [7:0] div_counter = 0;
	
	always @(posedge clk, posedge in_reset) begin
		if (in_reset) begin
			state = reset;
		end else if (in_stop) begin
			state = halt3;
		end else if (fetch_buffer) begin
			case(in_ir[31:27])
				5'b00000 : state = load3; // Load
				5'b00001 : state = loadi3; // Load Immediate
				5'b00010 : state = store3; // Store
				5'b00011 : state = add3; // Add
				5'b00100 : state = sub3; // Sub
				5'b00101 : state = shr3; // Shift Right
				5'b00110 : state = shl3; // Shift Left
				5'b00111 : state = ror3; // Rotate Right
				5'b01000 : state = rol3; // Rotate Left
				5'b01001 : state = and3; // And
				5'b01010 : state = or3; // Or
				5'b01011 : state = addi3; // Add Immediate
				5'b01100 : state = andi3; // And Immediate
				5'b01101 : state = ori3; // Or Immediate
				5'b01110 : state = mul3; // Multiply
				5'b01111 : state = div3; // Divide
				5'b10000 : state = neg3; // Negate
				5'b10001 : state = not3; // Not
				5'b10010 : state = branch3; // Branch
				5'b10011 : state = jr3; // Jump Return
				5'b10100 : state = jal3; // Jump Link
				5'b10101 : state = in3; // Input
				5'b10110 : state = out3; // Output
				5'b10111 : state = mfhi3; // Move from HI
				5'b11000 : state = mflo3; // Move from LO
				5'b11001 : state = nop3; // NOP
				5'b11010 : state = halt3; // Halt
			endcase
			
			fetch_buffer = 0;
		end else if (state == div4) begin
			if(div_counter == 0) begin
				out_div_reset = 1;
				div_counter = div_counter + 1;
			end else if (div_counter == 34) begin
				div_counter = 0;
				state = div5;
			end else begin
				out_div_reset = 0;
				div_counter = div_counter + 1;
			end
			
		end
		else begin
			case(state)
				reset : state = fetch0;
				fetch0 : state = fetch1;
				fetch1 : state = fetch2;
				fetch2 : fetch_buffer = 1;
				load3 : state = load4;
				load4 : state = load5;
				load5 : state = load6;
				load6 : state = load7;
				load7 : state = fetch0;
				
				loadi3 : state = loadi4;
				loadi4 : state = loadi5;
				loadi5 : state = fetch0;
				
				store3 : state = store4;
				store4 : state = store5;
				store5 : state = store6;
				store6 : state = fetch0;
				
				add3 : state = add4;
				add4 : state = add5;
				add5 : state = fetch0;
				
				sub3 : state = sub4;
				sub4 : state = sub5;
				sub5 : state = fetch0;
				
				shr3 : state = shr4;
				shr4 : state = shr5;
				shr5 : state = fetch0;
				
				shl3 : state = shl4;
				shl4 : state = shl5;
				shl5 : state = fetch0;
				
				ror3 : state = ror4;
				ror4 : state = ror5;
				ror5 : state = fetch0;
				
				rol3 : state = rol4;
				rol4 : state = rol5;
				rol5 : state = fetch0;
				
				and3 : state = and4;
				and4 : state = and5;
				and5 : state = fetch0;
				
				or3 : state = or4;
				or4 : state = or5;
				or5 : state=  fetch0;
				
				addi3 : state = addi4;
				addi4 : state = addi5;
				addi5 : state = fetch0;
				
				andi3 : state = andi4;
				andi4 : state = andi5;
				andi5 : state = fetch0;
				
				ori3 : state = ori4;
				ori4 : state = ori5;
				ori5 : state = fetch0;
				
				mul3 : state = mul4;
				mul4 : state = mul5;
				mul5 : state = mul6;
				mul6 : state = fetch0;
				
				div3 : state = div4;
				div5 : state = div6;
				div6 : state = fetch0;
				
				neg3 : state = neg4;
				neg4 : state = fetch0;
				
				not3 : state = not4;
				not4 : state = fetch0;
				
				branch3 : state = branch4;
				branch4 : state = branch5;
				branch5 : state = branch6;
				branch6 : state = fetch0;
				
				jr3 : state = fetch0;
				
				jal3 : state = jal4;
				jal4 : state = fetch0;
				
				in3 : state = fetch0;
				
				out3 : state = fetch0;
				
				mfhi3 : state = fetch0;
				
				mflo3 : state = fetch0;
				
				nop3 : state = fetch0;
				
				halt3 : state = halt3;
			endcase
		end
	end
	
	always @(state) begin
		case(state)
			reset : begin
				out_run <= 1;
				out_clear <= 0;
				out_gra <= 0;
				out_grb <= 0;
				out_grc <= 0;
				out_ba_read <= 0;
				out_regfile_write <= 0;
				out_hi_write <= 0;
				out_lo_write <= 0;
				out_z_write <= 0;
				out_pc_write <= 0;
				out_mdr_write <= 0;
				out_ir_write <= 0;
				out_y_write <= 0;
				out_mar_write <= 0;
				out_mem_write <= 0;
				out_outport_write <= 0;
				out_conff_write <= 0;
				out_hi_read <= 0;
				out_lo_read <= 0;
				out_z_lo_read <= 0;
				out_z_hi_read <= 0;
				out_regfile_read <= 0;
				out_pc_read <= 0;
				out_mdr_read <= 0;
				out_inport_read <= 0;
				out_c_read <= 0;
				out_mem_read <= 0;
				out_alu_opcode <= 4'b0000;
				out_mdr_select <= 0;
				out_inc_pc <= 0;
			end
			fetch0 : begin
				out_clear <= 1; // Make sure registers are not being cleared
				
				out_gra <= 0;
				out_grb <= 0;
				out_grc <= 0;
				out_ba_read <= 0;
				out_regfile_write <= 0;
				out_hi_write <= 0;
				out_lo_write <= 0;
				out_z_write <= 0;
				out_pc_write <= 0;
				out_mdr_write <= 0;
				out_ir_write <= 0;
				out_y_write <= 0;
				out_mem_write <= 0;
				out_outport_write <= 0;
				out_conff_write <= 0;
				out_hi_read <= 0;
				out_lo_read <= 0;
				out_z_lo_read <= 0;
				out_z_hi_read <= 0;
				out_regfile_read <= 0;
				out_mdr_read <= 0;
				out_inport_read <= 0;
				out_c_read <= 0;
				out_alu_opcode <= 4'b0000;
				out_mdr_select <= 0;
				
				out_pc_read <= 1;
				out_mar_write <= 1;
				out_inc_pc <= 1;
				out_pc_write <= 1;
				out_mem_read <= 1;
			end
			fetch1 : begin
				out_pc_read <= 0;
				out_mar_write <= 0;
				out_inc_pc <= 0;
				out_pc_write <= 0;
				out_mem_read <= 0;
				
				out_mdr_write <= 1;
				out_mdr_select <= 1;
			end
			fetch2 : begin
				out_mdr_write <= 0;
				out_mdr_select <= 0;
				
				out_mdr_read <= 1;
				out_ir_write <= 1;
			end
			load3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_grb <= 1;
				out_ba_read <= 1;
				out_y_write <= 1;
			end
			load4 : begin
				out_grb <= 0;
				out_ba_read <= 0;
				out_y_write <= 0;
				
				out_c_read <= 1;
				out_alu_opcode <= 4'b0000;
				out_z_write <= 1;
			end
			load5 : begin
				out_c_read <= 0;
				out_z_write <= 0;
				
				out_z_lo_read <= 1;
				out_mar_write <= 1;
				out_mem_read <= 1;
			end
			load6 : begin
				out_z_lo_read <= 0;
				out_mar_write <= 0;
				out_mem_read <= 0;
				
				out_mdr_write <= 1;
				out_mdr_select <= 1;
			end
			load7 : begin
				out_mdr_write <= 0;
				out_mdr_select <= 0;
				
				out_gra <= 1;
				out_regfile_write <= 1;
				out_mdr_read <= 1;
			end
			loadi3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_grb <= 1;
				out_ba_read <= 1;
				out_y_write <= 1;
			end
			loadi4 : begin
				out_grb <= 0;
				out_ba_read <= 0;
				out_y_write <= 0;
				
				out_c_read <= 1;
				out_alu_opcode <= 4'b0000;
				out_z_write <= 1;
			end
			loadi5 : begin
				out_c_read <= 0;
				out_z_write <= 0;
				
				out_z_lo_read <= 1;
				out_gra <= 1;
				out_regfile_write <= 1;
			end
			store3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_grb <= 1;
				out_ba_read <= 1;
				out_y_write <= 1;
			end
			store4 : begin
				out_grb <= 0;
				out_ba_read <= 0;
				out_y_write <= 0;
				
				out_c_read <= 1;
				out_alu_opcode <= 4'b0000;
				out_z_write <= 1;
			end
			store5 : begin
				out_c_read <= 0;
				out_z_write <= 0;
				
				out_z_lo_read <= 1;
				out_mar_write <= 1;
			end
			store6 : begin
				out_z_lo_read <= 0;
				out_mar_write <= 0;
				
				out_gra <= 1;
				out_regfile_read <= 1;
				out_mdr_write <= 1;
				out_mdr_select <= 0;
				out_mem_write <= 1;
			end
			add3, sub3, shr3, shl3, ror3, rol3, and3, or3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_grb <= 1;
				out_regfile_read <= 1;
				out_y_write <= 1;
			end
			add4, sub4, shr4, shl4, ror4, rol4, and4, or4 : begin
				out_grb <= 0;
				out_y_write <= 0;
				
				out_regfile_read <= 1;
				out_grc <= 1;
				out_z_write <= 1;
				
				case(state)
					add4 : out_alu_opcode <= 4'b0000;
					sub4 : out_alu_opcode <= 4'b0001;
					shr4 : out_alu_opcode <= 4'b0100;
					shl4 : out_alu_opcode <= 4'b0101;
					ror4 : out_alu_opcode <= 4'b0010;
					rol4 : out_alu_opcode <= 4'b0011;
					and4 : out_alu_opcode <= 4'b0110;
					or4 : out_alu_opcode <= 4'b0111;
				endcase
			end
			add5, sub5, shr5, shl5, ror5, rol5, and5, or5  : begin
				out_regfile_read <= 0;
				out_grc <= 0;
				out_z_write <= 0;
				
				out_z_lo_read <= 1;
				out_regfile_write <= 1;
				out_gra <= 1;
			end
			mul3, div3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_regfile_read <= 1;
				out_gra <= 1;
				out_y_write <= 1;
			end
			mul4, div4 : begin
				out_gra <= 0;
				out_y_write <= 0;
				
				out_grb <= 1;
				out_z_write <= 1;
				
				case(state)
					mul4 : out_alu_opcode <= 4'b1000;
					div4 : out_alu_opcode <= 4'b1001;
				endcase
			end
			mul5, div5 : begin
				out_regfile_read <= 0;
				out_grb <= 0;
				out_z_write <= 0;
				
				out_z_lo_read <= 1;
				out_lo_write <= 1;
			end
			mul6, div6 : begin
				out_z_lo_read <= 0;
				out_lo_write <= 0;
				
				out_z_hi_read <= 1;
				out_hi_write <= 1;
			end
			neg3, not3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_regfile_read <= 1;
				out_grb <= 1;
				out_z_write <= 1;
				
				case(state)
					neg3 : out_alu_opcode <= 4'b1010;
					not3 : out_alu_opcode <= 4'b1011;
				endcase
			end
			neg4, not4 : begin
				out_regfile_read <= 0;
				out_grb <= 0;
				out_z_write <= 0;
				
				out_z_lo_read <= 1;
				out_gra <= 1;
				out_regfile_write <= 1;
			end
			addi3, andi3, ori3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_grb <= 1;
				out_regfile_read <= 1;
				out_y_write <= 1;
			end
			addi4, andi4, ori4 : begin
				out_grb <= 0;
				out_regfile_read <= 0;
				out_y_write <= 0;
				
				out_c_read <= 1;
				out_z_write <= 1;
				
				case(state)
					addi4 : out_alu_opcode <= 4'b0000;
					andi4 : out_alu_opcode <= 4'b0110;
					ori4 : out_alu_opcode <= 4'b0111;
				endcase
			end
			addi5, andi5, ori5 : begin
				out_c_read <= 0;
				out_z_write <= 0;
				
				out_z_lo_read <= 1;
				out_gra <= 1;
				out_regfile_write <= 1;
			end
			branch3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_gra <= 1;
				out_regfile_read <= 1;
				out_conff_write <= 1;
			end
			branch4 : begin
				out_gra <= 0;
				out_regfile_read <= 0;
				out_conff_write <= 0;
				
				out_pc_read <= 1;
				out_y_write <= 1;
			end
			branch5 : begin
				out_pc_read <= 0;
				out_y_write <= 0;
				
				out_c_read <= 1;
				out_alu_opcode <= 4'b0000;
				out_z_write <= 1;
			end
			branch6 : begin
				out_c_read <= 0;
				out_z_write <= 0;
				
				out_z_lo_read <= 1;
				out_pc_write <= in_branch;
			end
			jr3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_gra <= 1;
				out_regfile_read <= 1;
				out_pc_write <= 1;
			end
			jal3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_pc_read <= 1;
				out_regfile_write <= 1;
				out_grb <= 1;
			end
			jal4 : begin
				out_pc_read <= 0;
				out_regfile_write <= 0;
				out_grb <= 0;
				
				out_gra <= 1;
				out_regfile_read <= 1;
				out_pc_write <= 1;
			end
			in3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_gra <= 1;
				out_inport_read <= 1;
				out_regfile_write <= 1;
			end
			out3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_gra <= 1;
				out_outport_write <= 1;
				out_regfile_read <= 1;
			end
			mfhi3, mflo3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
				
				out_gra <= 1;
				out_regfile_write <= 1;
				
				case(state)
					mflo3 : out_lo_read <= 1;
					mfhi3 : out_hi_read <= 1;
				endcase
			end
			nop3 : begin
				out_mdr_read <= 0;
				out_ir_write <= 0;
			end
			halt3 : begin
				out_run <= 0;
			end
		endcase
	end
endmodule
