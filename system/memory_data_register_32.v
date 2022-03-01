module memory_data_register_32 (input [31:0] in_bus, input [31:0] in_memory, input in_read, input in_clr, input in_clk, input in_write, output [31:0] out);

	reg [31:0] r_mem_data_mux;

	/*register_32 mdr_reg(.D (r_mem_data_mux),
		.Q (out),
		.clr (in_clr),
		.clk (in_clk),
		.write (in_write));*/
	
	assign out = r_mem_data_mux;

	always@(*) begin
		case(in_read)
			1'b0: r_mem_data_mux = in_bus;
			1'b1: r_mem_data_mux = in_memory;
			default: r_mem_data_mux = 32'hx;
		endcase
	end

endmodule
