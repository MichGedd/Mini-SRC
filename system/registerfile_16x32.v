//16 32 bit registers 

module registerfile_16x32 (
	input [31:0] in_Cdata, //the input data
	input [3:0] in_Cselect, //the input select
	//there would be a B here if we were using the 3 bus
	input [3:0] in_Aselect, //output select
	output[31:0] out_Adata, //the output data
	input in_clr, //clears the entire register file
	input in_write, //stores the Cdata in the Cselect location
	input in_read, //retrieves the Adata from Aselect location.
	input in_clk, //clock in
	input in_BAout, //base address in signal.
	output [31:0] out_R12);


	wire [15:0] w_A_select;
	wire [15:0] w_C_select;
	
	assign out_R12 = w_reg_out [3];
	
	//Decodes the output select
	decoder_4_to_16 Adecoder(
		.in_4 (in_Aselect),
		.out_16 (w_A_select),
		.in_enable(1)
	);
	
	//Decodes the input select
	decoder_4_to_16 Cdecoder(
		.in_4 (in_Cselect),
		.out_16 (w_C_select),
		.in_enable(1)
	);
	
	wire [31:0] w_reg_out [0:15]; //output wire values. 
	
	
	//need a way to select which register to write to
	
	
	//This block generates the registers.
	genvar index;
	generate
	for (index = 0; index < 16; index=index+1)
		begin: gen_registers
			register_32 register_inst (
				.D (in_Cdata),
				.clr (in_clr),
				.clk (in_clk),
				.write(in_write & w_C_select[index]),
				.Q(w_reg_out[index])
			);
			
		end
	endgenerate
	
	reg [31:0] r_data_out;
	
	//this selects the correct output wire from the registers to be fed to the A output
	always @ (*) begin
		case (in_Aselect)
			16'b0000:
				r_data_out = w_reg_out[0]&(~in_BAout);
			16'b0001:
				r_data_out = w_reg_out[1];
			16'b0010:
				r_data_out = w_reg_out[2];
			16'b0011:
				r_data_out = w_reg_out[3];
			16'b0100:
				r_data_out = w_reg_out[4];
			16'b0101:
				r_data_out = w_reg_out[5];
			16'b0110:
				r_data_out = w_reg_out[6];
			16'b0111:
				r_data_out = w_reg_out[7];
			16'b1000:
				r_data_out = w_reg_out[8];
			16'b1001:
				r_data_out = w_reg_out[9];
			16'b1010:
				r_data_out = w_reg_out[10];
			16'b1011:
				r_data_out = w_reg_out[11];
			16'b1100:
				r_data_out = w_reg_out[12];
			16'b1101:
				r_data_out = w_reg_out[13];
			16'b1110:
				r_data_out = w_reg_out[14];
			16'b1111:
				r_data_out = w_reg_out[15];
			default:
				r_data_out = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
		endcase
	end
	
	
	
	assign out_Adata = r_data_out;
	
endmodule

//---------------------------------------

module registerfile_16x32_tb;

reg [31:0] r_Cdata; //the input data
reg [3:0] r_Cselect; //the input select
	//there would be a B here if we were using the 3 bus
reg [3:0] r_Aselect; //output select
wire [31:0] w_Adata; //the output data
reg r_clr; //clears the entire register file
reg r_write; //stores the Cdata in the Cselect location
reg r_read; //retrieves the Adata from Aselect location.
reg r_clk; //clock in


parameter Default = 4'b0000;
parameter s_clear_all = 4'b0001;
parameter s_write_to_first = 4'b0010;
parameter s_write_to_first_2 = 4'b0011;
parameter s_write_to_second = 4'b0100;
parameter s_read_from_second = 4'b0101;

reg [3:0] Present_state = Default;

registerfile_16x32 DUT (
	.in_Cdata(r_Cdata),
	.in_Cselect(r_Cselect),
	.in_Aselect(r_Aselect),
	.out_Adata(w_Adata),
	.in_clr(r_clr),
	.in_write(r_write),
	.in_read(r_read),
	.in_clk(r_clk)
 );
 
initial
	begin
		r_clk = 0;
		forever #10 r_clk = ~r_clk;
end

always @ (posedge r_clk)
	begin
		case (Present_state)
			Default : Present_state = s_write_to_first;
			s_write_to_first : Present_state = s_write_to_first_2;
			s_write_to_first_2 : Present_state = s_write_to_second;
			s_write_to_second : Present_state = s_read_from_second;
			s_read_from_second : Present_state = s_clear_all;
		endcase
	end
	
always @ (Present_state)
begin
	case(Present_state)
	Default: begin
		r_Cdata = 'h00000000;
		r_Cselect = 'b0000;
		r_Aselect = 'b0000;
		r_clr = 1;
		r_write = 0;
		r_read = 0;
	end
	s_clear_all: begin
		r_Cdata = 'h00000000;
		r_Cselect = 'b0000;
		r_Aselect = 'b0000;
		r_clr = 0;
		r_write = 0;
		r_read = 0;
	end
	s_write_to_first: begin
		r_Cdata = 'h11111111;
		r_Cselect = 'b0000;
		r_clr = 1;
		r_write = 1;
		r_read = 0;
	end
	s_write_to_first_2: begin
		r_Cdata = 'h11110000;
		r_Cselect = 'b0000;
		r_clr = 1;
		r_write = 1;
		r_read = 0;
	end
	s_write_to_second: begin
		r_Cdata = 'h11111111;
		r_Cselect = 'b0001;
		r_clr = 1;
		r_write = 1;
		r_read = 0;
	end
	endcase
end
	
	
 
	
 /*
 r_clk = 0;
 r_Cdata = 'h00000000;
 r_Cselect = 'b0000;
 r_Aselect = 'b0000;
 r_clr = 1;
 r_write = 0;
 r_read = 0;
 
 #period
 
 r_clk = 1;
 r_Cdata = 'h11111111;
 r_Cselect = 'b0000;
 r_Aselect = 'b0000;
 r_clr = 1;
 r_write = 0;
 r_read = 0;
 
 #period
 
 r_clk = 0;
 r_Cdata = 'h11111111;
 r_Cselect = 'b0000;
 r_Aselect = 'b0000;
 r_clr = 1;
 r_write = 1;
 r_read = 0;
 
 #period
  1;
 r_read = 0;
 
 r_clk = 1;
 r_Cdata = 'h11111111;
 r_Cselect = 'b0000;
 r_Aselect = 'b0000;
 r_clr = 1;
 r_write =
 
 */

endmodule
