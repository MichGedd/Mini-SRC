module cla_logic(input in_carry, input [3:0] in_generate, input [3:0] in_propogate, output [3:0] out_carry, output out_generate, output out_propogate);

	assign out_carry[0] = in_generate[0] | (in_propogate[0] & in_carry);
	assign out_carry[1] = in_generate[1] | (in_propogate[1] & in_generate[0]) | (in_propogate[1] & in_propogate[0] & in_carry);
	assign out_carry[2] = in_generate[2] | (in_propogate[2] & in_generate[1]) | (in_propogate[2] & in_propogate[1] & in_generate[0]) | (in_propogate[2] & in_propogate[1] & in_propogate[0] & in_carry);
	assign out_carry[3] = in_generate[3] | (in_propogate[3] & in_generate[2]) | (in_propogate[3] & in_propogate[2] & in_generate[1]) | (in_propogate[3] & in_propogate[2] & in_propogate[1] & in_generate[0]) | (in_propogate[3] & in_propogate[2] & in_propogate[1] & in_propogate[0] & in_carry);
	
	assign out_propogate = &in_propogate;
	assign out_generate = in_generate[3] | (in_generate[2] & in_propogate[3]) | (in_generate[1] & in_propogate[3] & in_propogate[2]) | (in_generate[0] & in_propogate[3] & in_propogate[2] & in_propogate[1]);

endmodule
