`include "architecture_specific_macro.h.v"


module decoder_signal_pack
(
	input use_rs1, use_rs2, use_rd, use_imm, read_mem, write_mem,
	input[`opcode_width - 1:0] decoded_opcode,
	input[`inst_type_signal_width - 1:0] inst_type,
	input[`reg_addr_size - 1:0] rs1_addr, rs2_addr, rd_addr,
	input[`imm_width - 1:0] imm,
	input[`value_width - 1:0] value,
	input[`pc_width - 1:0] npc,
	output[`decoded_inst_word_width - 1:0] decoded_signal_word
);

	assign decoded_signal_word[`use_rs1] = use_rs1;
	assign decoded_signal_word[`use_rs2] = use_rs2;
	assign decoded_signal_word[`use_rd] = use_rd;
	assign decoded_signal_word[`use_imm] = use_imm;
	assign decoded_signal_word[`read_mem] = read_mem;
	assign decoded_signal_word[`write_mem] = write_mem;

	assign decoded_signal_word[`decoded_opcode_end_bit:`decoded_opcode_start_bit] = decoded_opcode;
	assign decoded_signal_word[`type_signal_end_bit:`type_signal_start_bit] = inst_type;
	assign decoded_signal_word[`rs1_addr_end_bit:`rs1_addr_start_bit] = rs1_addr;
	assign decoded_signal_word[`rs2_addr_end_bit:`rs2_addr_start_bit] = rs2_addr;
	assign decoded_signal_word[`rd_addr_end_bit:`rd_addr_start_bit] = rd_addr;
	assign decoded_signal_word[`imm_end_bit:`imm_start_bit] = imm;
	assign decoded_signal_word[`value_end_bit:`value_start_bit] = value;
	assign decoded_signal_word[`npc_end_bit:`npc_start_bit] = npc;
endmodule
