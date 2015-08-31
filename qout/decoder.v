`include "architecture_specific_macro.h.v"


module decoder
(
	input[`inst_width - 1:0] inst,
	input[`pc_width - 1:0] npc,
	output[`decoded_inst_word_width - 1:0] decoded_signal_word
);

	wire use_rs1, use_rs2, use_rd, use_imm, read_mem, write_mem;
	wire[`opcode_width - 1:0] decoded_opcode;
	wire [`inst_type_signal_width - 1:0] inst_type;
	wire[`reg_addr_size - 1:0] rs1_addr, rs2_addr, rd_addr;
	wire[`imm_width - 1:0] imm;
	wire[`value_width - 1:0] value;

	decoder_signal_generator decoder_signal_generator(inst, use_rs1, use_rs2, use_rd, use_imm, read_mem, write_mem, decoded_opcode, inst_type, rs1_addr, rs2_addr, rd_addr, imm, value);
	decoder_signal_pack decoder_signal_pack(use_rs1, use_rs2, use_rd, use_imm, read_mem, write_mem, decoded_opcode, inst_type, rs1_addr, rs2_addr, rd_addr, imm, value, npc, decoded_signal_word);
	
endmodule
