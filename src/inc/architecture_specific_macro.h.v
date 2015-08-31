`ifndef ARCHITECTURE_MACRO_H
 `define ARCHITECTURE_MACRO_H

`include "commondef.h.v"
`include "dlx_specific_macro.h.v"


// This part defines paras about decoded info output from decoder.
// ======================================================
`define inst_type_signal_width 4
`define NOP                    0
`define R_TYPE                 1
`define I_TYPE_ALU             2
`define I_TYPE_MEM             3
`define J_TYPE_J               4
`define J_TYPE_JAL             5
`define J_TYPE_JALR            6
`define J_TYPE_JR              7
`define J_BEQZ                 8
`define J_BNEZ                 9

`define use_signal_num 6

// Word width of the decoded bunch of signals and their offset.
`define decoded_inst_word_width (`use_signal_num + `opcode_width + `inst_type_signal_width + `reg_addr_size * 3 + `imm_width + `value_width + `pc_width)
`define use_rs1 0
`define use_rs2 1
`define use_rd  2
`define use_imm 3
`define read_mem 4
`define write_mem 5
`define decoded_opcode_start_bit 6
`define decoded_opcode_end_bit (`decoded_opcode_start_bit + `opcode_width - 1)
`define type_signal_start_bit (`decoded_opcode_end_bit + 1)
`define type_signal_end_bit (`type_signal_start_bit + `inst_type_signal_width - 1)
`define rs1_addr_start_bit (`type_signal_end_bit + 1)
`define rs1_addr_end_bit (`rs1_addr_start_bit + `reg_addr_size - 1)
`define rs2_addr_start_bit (`rs1_addr_end_bit + 1)
`define rs2_addr_end_bit (`rs2_addr_start_bit + `reg_addr_size - 1)
`define rd_addr_start_bit (`rs2_addr_end_bit + 1)
`define rd_addr_end_bit (`rd_addr_start_bit + `reg_addr_size - 1)
`define imm_start_bit (`rd_addr_end_bit + 1)
`define imm_end_bit (`imm_start_bit + `imm_width - 1)
`define value_start_bit (`imm_end_bit + 1)
`define value_end_bit (`value_start_bit + `value_width - 1)
`define npc_start_bit (`value_end_bit + 1)
`define npc_end_bit (`npc_start_bit + `pc_width - 1)


// ======================================================

`endif
