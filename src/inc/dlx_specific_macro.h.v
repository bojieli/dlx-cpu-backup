`ifndef DLX_MACRO_H
 `define DLX_MACRO_H

// This header file define macro concerning the dlx-specified paras. Since once
// the ISA is determined, its instruction width, the format of instructions is
// also determined. Thus we use macros to represent those information.


`include "commondef.h.v"


// This part defines paras about the format of the ISA of dlx.
`define inst_width     `word_size
`define inst_bytes    (`word_size/8)
`define opcode_width    6
`define value_width    26
`define func_width      6
`define imm_width      16
`define reg_addr_size  `reg_addr_width
`define rs1_addr_high  (`inst_width - `opcode_width - 1)
`define rs1_addr_low   (`rs1_addr_high - `reg_addr_size + 1)
`define rs2_addr_high  (`rs1_addr_low - 1)
`define rs2_addr_low   (`rs2_addr_high - `reg_addr_size + 1)
`define rd_addr_high_i (`rs1_addr_low - 1)
`define rd_addr_low_i  (`rd_addr_high_i - `reg_addr_size + 1)
`define rd_addr_high_r (`rs2_addr_low - 1)
`define rd_addr_low_r  (`rd_addr_high_r - `reg_addr_size + 1)
`define imm_addr_high  (`imm_width - 1)
`define imm_addr_low 0
`define value_addr_high (`value_width - 1)
`define value_addr_low 0

// ==================================
// Instruction name opcode table.
// R type instructions.
`define ADD 6'h20
`define AND 6'h24
`define OR  6'h25
`define SEQ 6'h28
`define SLE 6'h2c
`define SLL 6'h04
`define SLT 6'h2a
`define SNE 6'h29
`define SRA 6'h07
`define SRL 6'h06
`define SUB 6'h22
`define XOR 6'h26

// I type alu instructions.
`define ADDI 6'h08
`define ANDI 6'h0c
`define ORI  6'h0d
`define SEQI 6'h18
`define SLEL 6'h1c
`define SLLI 6'h14
`define SLTI 6'h1a
`define SNEI 6'h19
`define SRAI 6'h17
`define SRLI 6'h16
`define SUBI 6'h0a
`define XORI 6'h0e

// I type mem instructions.
`define LW   6'h23
`define SW   6'h2b

// I type reg instructions.
`define LHI  6'h0f

// I type unconditional branch instructions.
`define JALR 6'h13
`define JR   6'h12

// I type conditional branch instructions.
`define BEQZ 6'h04
`define BNEZ 6'h05

// J type Instructions.
`define J    6'h02
`define JAL  6'h03
// ==================================

`endif //  `ifndef DLX_MACRO_H

