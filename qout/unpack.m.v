

generate
    for(i = 0; i < `iq_size; i = i+1)
    begin: unpack
        assign use_rs1[i] = `idec_use_rs1(insts[i]);
        assign use_rs2[i] = `idec_use_rs2(insts[i]);
        assign use_rd[i]  = `idec_use_rd(insts[i]);
        assign use_imm[i] = `idec_use_imm(insts[i]);

        assign rs1_addr[i] = `idec_rs1_addr(insts[i]);
        assign rs2_addr[i] = `idec_rs2_addr(insts[i]);
        assign rd_addr[i]  = `idec_rd_addr(insts[i]);

        assign inst_opcode[i] = `idec_opcode(insts[i]);
        assign inst_type[i]   = `idec_type(insts[i]);

        assign is_alu[i]      = inst_type[i] == `R_TYPE || inst_type[i] == `I_TYPE_ALU;
        assign is_mem[i]      = inst_type[i] == `I_TYPE_MEM;
        assign is_jump[i]     = inst_type[i] == `J_TYPE_J    ||
                                inst_type[i] == `J_TYPE_JAL  ||
                                inst_type[i] == `J_TYPE_JALR ||
                                inst_type[i] == `J_TYPE_JR   ||
                                inst_type[i] == `J_BEQZ      ||
                                inst_type[i] == `J_BNEZ      ;
    end
endgenerate

assign is_alujump = is_alu | is_jump;

