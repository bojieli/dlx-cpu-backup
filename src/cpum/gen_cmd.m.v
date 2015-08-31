
generate
    for(i=0; i<`issue_num; i=i+1)
    begin: gen_alu_cmd
        wire [`iq_addr_width-1:0] iq_pos;
        wire [`width_idec-1:0]    id;            // decoded instrunction
        // wire [`width_cmd-1:0]     ic;            // generated command
        wire                      valid;
        wire                      this_is_mem;

        if( i == 0 )
        begin
            assign iq_pos = mem_iqpos_next;
            assign valid  = mem_valid;
            assign this_is_mem = 1;
        end
        else
        begin
            assign iq_pos = alu_iqpos_next[i-1];
            assign valid  = alu_valid[i-1];
            assign this_is_mem = 0;
        end

        assign id = insts[iq_pos];

        always@(negedge nrst or posedge clk)
            if(~nrst)
                issue_bus[i] <= 0;
            else
            begin
                // for debug
                `cmd_valid(issue_bus[i])    <= valid;


                `cmd_is_jump(issue_bus[i])  <= is_jump[iq_pos];

                `cmd_opcode(issue_bus[i])   <= `idec_opcode(id);

                `cmd_rs1_addr(issue_bus[i]) <= rs1_addr[iq_pos];
                `cmd_rs2_addr(issue_bus[i]) <= rs2_addr[iq_pos];
                `cmd_dep1(issue_bus[i])     <= dep1pos[iq_pos];
                `cmd_dep2(issue_bus[i])     <= dep2pos[iq_pos];
                `cmd_imm(issue_bus[i])      <= `idec_imm(id);
                `cmd_value(issue_bus[i])    <= `idec_value(id);

                `cmd_a1_use_reg(issue_bus[i]) <= dep1bit[iq_pos] == 0;
                `cmd_a1_use_dep(issue_bus[i]) <= dep1bit[iq_pos] != 0;
                `cmd_a2_use_reg(issue_bus[i]) <= dep2bit[iq_pos] == 0 && !use_imm[iq_pos];
                `cmd_a2_use_dep(issue_bus[i]) <= dep2bit[iq_pos] != 0 && !use_imm[iq_pos];
                `cmd_a2_use_imm(issue_bus[i]) <= use_imm[iq_pos];

                `cmd_rd_addr(issue_bus[i])  <= rd_addr[iq_pos];
                `cmd_reg_we(issue_bus[i])   <= valid && use_rd[iq_pos];

                `cmd_mem_re(issue_bus[i])   <= valid && `idec_read_mem(id);
                `cmd_mem_we(issue_bus[i])   <= valid && `idec_write_mem(id);
                    
                `cmd_npc(issue_bus[i])      <= `idec_npc(id);
                `cmd_pred_npc(issue_bus[i]) <= `idec_npc(id);

                `cmd_beqz(issue_bus[i])     <= valid && `idec_type(id) == `J_BEQZ;
                `cmd_bnez(issue_bus[i])     <= valid && `idec_type(id) == `J_BNEZ;
                `cmd_jmp(issue_bus[i])      <= valid &&
                                     (
                                      `idec_type(id) == `J_TYPE_J   ||
                                      `idec_type(id) == `J_TYPE_JAL ||
                                      `idec_type(id) == `J_TYPE_JR  ||
                                      `idec_type(id) == `J_TYPE_JALR
                                      );
                `cmd_jr(issue_bus[i])       <= valid &&
                                     (
                                      `idec_type(id) == `J_TYPE_JR  ||
                                      `idec_type(id) == `J_TYPE_JALR
                                      );
                `cmd_jal(issue_bus[i])      <= valid &&
                                     (
                                      `idec_type(id) == `J_TYPE_JAL ||
                                      `idec_type(id) == `J_TYPE_JALR
                                      );

                `cmd_iq_pos(issue_bus[i])   <= iq_pos;
            end
    end // block: gen_alu_cmd
endgenerate

