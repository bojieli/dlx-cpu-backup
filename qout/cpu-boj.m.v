// pseudo-module declaration
//  input `type_global();
//  input `type_cmd(issue_bus) [`issue_num-1:0];
//      issue_bus[0] is memory operation
//      issue_bus[`issue_num-1:0] are ALUs
//  output should_jump;
//  output [`iq_addr_width-1:0] jump_iq_pos;
//  output [`pc_width-1:0] jump_next_pc;

assign `clk = clk;
assign `nrst = nrst;

// connections between issue bus, readbus and writebus
generate
    for (i=0; i<`issue_num; i=i+1)
    begin: bus_extract
        assign `readbus_ctl_use_addr(readbus_ctl, i*2)     = `cmd_a1_use_reg(issue_bus[i]);
        assign `readbus_ctl_addr  (readbus_ctl, i*2)       = `cmd_rs1_addr(issue_bus[i]);
        assign `readbus_ctl_use_iq_pos(readbus_ctl, i*2)   = `cmd_a1_use_dep(issue_bus[i]);
        assign `readbus_ctl_iq_pos(readbus_ctl, i*2)       = `cmd_dep1(issue_bus[i]);

        assign `readbus_ctl_use_addr(readbus_ctl, 1+i*2)   = `cmd_a2_use_reg(issue_bus[i]);
        assign `readbus_ctl_addr  (readbus_ctl, 1+i*2)     = `cmd_rs2_addr(issue_bus[i]);
        assign `readbus_ctl_use_iq_pos(readbus_ctl, 1+i*2) = `cmd_a2_use_dep(issue_bus[i]);
        assign `readbus_ctl_iq_pos(readbus_ctl, 1+i*2)     = `cmd_dep2(issue_bus[i]);

        assign `writebus_iq_pos   (writebus, i)            = `cmd_iq_pos(issue_bus[i]);

        assign `writebus_addr     (writebus, i)            = `cmd_jal(issue_bus[i]) ?
                                                             8'h1F : `cmd_rd_addr(issue_bus[i]);

        assign `writebus_we       (writebus, i)            = `cmd_jal(issue_bus[i]) ?
                                                             jump_here[i] : `cmd_reg_we(issue_bus[i]);
    end
endgenerate

vreg vreg(ctl_bus, readbus_data, readbus_ctl, writebus);

`define _alu_operand1(i) `readbus_data(readbus_data, 1+i*2)
`define _alu_operand2(i) `cmd_a2_use_imm(issue_bus[i]) ? `cmd_imm(issue_bus[i]) : `readbus_data(readbus_data, i*2)

// general-purpose ALUs
wire [`word_size-1:0] alu_output [`issue_num-1:1];
generate
    for (i=1; i<`issue_num; i=i+1)
    begin: alus
        alu alu(
            `cmd_opcode(issue_bus[i]),
            `readbus_data(readbus_data, 1+i*2),
            `cmd_a2_use_imm(issue_bus[i]),
            `cmd_imm(issue_bus[i]),
            `readbus_data(readbus_data, i*2),
            alu_output[i]
           );
        assign `writebus_data(writebus, i) = `cmd_jal(issue_bus[i]) ? `cmd_npc(issue_bus[i]) : alu_output[i];
    end
endgenerate

// jump judge
assign jump_here[0] = 0; // ALU 0 is memory operation, never jump
generate
    for(xi=0; xi<1; xi=xi+1)
    begin: jump_judge_p
        wire jump_before[`issue_num:1];
        wire [`iq_addr_width-1:0] iq_pos_before[`issue_num:1];
        wire [`pc_width-1:0] next_pc_before[`issue_num:1];
        assign jump_before[1] = 0; // initialize
        assign iq_pos_before[1] = 0;
        assign next_pc_before[1] = 0;

        for (i=1; i<`issue_num; i=i+1)
        begin: jump_judge
            wire [`pc_width-1:0] jump_next_pc;
            wire real_jump_here; // branch prediction failed
            assign jump_next_pc = `cmd_jr(issue_bus[i])
                ? `cmd_npc(issue_bus[i]) + `cmd_value(issue_bus[i]) 
                : `writebus_data(writebus, i);
            assign jump_here[i] = `cmd_jmp(issue_bus[i])
                || `cmd_beqz(issue_bus[i]) && alu_output[i] == 0
                || `cmd_bnez(issue_bus[i]) && alu_output[i] != 0;
            assign real_jump_here = jump_here[i] && jump_next_pc != `cmd_pred_npc(issue_bus[i]);
            assign jump_before[i+1] = jump_before[i] || real_jump_here;
            assign iq_pos_before[i+1] = real_jump_here ? `cmd_iq_pos(issue_bus[i]) : iq_pos_before[i];
            assign next_pc_before[i+1] = real_jump_here ? jump_next_pc : next_pc_before[i];
        end
        assign should_jump = jump_before[`issue_num];
        assign jump_iq_pos = iq_pos_before[`issue_num];
        assign jump_next_pc = next_pc_before[`issue_num] - 4; // jump to (pc + offset) = (npc + offset - 4)
    end
endgenerate

// mem inst is always at position 0 of issue bus
wire [`word_size-1:0] vmem_imm;
assign vmem_imm[`imm_width-1:0] = `cmd_imm(issue_bus[0]);
generate
    for (i=`imm_width; i<`word_size; i=i+1)
    begin: sign_extend
        assign vmem_imm[i] = vmem_imm[`imm_width-1];
    end
endgenerate
vmem vmem(ctl_bus,
     `readbus_data(readbus_data, 1) + vmem_imm, // mem address
     `cmd_iq_pos(issue_bus[0]),      // which ROB should I save?
     `cmd_mem_re(issue_bus[0]),      // read enable
     `writebus_data(writebus, 0),    // read from mem to reg
     `cmd_mem_we(issue_bus[0]),      // write enable
     `readbus_data(readbus_data, 0)  // write to mem (from reg)
    );
