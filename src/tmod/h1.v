`include "common.h.v"

module main();
    reg nrst;
    wire clk;

    //This part is for debug.=================
    reg[4:0] current_cycle;
    wire [`width_cmd-1:0] issue_bus_0, issue_bus_1, issue_bus_2;


    initial
    begin
        current_cycle <= 0;
    end

    always@(posedge clk)
    begin
        current_cycle = current_cycle + 1;
    end
    //End.==================================

    clock_generator#(5) clk_gen(clk);
    cpu cpu(nrst, clk);


    initial
        nrst  <= 1;
    initial
        #100
        nrst  <= 0;
    initial
        #200
        nrst  <= 1;


    assign issue_bus_0 = cpu.issue_bus[0];
    assign issue_bus_1 = cpu.issue_bus[1];
    assign issue_bus_2 = cpu.issue_bus[2];


    always@(posedge clk)
    // always@(cpu.pc_value or cpu.decoded_signal_word)
    // if(0)
    begin
        $monitor(
            // For system signals..===========================================
            "Tick: %d.\n", $time,
            "Current Cycle: %d\tLevel: %s\n", current_cycle, clk?"High":"Low",
            "Reset: \t\t\t%b\n", nrst,
            "PC: \t\t\t%h\n", cpu.pc_value,
            "----------------------------------------------------------------------------------------------------------------------",
            "\n",
            // For pc.===========================================
            "Fetched Inst:\n",
            "inst: \t\t\t%b\n", cpu.fetched_inst,
            // "Opcode1: \t\t\t%h\n", cpu.fetched_inst[31:26],
            // "rs1: \t\t\t%d\n", cpu.fetched_inst[25:21],
            // "rs2: \t\t\t%d\n", cpu.fetched_inst[20:16],
            // "rd: \t\t\t%d\n", cpu.fetched_inst[15:11],
            // "opcode2: \t\t\t%h\n", cpu.fetched_inst[5:0],
            // "imm: \t\t\t%h\n", cpu.fetched_inst[15:0],
            // "offset: \t\t\t%h\n", cpu.fetched_inst[25:0],
            // "----------------------------------------------------------------------------------------------------------------------",
            "\n",
            // For Decoder..===========================================
            "Decoded Signal Word: \t%b\n", cpu.decoded_signal_word,
            // "Use rs1: \t\t\t%b\n", cpu.decoded_signal_word[`use_rs1],
            // "Use rs2: \t\t\t%b\n", cpu.decoded_signal_word[`use_rs2],
            // "Use rd:  \t\t\t%b\n", cpu.decoded_signal_word[`use_rd],
            // "Use imm: \t\t\t%b\n", cpu.decoded_signal_word[`use_imm],
            // "Decoded Opcode: \t\t\t%h\n", cpu.decoded_signal_word[`decoded_opcode_end_bit:`decoded_opcode_start_bit],
            // "Inst Type:: \t\t\t%d\n", cpu.decoded_signal_word[`type_signal_end_bit:`type_signal_start_bit],
            // "rs1 addr: \t\t\t%d\n", cpu.decoded_signal_word[`rs1_addr_end_bit:`rs1_addr_start_bit],
            // "rs2 addr: \t\t\t%d\n", cpu.decoded_signal_word[`rs2_addr_end_bit:`rs2_addr_start_bit],
            // "rd addr: \t\t\t%d\n", cpu.decoded_signal_word[`rd_addr_end_bit:`rd_addr_start_bit],
            // "imm: \t\t\t%h\n", cpu.decoded_signal_word[`imm_end_bit:`imm_start_bit],
            // "value: \t\t\t%h\n", cpu.decoded_signal_word[`value_end_bit:`value_start_bit],
            // "npc: \t\t\t%h\n", cpu.decoded_signal_word[`npc_end_bit:`npc_start_bit],
            // "-------------------------------------------------------------------------------------------------------",
            "\n",
            // For iq. .===========================================
            "Decoded inst valid: \t\t\t%b\n", cpu.decoded_inst_valid,
            // "IQ received Signal Word 0: \t%b\n", cpu.insts[0],
            // "IQ received Signal Word 1: \t%b\n", cpu.insts[1],
            // "IQ received Signal Word 2: \t%b\n", cpu.insts[2],
            // "IQ received Signal Word 3: \t%b\n", cpu.insts[3],
            // "IQ received Signal Word 4: \t%b\n", cpu.insts[4],
            // "IQ received Signal Word 5: \t%b\n", cpu.insts[5],
            // "IQ received Signal Word 6: \t%b\n", cpu.insts[6],
            // "IQ received Signal Word 7: \t%b\n", cpu.insts[7],
            // "ALL Use rs1: \t\t\t%b\n", cpu.use_rs1,
            // "ALL Use rs2: \t\t\t%b\n", cpu.use_rs2,
            // "ALL Use rd:  \t\t\t%b\n", cpu.use_rd,
            // "ALL Use imm: \t\t\t%b\n", cpu.use_imm,
            // "Use rs1: \t\t\t%b\n", cpu.insts[0][`use_rs1],
            // "Use rs2: \t\t\t%b\n", cpu.insts[0][`use_rs2],
            // "Use rd:  \t\t\t%b\n", cpu.insts[0][`use_rd],
            // "Use imm: \t\t\t%b\n", cpu.insts[0][`use_imm],
            // "Decoded Opcode: \t\t\t%h\n", cpu.insts[0][`decoded_opcode_end_bit:`decoded_opcode_start_bit],
            // "Inst Type:: \t\t\t%d\n", cpu.insts[0][`type_signal_end_bit:`type_signal_start_bit],
            // "rs1 addr: \t\t\t%d\n", cpu.insts[0][`rs1_addr_end_bit:`rs1_addr_start_bit],
            // "rs2 addr: \t\t\t%d\n", cpu.insts[0][`rs2_addr_end_bit:`rs2_addr_start_bit],
            // "rd addr: \t\t\t%d\n", cpu.insts[0][`rd_addr_end_bit:`rd_addr_start_bit],
            // "imm: \t\t\t%h\n", cpu.insts[0][`imm_end_bit:`imm_start_bit],
            // "value: \t\t\t%h\n", cpu.insts[0][`value_end_bit:`value_start_bit],
            // "npc: \t\t\t%h\n", cpu.insts[0][`npc_end_bit:`npc_start_bit],
            // "-------------------------------------------------------------------------------------------------------",
            "\n",

            // For ptr. .===========================================
            "validbit:   \t\t\t%b\n", cpu.validbit,
            "validbit_next:\t\t\t%b\n", cpu.validbit_next,
            "\n",
            // "afterbit[0]: \t\t\t%b\n", cpu.afterbit[0],
            // "afterbit[1]: \t\t\t%b\n", cpu.afterbit[1],
            // "afterbit[2]: \t\t\t%b\n", cpu.afterbit[2],
            // "afterbit[3]: \t\t\t%b\n", cpu.afterbit[3],
            // "afterbit[4]: \t\t\t%b\n", cpu.afterbit[4],
            // "afterbit[5]: \t\t\t%b\n", cpu.afterbit[5],
            // "afterbit[6]: \t\t\t%b\n", cpu.afterbit[6],
            // "afterbit[7]: \t\t\t%b\n", cpu.afterbit[7],
            // "\n",

            // For dep. .===========================================
            // "fdep1bit[0]: \t\t\t%b\n", cpu.fdep1bit[0],
            // "fdep1bit[1]: \t\t\t%b\n", cpu.fdep1bit[1],
            // "fdep1bit[2]: \t\t\t%b\n", cpu.fdep1bit[2],
            // "fdep1bit[3]: \t\t\t%b\n", cpu.fdep1bit[3],
            // "fdep1bit[4]: \t\t\t%b\n", cpu.fdep1bit[4],
            // "fdep1bit[5]: \t\t\t%b\n", cpu.fdep1bit[5],
            // "fdep1bit[6]: \t\t\t%b\n", cpu.fdep1bit[6],
            // "fdep1bit[7]: \t\t\t%b\n", cpu.fdep1bit[7],
            // "\n",
            // "dep1bit[0]: \t\t\t%b\n", cpu.dep1bit[0],
            // "dep1bit[1]: \t\t\t%b\n", cpu.dep1bit[1],
            // "dep1bit[2]: \t\t\t%b\n", cpu.dep1bit[2],
            // "dep1bit[3]: \t\t\t%b\n", cpu.dep1bit[3],
            // "dep1bit[4]: \t\t\t%b\n", cpu.dep1bit[4],
            // "dep1bit[5]: \t\t\t%b\n", cpu.dep1bit[5],
            // "dep1bit[6]: \t\t\t%b\n", cpu.dep1bit[6],
            // "dep1bit[7]: \t\t\t%b\n", cpu.dep1bit[7],
            // "\n",
            "dep1pos[0]: \t\t\t%d\n", cpu.dep1pos[0],
            "dep1pos[1]: \t\t\t%d\n", cpu.dep1pos[1],
            "dep1pos[2]: \t\t\t%d\n", cpu.dep1pos[2],
            "dep1pos[3]: \t\t\t%d\n", cpu.dep1pos[3],
            "dep1pos[4]: \t\t\t%d\n", cpu.dep1pos[4],
            "dep1pos[5]: \t\t\t%d\n", cpu.dep1pos[5],
            "dep1pos[6]: \t\t\t%d\n", cpu.dep1pos[6],
            "dep1pos[7]: \t\t\t%d\n", cpu.dep1pos[7],
            "\n",
            "dep2pos[0]: \t\t\t%d\n", cpu.dep2pos[0],
            "dep2pos[1]: \t\t\t%d\n", cpu.dep2pos[1],
            "dep2pos[2]: \t\t\t%d\n", cpu.dep2pos[2],
            "dep2pos[3]: \t\t\t%d\n", cpu.dep2pos[3],
            "dep2pos[4]: \t\t\t%d\n", cpu.dep2pos[4],
            "dep2pos[5]: \t\t\t%d\n", cpu.dep2pos[5],
            "dep2pos[6]: \t\t\t%d\n", cpu.dep2pos[6],
            "dep2pos[7]: \t\t\t%d\n", cpu.dep2pos[7],
            "\n",

            // For ready. .===========================================
            "ready_next:\t\t\t%b\n", cpu.ready_next,
            "ready_alu_next:\t\t\t%b\n", cpu.ready_alu_next,
            "ready_mem_next:\t\t\t%b\n", cpu.ready_mem_next,
            "\n",

            // For alu schedule. .===========================================
            "alu_valid[0]:\t\t\t%b\n", cpu.alu_valid[0],
            "alu_valid[1]:\t\t\t%b\n", cpu.alu_valid[1],
            "alu_iqpos_next[0]:\t\t%b\n", cpu.alu_iqpos_next[0],
            "alu_iqpos_next[1]:\t\t%b\n", cpu.alu_iqpos_next[1],
            // "alu_local_ready_next[0]:\t\t%b\n", cpu.alu_local_ready_next[0],
            // "alu_local_ready_next[1]:\t\t%b\n", cpu.alu_local_ready_next[1],
            // "\n",

            // For issue bus. .===========================================
            // "alu_issue_bus[0]:\t%b\n", cpu.alu_issue_bus[0],
            // "alu_issue_bus[1]:\t%b\n", cpu.alu_issue_bus[1],
            // "mem_issue_bus:\t\t%b\n",  cpu.mem_issue_bus,
            "issue_bus[0]:\t\t%b\n",   issue_bus_0,
            "issue_bus[1]:\t\t%b\n",   issue_bus_1,
            "issue_bus[2]:\t\t%b\n",   issue_bus_2,
            // "alu_valid[1]:\t\t\t%b\n", cpu.alu_valid[1],
            // "alu_iqpos_next[0]:\t\t%b\n", cpu.alu_iqpos_next[0],
            // "alu_iqpos_next[1]:\t\t%b\n", cpu.alu_iqpos_next[1],
            // "alu_local_ready_next[0]:\t\t%b\n", cpu.alu_local_ready_next[0],
            // "alu_local_ready_next[1]:\t\t%b\n", cpu.alu_local_ready_next[1],
            // "\n",

            "\n",
            
            "valid:\t\t%b\n",     `cmd_valid(issue_bus_1),
            "a1_use_reg:\t%b\n",  `cmd_a1_use_reg(issue_bus_1),
            "rs1_addr:\t%b\n",    `cmd_rs1_addr(issue_bus_1),
            "a1_use_dep:\t%b\n",  `cmd_a1_use_dep(issue_bus_1),
            "rs1_iq_pos:\t%b\n",  `cmd_dep1(issue_bus_1),
            "a2_use_reg:\t%b\n",  `cmd_a2_use_reg(issue_bus_1),
            "rs2_addr:\t%b\n",    `cmd_rs2_addr(issue_bus_1),
            "a2_use_dep:\t%b\n",  `cmd_a2_use_dep(issue_bus_1),
            "rs2_iq_pos:\t%b\n",  `cmd_dep2(issue_bus_1),
            "rs2_use_imm:\t%b\n", `cmd_a2_use_imm(issue_bus_1),
            "rs2_imm:\t%b\n",     `cmd_imm(issue_bus_1),
            "reg_we:\t\t%b\n",    `cmd_reg_we(issue_bus_1),
            "rd_addr:\t%b\n",     `cmd_rd_addr(issue_bus_1),
            "\n",

            "valid:\t\t%b\n",     `cmd_valid(issue_bus_2),
            "a1_use_reg:\t%b\n",  `cmd_a1_use_reg(issue_bus_2),
            "rs1_addr:\t%b\n",    `cmd_rs1_addr(issue_bus_2),
            "a1_use_dep:\t%b\n",  `cmd_a1_use_dep(issue_bus_2),
            "rs1_iq_pos:\t%b\n",  `cmd_dep1(issue_bus_2),
            "a2_use_reg:\t%b\n",  `cmd_a2_use_reg(issue_bus_2),
            "rs2_addr:\t%b\n",    `cmd_rs2_addr(issue_bus_2),
            "a2_use_dep:\t%b\n",  `cmd_a2_use_dep(issue_bus_2),
            "rs2_iq_pos:\t%b\n",  `cmd_dep2(issue_bus_2),
            "rs2_use_imm:\t%b\n", `cmd_a2_use_imm(issue_bus_2),
            "rs2_imm:\t%b\n",     `cmd_imm(issue_bus_2),
            "reg_we:\t\t%b\n",    `cmd_reg_we(issue_bus_2),
            "rd_addr:\t%b\n",     `cmd_rd_addr(issue_bus_2),
            "\n",

            // "alu[0].iq_pos:\t%b\n", `cmd_iq_pos(cpu.alu_issue_bus[0]),
            // "alu[1].iq_pos:\t%b\n", `cmd_iq_pos(cpu.alu_issue_bus[1]),
            // "\n",

            "readbus_ctl:\t%b\n", cpu.readbus_ctl,
            "readbus_data:\t%b\n", cpu.readbus_data,
            "writebus:\t%b\n", cpu.writebus,
            "\n",

            // "vreg.rob[0]:\t%b\n", cpu.vreg.rob[0],
            // "vreg.rob[1]:\t%b\n", cpu.vreg.rob[1],
            // "vreg.rob[2]:\t%b\n", cpu.vreg.rob[2],
            // "vreg.rob[3]:\t%b\n", cpu.vreg.rob[3],
            // "vreg.rob[4]:\t%b\n", cpu.vreg.rob[4],
            // "vreg.rob[5]:\t%b\n", cpu.vreg.rob[5],
            // "vreg.rob[6]:\t%b\n", cpu.vreg.rob[6],
            // "vreg.rob[7]:\t%b\n", cpu.vreg.rob[7],

            // "vreg.regfile[0]:\t%b\n", cpu.vreg.regfile[0],
            // "vreg.regfile[1]:\t%b\n", cpu.vreg.regfile[1],
            // "vreg.regfile[2]:\t%b\n", cpu.vreg.regfile[2],
            // "vreg.regfile[3]:\t%b\n", cpu.vreg.regfile[3],

            // "vmem.rob[0]:\t%b\n", cpu.vmem.rob[0],
            // "vmem.rob[1]:\t%b\n", cpu.vmem.rob[1],
            // "vmem.rob[2]:\t%b\n", cpu.vmem.rob[2],
            // "vmem.rob[3]:\t%b\n", cpu.vmem.rob[3],
            // "vmem.rob[4]:\t%b\n", cpu.vmem.rob[4],
            // "vmem.rob[5]:\t%b\n", cpu.vmem.rob[5],
            // "vmem.rob[6]:\t%b\n", cpu.vmem.rob[6],
            // "vmem.rob[7]:\t%b\n", cpu.vmem.rob[7],

            // "vmem.addr:\t%b\n", cpu.vmem.addr,
            // "vmem.phy_read_en:\t%b\n", cpu.vmem.phy_read_en,
            // "vmem.phy_read_data:\t%b\n", cpu.vmem.phy_read_data,
            // "vmem.phy_write_en:\t%b\n", cpu.vmem.phy_write_en,
            // "vmem.phy_write_data:\t%b\n", cpu.vmem.phy_write_data,

            "=======================================================================================================",
        );
    end
endmodule // main
