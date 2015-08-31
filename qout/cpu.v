`include "common.h.v"
`include "idec.table.v"

module cpu(nrst, clk, pc_value, commitbit, readbus_ctl, readbus_data, writebus);
    input nrst;
    input clk;

	output [`iq_size-1:0] commitbit;
	output [`pc_width-1:0] pc_value;
    output `type_readbus_data(readbus_data);
    output `type_readbus_ctl (readbus_ctl);
    output `type_writebus    (writebus);

    // from Li Shuai
    wire [`pc_width - 1:0]   pc_value;
    wire [`inst_width - 1:0] fetched_inst;
    wire [`decoded_inst_word_width - 1:0] decoded_signal_word;
    reg                      decoded_inst_valid;

    // from Li Bojie
    wire `type_global(ctl_bus);
    reg  [`width_cmd-1:0] issue_bus [1+`alu_num-1:0];
    // wire `type_cmd(alu_issue_bus) [`alu_num-1:0];
    // wire `type_cmd(mem_issue_bus);
    
    wire `type_readbus_data(readbus_data);
    wire `type_readbus_ctl (readbus_ctl);
    wire `type_writebus    (writebus);

    wire [`iq_size-1:0] rob_valid;

    wire                      jump_here[`issue_num-1:0];
    wire                      should_jump;
    wire [`iq_addr_width-1:0] jump_iq_pos;
    wire [`pc_width-1:0]      jump_next_pc;
    
    

    // from Guo Jiahua
    wire [`iq_addr_width-1:0]   head_move;
    wire [`iq_addr_width-1:0]   tail_move;
    wire [`iq_addr_width-1:0]   newhead;
    wire [`iq_addr_width-1:0]   newtail;

    reg [`iq_addr_width-1:0]    head;
    reg [`iq_addr_width-1:0]    tail;

    wire [`iq_size-1:0]         headbit;
    wire [`iq_size-1:0]         tailbit;
    reg [`iq_size-1:0]          validbit;
    wire [`iq_size-1:0]         validbit_next;
    wire [`iq_size-1:0]         flushbit;
    wire [`iq_size-1:0]         commitbit;
    wire [`iq_size-1:0]         insertbit;

    wire                        could_insert;
    wire                        need_insert;


    reg [`iq_size-1:0]          afterbit [`iq_size-1:0];
    
    reg [`width_idec-1:0]       insts   [`iq_size-1:0];

    // unpack
    wire [`iq_size-1:0]         use_rs1;
    wire [`iq_size-1:0]         use_rs2;
    wire [`iq_size-1:0]         use_rd;
    wire [`iq_size-1:0]         use_imm;
    wire [`reg_addr_width-1:0]  rs1_addr [`iq_size-1:0];
    wire [`reg_addr_width-1:0]  rs2_addr [`iq_size-1:0];
    wire [`reg_addr_width-1:0]  rd_addr  [`iq_size-1:0];
    wire [`opcode_width-1:0]    inst_opcode [`iq_size-1:0];
    wire [`inst_type_signal_width-1:0] inst_type [`iq_size-1:0];
    wire [`iq_size-1:0]         is_alu;
    wire [`iq_size-1:0]         is_mem;
    wire [`iq_size-1:0]         is_jump;
    wire [`iq_size-1:0]         is_alujump;

    // dep
    wire [`iq_size-1:0]         fdep1bit  [`iq_size-1:0];
    wire [`iq_size-1:0]         fdep2bit  [`iq_size-1:0];
    wire [`iq_size-1:0]         dep1bit  [`iq_size-1:0];
    wire [`iq_size-1:0]         dep2bit  [`iq_size-1:0];
    reg [`iq_addr_width-1:0]    dep1pos  [`iq_size-1:0];
    reg [`iq_addr_width-1:0]    dep2pos  [`iq_size-1:0];
    // NOTE: dep1pos and dep2pos is updated @(dep1bit) and @(dep2bit)


    reg [`iq_size-1:0]          finished;
    wire [`iq_size-1:0]         finish;
    wire [`iq_size-1:0]         finished_next;
    reg [`iq_size-1:0]          issued;
    wire [`iq_size-1:0]         issuebit;
    reg  [`iq_size-1:0]         issuebit_prev;
    wire [`iq_size-1:0]         issued_next;
    

    // ready
    wire [`iq_size-1:0]       nodep_next;
    wire [`iq_size-1:0]       ready_next;
    wire [`iq_size-1:0]       ready_alu_next;
    wire [`iq_size-1:0]       ready_mem_next;


    // alu_schedule
    wire [`iq_size-1:0]         alu_local_ready_next [`alu_num-1:0];
    wire [`iq_addr_width-1:0]   alu_iqpos_next [`alu_num-1:0];
    wire                        alu_valid [`alu_num-1:0];
    wire [`iq_size-1:0]         alu_issuebit;

    // mem_schedule
    wire [`iq_addr_width-1:0]   mem_iqpos_next;
    wire                        mem_valid;
    wire [`iq_size-1:0]         mem_issuebit;




    reg [31:0]                  cycles;
    

    genvar                      i, ci, ni, di, ai, mi, xi;

`include "cpu-boj.m.v"

`include "ptr.m.v"
`include "unpack.m.v"
`include "iq.m.v"
`include "dep.m.v"
`include "ready.m.v"
`include "alu_schedule.m.v"
`include "mem_schedule.m.v"
`include "gen_cmd.m.v"

    pc      pc(clk, nrst, 1'b1, pc_value);

    imem    imem(clk, pc_value, fetched_inst);
    decoder decoder(fetched_inst, pc_value, decoded_signal_word);

    always@(negedge nrst or posedge clk)
        if(~nrst)
            decoded_inst_valid <= 0;
        else
            decoded_inst_valid <= 1;


    // assign issue_bus[0] = mem_issue_bus;
    // generate
    //     for(ai=0; ai<`alu_num; ai=ai+1)
    //         assign issue_bus[ai+1] = alu_issue_bus[ai];
    // endgenerate

endmodule // cpu
