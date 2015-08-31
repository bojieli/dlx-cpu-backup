// A clock generator to generate clock for the dlx cpu.
// It has a counter to count the tick time. A parameter is given when
// instantiate the module, which decides when the simulation stops.

module clock_generator
(
	output reg clk
);
	parameter FINISH_TICK_TIME = 4;

	reg[FINISH_TICK_TIME -1:0] counter;
	
	initial
	begin
		counter = 1;
		clk = 0;
	end

	always
	begin
		#500
		clk = ~clk;
		counter = counter + 1;
		if(counter == 0)
			$finish;
	end
endmodule











// This part defines paras about iq.




// This part defines paras about addressing space.











// `define imm_width       10




 //  `ifndef COMMONDEF_H


 






























 //  `ifndef COMMONDEF_H


 

// This header file define macro concerning the dlx-specified paras. Since once
// the ISA is determined, its instruction width, the format of instructions is
// also determined. Thus we use macros to represent those information.































 //  `ifndef COMMONDEF_H



// This part defines paras about the format of the ISA of dlx.




















// ==================================
// Instruction name opcode table.
// R type instructions.













// I type alu instructions.













// I type mem instructions.



// I type reg instructions.


// I type unconditional branch instructions.



// I type conditional branch instructions.



// J type Instructions.


// ==================================

 //  `ifndef DLX_MACRO_H



// This part defines paras about decoded info output from decoder.
// ======================================================














// Word width of the decoded bunch of signals and their offset.

























// ======================================================





























































































































































 //  `ifndef COMMON_H
































module cpu(nrst, clk);
    input nrst;
    input clk;

    // from Li Shuai
    wire [32 - 1:0]   pc_value;
    wire [32 - 1:0] fetched_inst;
    wire [(6 + 6 + 4 + 5 * 3 + 16 + 26 + 32) - 1:0] decoded_signal_word;
    reg                      decoded_inst_valid;


    // from Li Bojie
    wire [0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)+(1<<3)+(1<<3)-1:0] ctl_bus;
    reg  [0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1+3-1:0] issue_bus [1+2-1:0];
    // wire `type_cmd(alu_issue_bus) [`alu_num-1:0];
    // wire `type_cmd(mem_issue_bus);
    
    wire [0+((2+1)*2)*32-1:0] readbus_data;
    wire [0+((2+1)*2)*1+((2+1)*2)*1+((2+1)*2)*5+((2+1)*2)*3-1:0] readbus_ctl;
    wire [0+(2+1)*1+(2+1)*5+(2+1)*3+(2+1)*32-1:0] writebus;

    wire [(1<<3)-1:0] rob_valid;

    wire                      jump_here[(2+1)-1:0];
    wire                      should_jump;
    wire [3-1:0] jump_iq_pos;
    wire [32-1:0]      jump_next_pc;
    
    

    // from Guo Jiahua
    wire [3-1:0]   head_move;
    wire [3-1:0]   tail_move;
    wire [3-1:0]   newhead;
    wire [3-1:0]   newtail;

    reg [3-1:0]    head;
    reg [3-1:0]    tail;

    wire [(1<<3)-1:0]         headbit;
    wire [(1<<3)-1:0]         tailbit;
    reg [(1<<3)-1:0]          validbit;
    wire [(1<<3)-1:0]         validbit_next;
    wire [(1<<3)-1:0]         flushbit;
    wire [(1<<3)-1:0]         commitbit;
    wire [(1<<3)-1:0]         insertbit;

    wire                        could_insert;
    wire                        need_insert;


    reg [(1<<3)-1:0]          afterbit [(1<<3)-1:0];
    
    reg [0+1+1+1+1+1+1+6+4+5+5+5+16+26+32-1:0]       insts   [(1<<3)-1:0];

    // unpack
    wire [(1<<3)-1:0]         use_rs1;
    wire [(1<<3)-1:0]         use_rs2;
    wire [(1<<3)-1:0]         use_rd;
    wire [(1<<3)-1:0]         use_imm;
    wire [5-1:0]  rs1_addr [(1<<3)-1:0];
    wire [5-1:0]  rs2_addr [(1<<3)-1:0];
    wire [5-1:0]  rd_addr  [(1<<3)-1:0];
    wire [6-1:0]    inst_opcode [(1<<3)-1:0];
    wire [4-1:0] inst_type [(1<<3)-1:0];
    wire [(1<<3)-1:0]         is_alu;
    wire [(1<<3)-1:0]         is_mem;
    wire [(1<<3)-1:0]         is_jump;
    wire [(1<<3)-1:0]         is_alujump;

    // dep
    wire [(1<<3)-1:0]         fdep1bit  [(1<<3)-1:0];
    wire [(1<<3)-1:0]         fdep2bit  [(1<<3)-1:0];
    wire [(1<<3)-1:0]         dep1bit  [(1<<3)-1:0];
    wire [(1<<3)-1:0]         dep2bit  [(1<<3)-1:0];
    reg [3-1:0]    dep1pos  [(1<<3)-1:0];
    reg [3-1:0]    dep2pos  [(1<<3)-1:0];
    // NOTE: dep1pos and dep2pos is updated @(dep1bit) and @(dep2bit)


    reg [(1<<3)-1:0]          finished;
    wire [(1<<3)-1:0]         finish;
    wire [(1<<3)-1:0]         finished_next;
    reg [(1<<3)-1:0]          issued;
    wire [(1<<3)-1:0]         issuebit;
    reg  [(1<<3)-1:0]         issuebit_prev;
    wire [(1<<3)-1:0]         issued_next;
    

    // ready
    wire [(1<<3)-1:0]       nodep_next;
    wire [(1<<3)-1:0]       ready_next;
    wire [(1<<3)-1:0]       ready_alu_next;
    wire [(1<<3)-1:0]       ready_mem_next;


    // alu_schedule
    wire [(1<<3)-1:0]         alu_local_ready_next [2-1:0];
    wire [3-1:0]   alu_iqpos_next [2-1:0];
    wire                        alu_valid [2-1:0];
    wire [(1<<3)-1:0]         alu_issuebit;

    // mem_schedule
    wire [3-1:0]   mem_iqpos_next;
    wire                        mem_valid;
    wire [(1<<3)-1:0]         mem_issuebit;




    reg [31:0]                  cycles;
    

    genvar                      i, ci, ni, di, ai, mi, xi;

// pseudo-module declaration
//  input `type_global();
//  input `type_cmd(issue_bus) [`issue_num-1:0];
//      issue_bus[0] is memory operation
//      issue_bus[`issue_num-1:0] are ALUs
//  output should_jump;
//  output [`iq_addr_width-1:0] jump_iq_pos;
//  output [`pc_width-1:0] jump_next_pc;

assign ctl_bus[0+1+1-1:0+1] = clk;
assign ctl_bus[0+1-1:0] = nrst;

// connections between issue bus, readbus and writebus
generate
    for (i=0; i<(2+1); i=i+1)
    begin: bus_extract
        assign readbus_ctl[0 +1*( i*2) +1-1: 0 +1*( i*2)]     = issue_bus[i][0+1+1+6+5+5+3+3+16+26+1-1:0+1+1+6+5+5+3+3+16+26];
        assign readbus_ctl[0+((2+1)*2)*1+((2+1)*2)*1 +5*( i*2) +5-1: 0+((2+1)*2)*1+((2+1)*2)*1 +5*( i*2)]       = issue_bus[i][0+1+1+6+5-1:0+1+1+6];
        assign readbus_ctl[0+((2+1)*2)*1 +1*( i*2) +1-1: 0+((2+1)*2)*1 +1*( i*2)]   = issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1-1:0+1+1+6+5+5+3+3+16+26+1];
        assign readbus_ctl[0+((2+1)*2)*1+((2+1)*2)*1+((2+1)*2)*5 +3*( i*2) +3-1: 0+((2+1)*2)*1+((2+1)*2)*1+((2+1)*2)*5 +3*( i*2)]       = issue_bus[i][0+1+1+6+5+5+3-1:0+1+1+6+5+5];

        assign readbus_ctl[0 +1*( 1+i*2) +1-1: 0 +1*( 1+i*2)]   = issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1];
        assign readbus_ctl[0+((2+1)*2)*1+((2+1)*2)*1 +5*( 1+i*2) +5-1: 0+((2+1)*2)*1+((2+1)*2)*1 +5*( 1+i*2)]     = issue_bus[i][0+1+1+6+5+5-1:0+1+1+6+5];
        assign readbus_ctl[0+((2+1)*2)*1 +1*( 1+i*2) +1-1: 0+((2+1)*2)*1 +1*( 1+i*2)] = issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1];
        assign readbus_ctl[0+((2+1)*2)*1+((2+1)*2)*1+((2+1)*2)*5 +3*( 1+i*2) +3-1: 0+((2+1)*2)*1+((2+1)*2)*1+((2+1)*2)*5 +3*( 1+i*2)]     = issue_bus[i][0+1+1+6+5+5+3+3-1:0+1+1+6+5+5+3];

        assign writebus[0+(2+1)*1+(2+1)*5 +3*( i) +3-1: 0+(2+1)*1+(2+1)*5 +3*( i)]            = issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1+3-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1];

        assign writebus[0+(2+1)*1 +5*( i) +5-1: 0+(2+1)*1 +5*( i)]            = issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1] ?
                                                             8'h1F : issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1];

        assign writebus[0 +1*( i) +1-1: 0 +1*( i)]            = issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1] ?
                                                             jump_here[i] : issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5];
    end
endgenerate

vreg vreg(ctl_bus, readbus_data, readbus_ctl, writebus);




// general-purpose ALUs
wire [32-1:0] alu_output [(2+1)-1:1];
generate
    for (i=1; i<(2+1); i=i+1)
    begin: alus
        alu alu(
            issue_bus[i][0+1+1+6-1:0+1+1],
            readbus_data[0 +32*( 1+i*2) +32-1: 0 +32*( 1+i*2)],
            issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1],
            issue_bus[i][0+1+1+6+5+5+3+3+16-1:0+1+1+6+5+5+3+3],
            readbus_data[0 +32*( i*2) +32-1: 0 +32*( i*2)],
            alu_output[i]
           );
        assign writebus[0+(2+1)*1+(2+1)*5+(2+1)*3 +32*( i) +32-1: 0+(2+1)*1+(2+1)*5+(2+1)*3 +32*( i)] = issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1] ? issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1] : alu_output[i];
    end
endgenerate

// jump judge
assign jump_here[0] = 0; // ALU 0 is memory operation, never jump
generate
    for(xi=0; xi<1; xi=xi+1)
    begin: jump_judge_p
        wire jump_before[(2+1):1];
        wire [3-1:0] iq_pos_before[(2+1):1];
        wire [32-1:0] next_pc_before[(2+1):1];
        assign jump_before[1] = 0; // initialize
        assign iq_pos_before[1] = 0;
        assign next_pc_before[1] = 0;

        for (i=1; i<(2+1); i=i+1)
        begin: jump_judge
            wire [32-1:0] jump_next_pc;
            wire real_jump_here; // branch prediction failed
            assign jump_next_pc = issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1]
                ? issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1] + issue_bus[i][0+1+1+6+5+5+3+3+16+26-1:0+1+1+6+5+5+3+3+16] 
                : writebus[0+(2+1)*1+(2+1)*5+(2+1)*3 +32*( i) +32-1: 0+(2+1)*1+(2+1)*5+(2+1)*3 +32*( i)];
            assign jump_here[i] = issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1]
                || issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32] && alu_output[i] == 0
                || issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1] && alu_output[i] != 0;
            assign real_jump_here = jump_here[i] && jump_next_pc != issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32];
            assign jump_before[i+1] = jump_before[i] || real_jump_here;
            assign iq_pos_before[i+1] = real_jump_here ? issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1+3-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1] : iq_pos_before[i];
            assign next_pc_before[i+1] = real_jump_here ? jump_next_pc : next_pc_before[i];
        end
        assign should_jump = jump_before[(2+1)];
        assign jump_iq_pos = iq_pos_before[(2+1)];
        assign jump_next_pc = next_pc_before[(2+1)] - 4; // jump to (pc + offset) = (npc + offset - 4)
    end
endgenerate

// mem inst is always at position 0 of issue bus
wire [32-1:0] vmem_imm;
assign vmem_imm[16-1:0] = issue_bus[0][0+1+1+6+5+5+3+3+16-1:0+1+1+6+5+5+3+3];
generate
    for (i=16; i<32; i=i+1)
    begin: sign_extend
        assign vmem_imm[i] = vmem_imm[16-1];
    end
endgenerate
vmem vmem(ctl_bus,
     readbus_data[0 +32*( 1) +32-1: 0 +32*( 1)] + vmem_imm, // mem address
     issue_bus[0][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1+3-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1],      // which ROB should I save?
     issue_bus[0][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1],      // read enable
     writebus[0+(2+1)*1+(2+1)*5+(2+1)*3 +32*( 0) +32-1: 0+(2+1)*1+(2+1)*5+(2+1)*3 +32*( 0)],    // read from mem to reg
     issue_bus[0][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1],      // write enable
     readbus_data[0 +32*( 0) +32-1: 0 +32*( 0)]  // write to mem (from reg)
    );



assign newhead = (head + head_move)&((1<<3)-1);
assign newtail = (tail + tail_move)&((1<<3)-1);

assign headbit = 1<<head;
assign tailbit = 1<<tail;

assign validbit_next = (validbit & ~(flushbit|commitbit)) | insertbit;
assign finished_next = (finish | finished) & ~(flushbit|commitbit);
assign issued_next   = (issuebit | issued) & ~(flushbit|commitbit);


// { for debug
assign flushbit = 0;
assign commitbit = 0;
// assign finish = 0;
// }

assign could_insert = !( ((tail + 1)&((1<<3)-1)) == head );
assign need_insert  = could_insert && decoded_inst_valid;
assign head_move    = 0;
assign tail_move    = need_insert ? 1 : 0;
assign insertbit    = need_insert ? (1 << tail) : 0;


assign issuebit = alu_issuebit | mem_issuebit;

// assign finish = issuebit_prev;
assign finish = issuebit;


generate
    for(ci=0; ci<(1<<3); ci = ci+1)
    begin: afterbit_current
        for(ni=0; ni<(1<<3); ni = ni+1)
        begin: afterbit_next
            always@(negedge nrst or posedge clk)
                if(~nrst)
                begin
                    if( ci > ni )
                        afterbit[ci][ni] <= 1;
                    else
                        afterbit[ci][ni] <= 0;
                end
                else
                    afterbit[ci][ni]
                        <= afterbit[(ci-head_move)&((1<<3)-1)][(ni-head_move)&((1<<3)-1)];
        end
    end
endgenerate


always@(negedge nrst or posedge clk)
    if(~nrst)
    begin
        head <= 0;
        tail <= 0;
        validbit <= 0;
        finished <= 0;
        issued <= 0;
        issuebit_prev <= 0;
    end
    else
    begin
        head <= newhead;
        tail <= newtail;
        validbit <= validbit_next;
        finished <= finished_next;
        issued   <= issued_next;
        issuebit_prev <= issuebit;
    end // else: !if(~nrst)
    



generate
    for(i = 0; i < (1<<3); i = i+1)
    begin: unpack
        assign use_rs1[i] = insts[i][0+1-1:0];
        assign use_rs2[i] = insts[i][0+1+1-1:0+1];
        assign use_rd[i]  = insts[i][0+1+1+1-1:0+1+1];
        assign use_imm[i] = insts[i][0+1+1+1+1-1:0+1+1+1];

        assign rs1_addr[i] = insts[i][0+1+1+1+1+1+1+6+4+5-1:0+1+1+1+1+1+1+6+4];
        assign rs2_addr[i] = insts[i][0+1+1+1+1+1+1+6+4+5+5-1:0+1+1+1+1+1+1+6+4+5];
        assign rd_addr[i]  = insts[i][0+1+1+1+1+1+1+6+4+5+5+5-1:0+1+1+1+1+1+1+6+4+5+5];

        assign inst_opcode[i] = insts[i][0+1+1+1+1+1+1+6-1:0+1+1+1+1+1+1];
        assign inst_type[i]   = insts[i][0+1+1+1+1+1+1+6+4-1:0+1+1+1+1+1+1+6];

        assign is_alu[i]      = inst_type[i] == 1 || inst_type[i] == 2;
        assign is_mem[i]      = inst_type[i] == 3;
        assign is_jump[i]     = inst_type[i] == 4    ||
                                inst_type[i] == 5  ||
                                inst_type[i] == 6 ||
                                inst_type[i] == 7   ||
                                inst_type[i] == 8      ||
                                inst_type[i] == 9      ;
    end
endgenerate

assign is_alujump = is_alu | is_jump;


// iq-update
generate
    for( i = 0; i < (1<<3); i = i+1 )
    begin: iq_update
	    always@(negedge nrst or posedge clk)
	        if(~nrst)
		        insts[i] <= 0;
		        // insts[i] <= 200'hz;
	        else if(flushbit[i])
		        insts[i] <= 0;
	        else if(commitbit[i])
		        insts[i] <= 0;
	        else if(insertbit[i])
		        insts[i] <= decoded_signal_word;
    end // block: iq_update
endgenerate



function [3-1:0] encode83;
    input [(1<<3)-1:0] bits;
    casex(bits)
      8'b00000001:
          encode83 = 3'b000;
      8'b0000001x:
          encode83 = 3'b001;
      8'b000001xx:
          encode83 = 3'b010;
      8'b00001xxx:
          encode83 = 3'b011;
      8'b0001xxxx:
          encode83 = 3'b100;
      8'b001xxxxx:
          encode83 = 3'b101;
      8'b01xxxxxx:
          encode83 = 3'b110;
      8'b1xxxxxxx:
          encode83 = 3'b111;
      default:
          encode83 = 3'bxxx;
    endcase // case (bits)
endfunction // case

generate
    for(ci = 0; ci < (1<<3); ci = ci+1)
    begin: dep_current
        for(ni = 0; ni < (1<<3); ni = ni+1)
        begin: dep_next
            wire [(1<<3)-1:0] mask;
            assign fdep1bit[ci][ni] = validbit[ni] && afterbit[ci][ni] && use_rs1[ci] && use_rd[ni] && rs1_addr[ci] == rd_addr[ni];
            assign fdep2bit[ci][ni] = validbit[ni] && afterbit[ci][ni] && use_rs2[ci] && use_rd[ni] && rs2_addr[ci] == rd_addr[ni];
            if( ci < ni )
                assign mask = ( ((1<<ni)-1) & ((-1)<<(ci+1)) ) & ((1<<3)-1);
            else
                assign mask = ( ((1<<ni)-1) | ((-1)<<(ci+1)) ) & ((1<<3)-1);

            assign dep1bit[ci][ni] = (mask & fdep1bit[ci]) == 0 && fdep1bit[ci][ni];
            assign dep2bit[ci][ni] = (mask & fdep2bit[ci]) == 0 && fdep2bit[ci][ni];
        end // block: dep_next

        always@(dep1bit[ci])
            dep1pos[ci] <= encode83(dep1bit[ci]);
        always@(dep2bit[ci])
            dep2pos[ci] <= encode83(dep2bit[ci]);
        
    end // block: dep_current
endgenerate



generate
    for(ci = 0; ci < (1<<3); ci = ci+1)
    begin: ready_current
        assign nodep_next[ci] = ((dep1bit[ci]|dep2bit[ci]) & ~finished_next) == 0;
    end
endgenerate

assign ready_next = nodep_next & validbit & ~finished;
assign ready_alu_next = ready_next & is_alujump;
assign ready_mem_next = ready_next & is_mem;




generate
    for(ai=0; ai<2; ai=ai+1)
    begin: alu_schedule
        wire [3-1:0] alu_sort [(3+1) * (1<<3) - 1:0];
        wire [3-1:0] result;
        wire [(1<<3)-1:0]       this_ready;
        assign alu_local_ready_next[ai] = this_ready;
        if(ai == 0)
            assign this_ready = ready_alu_next;
        else
            assign this_ready = alu_local_ready_next[ai-1] & ~(1<<alu_iqpos_next[ai-1]);

        for(di=0; di<=3; di=di+1)
        begin: do_sort
            for(ci=0; ci<((1<<3) >> di); ci=ci+1)
            begin: do_sort_inst
                wire [3-1:0] pos1_i, pos2_i, next_i;
                wire                      valid_1, valid_2;
                if(di == 0)
                    assign alu_sort[di*(1<<3) + ci] = ci;
                else
                begin
                    assign pos1_i  = alu_sort[(di-1)*(1<<3) + ci];
                    assign pos2_i  = alu_sort[(di-1)*(1<<3) + ci + ((1<<3) >> di)];
                    assign valid_1 = this_ready[pos1_i];
                    assign valid_2 = this_ready[pos2_i];
                    assign next_i  = afterbit[pos2_i][pos1_i] ? ( valid_1 ? pos1_i : pos2_i ) : ( valid_2 ? pos2_i : pos1_i );
                    assign alu_sort[di*(1<<3) + ci] = next_i;
                end // else: !if(di == 0)
            end
        end
        assign result = alu_sort[3*(1<<3)];
        assign alu_iqpos_next[ai] = result;
        assign alu_valid[ai]      = this_ready[result];
    end
endgenerate


assign alu_issuebit = ready_alu_next & ~alu_local_ready_next[2-1];




generate
    for(mi=0; mi<1; mi=mi+1)
    begin: mem_schedule
        wire [3-1:0] mem_sort [(3+1) * (1<<3) - 1:0];
        wire [3-1:0] result;
        wire [(1<<3)-1:0]       this_ready;
        assign this_ready = ready_mem_next;

        for(di=0; di<=3; di=di+1)
        begin: do_sort
            for(ci=0; ci<((1<<3) >> di); ci=ci+1)
            begin: do_sort_inst
                wire [3-1:0] pos1_i, pos2_i, next_i;
                wire                      valid_1, valid_2;
                if(di == 0)
                    assign mem_sort[di*(1<<3) + ci] = ci;
                else
                begin
                    assign pos1_i  = mem_sort[(di-1)*(1<<3) + ci];
                    assign pos2_i  = mem_sort[(di-1)*(1<<3) + ci + ((1<<3) >> di)];
                    assign valid_1 = this_ready[pos1_i];
                    assign valid_2 = this_ready[pos2_i];
                    assign next_i  = afterbit[pos2_i][pos1_i] ? ( valid_1 ? pos1_i : pos2_i ) : ( valid_2 ? pos2_i : pos1_i );
                    assign mem_sort[di*(1<<3) + ci] = next_i;
                end // else: !if(di == 0)
            end
        end
        assign result = mem_sort[3*(1<<3)];
        assign mem_iqpos_next = result;
        assign mem_valid      = this_ready[result];
    end
endgenerate

assign mem_issuebit = ready_mem_next & (1<<mem_iqpos_next);


generate
    for(i=0; i<(2+1); i=i+1)
    begin: gen_alu_cmd
        wire [3-1:0] iq_pos;
        wire [0+1+1+1+1+1+1+6+4+5+5+5+16+26+32-1:0]    id;            // decoded instrunction
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
                issue_bus[i][0+1-1:0]    <= valid;


                issue_bus[i][0+1+1-1:0+1]  <= is_jump[iq_pos];

                issue_bus[i][0+1+1+6-1:0+1+1]   <= id[0+1+1+1+1+1+1+6-1:0+1+1+1+1+1+1];

                issue_bus[i][0+1+1+6+5-1:0+1+1+6] <= rs1_addr[iq_pos];
                issue_bus[i][0+1+1+6+5+5-1:0+1+1+6+5] <= rs2_addr[iq_pos];
                issue_bus[i][0+1+1+6+5+5+3-1:0+1+1+6+5+5]     <= dep1pos[iq_pos];
                issue_bus[i][0+1+1+6+5+5+3+3-1:0+1+1+6+5+5+3]     <= dep2pos[iq_pos];
                issue_bus[i][0+1+1+6+5+5+3+3+16-1:0+1+1+6+5+5+3+3]      <= id[0+1+1+1+1+1+1+6+4+5+5+5+16-1:0+1+1+1+1+1+1+6+4+5+5+5];
                issue_bus[i][0+1+1+6+5+5+3+3+16+26-1:0+1+1+6+5+5+3+3+16]    <= id[0+1+1+1+1+1+1+6+4+5+5+5+16+26-1:0+1+1+1+1+1+1+6+4+5+5+5+16];

                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1-1:0+1+1+6+5+5+3+3+16+26] <= dep1bit[iq_pos] == 0;
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1-1:0+1+1+6+5+5+3+3+16+26+1] <= dep1bit[iq_pos] != 0;
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1] <= dep2bit[iq_pos] == 0 && !use_imm[iq_pos];
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1] <= dep2bit[iq_pos] != 0 && !use_imm[iq_pos];
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1] <= use_imm[iq_pos];

                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1]  <= rd_addr[iq_pos];
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5]   <= valid && use_rd[iq_pos];

                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1]   <= valid && id[0+1+1+1+1+1-1:0+1+1+1+1];
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1]   <= valid && id[0+1+1+1+1+1+1-1:0+1+1+1+1+1];
                    
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1]      <= id[0+1+1+1+1+1+1+6+4+5+5+5+16+26+32-1:0+1+1+1+1+1+1+6+4+5+5+5+16+26];
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32] <= id[0+1+1+1+1+1+1+6+4+5+5+5+16+26+32-1:0+1+1+1+1+1+1+6+4+5+5+5+16+26];

                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32]     <= valid && id[0+1+1+1+1+1+1+6+4-1:0+1+1+1+1+1+1+6] == 8;
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1]     <= valid && id[0+1+1+1+1+1+1+6+4-1:0+1+1+1+1+1+1+6] == 9;
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1]      <= valid &&
                                     (
                                      id[0+1+1+1+1+1+1+6+4-1:0+1+1+1+1+1+1+6] == 4   ||
                                      id[0+1+1+1+1+1+1+6+4-1:0+1+1+1+1+1+1+6] == 5 ||
                                      id[0+1+1+1+1+1+1+6+4-1:0+1+1+1+1+1+1+6] == 7  ||
                                      id[0+1+1+1+1+1+1+6+4-1:0+1+1+1+1+1+1+6] == 6
                                      );
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1]       <= valid &&
                                     (
                                      id[0+1+1+1+1+1+1+6+4-1:0+1+1+1+1+1+1+6] == 7  ||
                                      id[0+1+1+1+1+1+1+6+4-1:0+1+1+1+1+1+1+6] == 6
                                      );
                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1]      <= valid &&
                                     (
                                      id[0+1+1+1+1+1+1+6+4-1:0+1+1+1+1+1+1+6] == 5 ||
                                      id[0+1+1+1+1+1+1+6+4-1:0+1+1+1+1+1+1+6] == 6
                                      );

                issue_bus[i][0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1+3-1:0+1+1+6+5+5+3+3+16+26+1+1+1+1+1+5+1+1+1+32+32+1+1+1+1+1]   <= iq_pos;
            end
    end // block: gen_alu_cmd
endgenerate


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






















































// Right now, module pc has no function about the branch, exception or
// interruption. It only increments to npc if the instruction queuen is not
// filled.
module pc
(
	input clk,
	input nreset,
	input need_insert,
	output reg[32 - 1:0] pc_value
);

	always@(posedge clk or negedge nreset)
		# 5
	begin
		if(~nreset)
			pc_value <= 0;
		else
		begin
            if(need_insert)
			begin
				pc_value <= pc_value + (32/8);
			end
		end
	end // always@ (posedge clk, negedge nreset)
endmodule






















































// This module depends on the actual type of memory or cache, thus up to now,
// it is almost empty, which just plays as a placeholder for the architecture
// of the overall cpu.
module imem
(
    input clk,
    input[32 -1:0] pc_value,
	output reg[32 - 1:0] inst
);

    reg [32 -1:0] saved_pc;
    always@(posedge clk)
        saved_pc <= pc_value;
    
    always@(posedge clk)
        # 10
        case(saved_pc)
           0: inst <= 32'h20620003; // ADDI r2, r3,  3; /* r2 <= r3 +  3; */  // 001000 00011 00010 0000000000000011
		   4: inst <= 32'h2041ffff; // ADDI r1, r2, -1; /* r1 <= r2 -  1; */  // 001000 00010 00001 1111111111111111
		   8: inst <= 32'h00222020; // ADD  r4, r1, r2; /* r4 <= r1 + r2; */  // 000000 00001 00010 00100 00000.100000
          // 12: inst <= 32'h00000020; // ADD
		  12: inst <= 32'h00c53820; // ADD  r7, r6, r5; /* r7 <= r6 + r5; */  // 000000 00110 00101 00111 00000.100000
          16: inst <= 32'h8c000000; // LW
          20: inst <= 32'h888888;
          default:
              inst <= 32'hxxxx;
        endcase // case (pc_value)

/* Register layout of instructions
 *                     R1  R2  R3  R4  R5  R6  R7  R8
 * initial              1   2   3   4   5   6   7   0
 * ADDI r2, r3,  3;     1   6   3   4   5   6   7   0
 * ADDI r1, r2, -1;     5   6   3   4   5   6   7   0
 * ADD  r4, r1, r2;     5   6   3   B   5   6   7   0
 * ADD  r7, r6, r5;     5   6   3   B   5   6   B   0
 */

endmodule






















































module decoder
(
	input[32 - 1:0] inst,
	input[32 - 1:0] npc,
	output[(6 + 6 + 4 + 5 * 3 + 16 + 26 + 32) - 1:0] decoded_signal_word
);

	wire use_rs1, use_rs2, use_rd, use_imm, read_mem, write_mem;
	wire[6 - 1:0] decoded_opcode;
	wire [4 - 1:0] inst_type;
	wire[5 - 1:0] rs1_addr, rs2_addr, rd_addr;
	wire[16 - 1:0] imm;
	wire[26 - 1:0] value;

	decoder_signal_generator decoder_signal_generator(inst, use_rs1, use_rs2, use_rd, use_imm, read_mem, write_mem, decoded_opcode, inst_type, rs1_addr, rs2_addr, rd_addr, imm, value);
	decoder_signal_pack decoder_signal_pack(use_rs1, use_rs2, use_rd, use_imm, read_mem, write_mem, decoded_opcode, inst_type, rs1_addr, rs2_addr, rd_addr, imm, value, npc, decoded_signal_word);
	
endmodule






















































module decoder_signal_generator
(
    input[32 - 1:0] inst,
    output reg use_rs1, use_rs2, use_rd, use_imm, read_mem, write_mem,
    output reg[6 - 1:0] decoded_opcode,
    output reg [4 - 1:0] inst_type,
    output[5 - 1:0] rs1_addr, rs2_addr,
    output reg [5 - 1:0] rd_addr,
    output[16 - 1:0] imm,
    output[26 - 1:0] value
);

    wire[6 - 1:0] opcode;
    wire[6 - 1:0] func;


    assign rs1_addr = inst[(32 - 6 - 1):((32 - 6 - 1) - 5 + 1)];
    assign rs2_addr = inst[(((32 - 6 - 1) - 5 + 1) - 1):((((32 - 6 - 1) - 5 + 1) - 1) - 5 + 1)];
    assign imm = inst[(16 - 1):0];
    assign value = inst[(26 - 1):0];

    assign opcode = inst[32 - 1:32 - 6];
    assign func = inst[6 - 1:0];

    // Actual decoding part.
    always@(*)
        # 10
    begin
        case(opcode)
            // R type instructions.
            // ========================================
            6'h0:
            begin
                // This section is the decoding part of R type instructions.
            rd_addr = inst[(((((32 - 6 - 1) - 5 + 1) - 1) - 5 + 1) - 1):((((((32 - 6 - 1) - 5 + 1) - 1) - 5 + 1) - 1) - 5 + 1)];
                case(func)
                    6'h20:
                    begin
                        use_rs1 = 1;
                        use_rs2 = 1;
                        use_rd = 1;
                        use_imm = 0;
                        read_mem = 0;
                        write_mem = 0;
                        decoded_opcode = func;
                        inst_type = 1;
                    end
                    default:
                    begin
                        use_rs1 = 0;
                        use_rs2 = 0;
                        use_rd = 0;
                        use_imm = 0;
                        read_mem = 0;
                        write_mem = 0;
                        decoded_opcode = 0;
                        inst_type = 0;
                        // $display("Error: The R type opcode being tested is wrong!");
                        // $display("%u\n", func);
                    end
                endcase
            end
            // ========================================
            
            // I type alu instructions.
            // ========================================
            6'h08:
            begin
                rd_addr = inst[(((32 - 6 - 1) - 5 + 1) - 1):((((32 - 6 - 1) - 5 + 1) - 1) - 5 + 1)];
                use_rs1 = 1;
                use_rs2 = 0;
                use_rd = 1;
                use_imm = 1;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = 2;
            end
            // ========================================

            // I type mem instructions.
            // ========================================
            6'h23:
            begin
                rd_addr = inst[(((32 - 6 - 1) - 5 + 1) - 1):((((32 - 6 - 1) - 5 + 1) - 1) - 5 + 1)];
                use_rs1 = 1;
                use_rs2 = 0;
                use_rd = 1;
                use_imm = 1;
                read_mem = 1;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = 3;
            end
            6'h2b:
            begin
                rd_addr = inst[(((32 - 6 - 1) - 5 + 1) - 1):((((32 - 6 - 1) - 5 + 1) - 1) - 5 + 1)];
                use_rs1 = 1;
                use_rs2 = 0;
                use_rd = 1;
                use_imm = 1;
                read_mem = 0;
                write_mem = 1;
                decoded_opcode = opcode;
                inst_type = 3;
            end
            // ========================================
            
            // I type reg instructions.
            // ========================================
            6'h0f:
            begin
                rd_addr = inst[(((32 - 6 - 1) - 5 + 1) - 1):((((32 - 6 - 1) - 5 + 1) - 1) - 5 + 1)];
                use_rs1 = 0;
                use_rs2 = 0;
                use_rd = 1;
                use_imm = 1;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = 2;
            end
            // ========================================
            
            // I type unconditional branch instructions.
            // ========================================
            6'h13:
            begin
                rd_addr = 5'bxxxxx;
                use_rs1 = 1;
                use_rs2 = 0;
                use_rd = 0;
                use_imm = 0;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = 6;
            end
            6'h12:
            begin
                rd_addr = 5'bxxxxx;
                use_rs1 = 1;
                use_rs2 = 0;
                use_rd = 0;
                use_imm = 0;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = 7;
            end
            // ========================================
            
            // J type Instructions.
            // ========================================
            6'h02:
            begin
                rd_addr = 5'bxxxxx;
                use_rs1 = 0;
                use_rs2 = 0;
                use_rd = 0;
                use_imm = 0;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = 4;
            end
            6'h03:
            begin
                rd_addr = 5'bxxxxx;
                use_rs1 = 0;
                use_rs2 = 0;
                use_rd = 0;
                use_imm = 0;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = 5;
            end
            // ========================================
            default:
            begin
                rd_addr = inst[(((32 - 6 - 1) - 5 + 1) - 1):((((32 - 6 - 1) - 5 + 1) - 1) - 5 + 1)];
                use_rs1 = 0;
                use_rs2 = 0;
                use_rd = 0;
                use_imm = 0;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = 0;
                inst_type = 0;
                // $display("Error: The opcode being tested is wrong!");
                // $display("%u\n", opcode);
            end
        endcase
    end

endmodule






















































module decoder_signal_pack
(
	input use_rs1, use_rs2, use_rd, use_imm, read_mem, write_mem,
	input[6 - 1:0] decoded_opcode,
	input[4 - 1:0] inst_type,
	input[5 - 1:0] rs1_addr, rs2_addr, rd_addr,
	input[16 - 1:0] imm,
	input[26 - 1:0] value,
	input[32 - 1:0] npc,
	output[(6 + 6 + 4 + 5 * 3 + 16 + 26 + 32) - 1:0] decoded_signal_word
);

	assign decoded_signal_word[0] = use_rs1;
	assign decoded_signal_word[1] = use_rs2;
	assign decoded_signal_word[2] = use_rd;
	assign decoded_signal_word[3] = use_imm;
	assign decoded_signal_word[4] = read_mem;
	assign decoded_signal_word[5] = write_mem;

	assign decoded_signal_word[(6 + 6 - 1):6] = decoded_opcode;
	assign decoded_signal_word[(((6 + 6 - 1) + 1) + 4 - 1):((6 + 6 - 1) + 1)] = inst_type;
	assign decoded_signal_word[(((((6 + 6 - 1) + 1) + 4 - 1) + 1) + 5 - 1):((((6 + 6 - 1) + 1) + 4 - 1) + 1)] = rs1_addr;
	assign decoded_signal_word[(((((((6 + 6 - 1) + 1) + 4 - 1) + 1) + 5 - 1) + 1) + 5 - 1):((((((6 + 6 - 1) + 1) + 4 - 1) + 1) + 5 - 1) + 1)] = rs2_addr;
	assign decoded_signal_word[(((((((((6 + 6 - 1) + 1) + 4 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 5 - 1):((((((((6 + 6 - 1) + 1) + 4 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1)] = rd_addr;
	assign decoded_signal_word[(((((((((((6 + 6 - 1) + 1) + 4 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 16 - 1):((((((((((6 + 6 - 1) + 1) + 4 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1)] = imm;
	assign decoded_signal_word[(((((((((((((6 + 6 - 1) + 1) + 4 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 16 - 1) + 1) + 26 - 1):((((((((((((6 + 6 - 1) + 1) + 4 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 16 - 1) + 1)] = value;
	assign decoded_signal_word[(((((((((((((((6 + 6 - 1) + 1) + 4 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 16 - 1) + 1) + 26 - 1) + 1) + 32 - 1):((((((((((((((6 + 6 - 1) + 1) + 4 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 5 - 1) + 1) + 16 - 1) + 1) + 26 - 1) + 1)] = npc;
endmodule














 //  `ifndef COMMON_H
module alu(opcode, in1, in2_is_imm, in2_imm, in2_readbus, out);

    input  [6-1:0] opcode;
    input  [32-1:0] in1, in2_readbus;
    input  in2_is_imm;
    input  [16-1:0] in2_imm;
    output [32-1:0] out;

    wire   [32-1:0] sign_extend_in2_imm;
    assign sign_extend_in2_imm[16-1:0] = in2_imm;
	genvar i;
    generate
        for (i=16; i<32; i=i+1)
        begin: sign_extend
            assign sign_extend_in2_imm[i] = in2_imm[16-1];
        end
    endgenerate
     
    wire   is_sign_extend;
    assign is_sign_extend = 1;
    wire   [32-1:0] in2;
    assign in2 = in2_is_imm ? (is_sign_extend ? sign_extend_in2_imm : in2_imm) : in2_readbus;

    assign out =
        (opcode == 6'h20 || opcode == 6'h08) ? in1 + in2 :
        (opcode == 6'h24 || opcode == 6'h0c) ? in1 & in2 :
        (opcode == 6'h25  || opcode == 6'h0d)  ? in1 | in2 :
        (opcode == 6'h22 || opcode == 6'h0a) ? in1 - in2 :
        (opcode == 6'h26 || opcode == 6'h0e) ? in1 ^ in2 :
        0;

endmodule // alu
/*
 * Commit Stage of our CPU
 * Versioned Register File
 * includes ROB (Reorder Buffer) and Registers.
 *
 * ROB: each entry in instruction queue has a 32-bit buffer for saving temporary result between Finish and Commit.
 * Registers: 32 general-purpose 32-bit registers whose values are saved when instruction commits.
 *
 * always:
 *     {ROB, regfile} => read_ports
 *
 * on Finish:
 *     write_ports => ROB
 *
 * on Commit:
 *     ROB => regfile
 */














 //  `ifndef COMMON_H
module vreg(ctl_bus, read_data, read_ctl, write);
wire [(1<<3)-1:0] headbit, tailbit, flushbit, commitbit, insertbit;
wire clk, nrst;
assign headbit = ctl_bus[0+1+1+3+3+(1<<3)-1:0+1+1+3+3];
assign tailbit = ctl_bus[0+1+1+3+3+(1<<3)+(1<<3)-1:0+1+1+3+3+(1<<3)];
assign flushbit = ctl_bus[0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)-1:0+1+1+3+3+(1<<3)+(1<<3)];
assign commitbit = ctl_bus[0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)+(1<<3)-1:0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)];
assign insertbit = ctl_bus[0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)+(1<<3)+(1<<3)-1:0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)+(1<<3)];
assign clk = ctl_bus[0+1+1-1:0+1];
assign nrst = ctl_bus[0+1-1:0];

    input [0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)+(1<<3)+(1<<3)-1:0] ctl_bus;

    output [0+((2+1)*2)*32-1:0] read_data;
    input [0+((2+1)*2)*1+((2+1)*2)*1+((2+1)*2)*5+((2+1)*2)*3-1:0] read_ctl;
    input [0+(2+1)*1+(2+1)*5+(2+1)*3+(2+1)*32-1:0] write;

    reg [32-1:0] rob[(1<<3)-1:0];                  // result buffer
    reg [5-1:0] rob_commit_addr[(1<<3)-1:0]; // write address for each instruction in IQ when it commits
    
    reg [32-1:0] regfile[(1<<5)-1:0];              // register file after commit
    
    genvar i,j,k;

    // read: ROB/regfile => readbus
    assign  read_data[0 +32*(0) +32-1: 0 +32*(0)] = regfile[read_ctl[0+((2+1)*2)*1+((2+1)*2)*1 +5*(0) +5-1: 0+((2+1)*2)*1+((2+1)*2)*1 +5*(0)]];
    generate
        for (i=1; i<((2+1)*2); i=i+1)
        begin: rob_to_readbus
            assign read_data[0 +32*( i) +32-1: 0 +32*( i)] = 
                read_ctl[0+((2+1)*2)*1 +1*( i) +1-1: 0+((2+1)*2)*1 +1*( i)] ?
                rob[read_ctl[0+((2+1)*2)*1+((2+1)*2)*1+((2+1)*2)*5 +3*( i) +3-1: 0+((2+1)*2)*1+((2+1)*2)*1+((2+1)*2)*5 +3*( i)]] : 
                regfile[read_ctl[0+((2+1)*2)*1+((2+1)*2)*1 +5*( i) +5-1: 0+((2+1)*2)*1+((2+1)*2)*1 +5*( i)]];
        end
    endgenerate

    // finish: writebus => ROB
    generate
        for (i=0; i<(1<<3); i=i+1)
        begin: writebus_to_rob
            wire update_rob_before[(2+1):0];
            wire [32-1:0] tmp_rob[(2+1):0];
            wire [5-1:0] tmp_commit_addr[(2+1):0];
            assign update_rob_before[0] = 0;
            assign tmp_rob[0] = 0;
            assign tmp_commit_addr[0] = 0;

            

            for (j=0; j<(2+1); j=j+1)
            begin: waterfall_rob_inner
                assign update_rob_before[j+1] = (write[0 +1*( j) +1-1: 0 +1*( j)] && write[0+(2+1)*1+(2+1)*5 +3*( j) +3-1: 0+(2+1)*1+(2+1)*5 +3*( j)] == i) || update_rob_before[j];
                assign tmp_rob[j+1] = (write[0 +1*( j) +1-1: 0 +1*( j)] && write[0+(2+1)*1+(2+1)*5 +3*( j) +3-1: 0+(2+1)*1+(2+1)*5 +3*( j)] == i) ? write[0+(2+1)*1+(2+1)*5+(2+1)*3 +32*( j) +32-1: 0+(2+1)*1+(2+1)*5+(2+1)*3 +32*( j)] : tmp_rob[j];
                assign tmp_commit_addr[j+1] = (write[0 +1*( j) +1-1: 0 +1*( j)] && write[0+(2+1)*1+(2+1)*5 +3*( j) +3-1: 0+(2+1)*1+(2+1)*5 +3*( j)] == i) ? write[0+(2+1)*1 +5*( j) +5-1: 0+(2+1)*1 +5*( j)] : tmp_commit_addr[j];
            end

            always@(posedge clk or negedge nrst)
            begin
                if (!nrst)
                    rob[i] <= 32'hdeadbeef;
                else if (update_rob_before[(2+1)])
                begin
                    rob[i] <= tmp_rob[(2+1)];
                    rob_commit_addr[i] <= tmp_commit_addr[(2+1)];
                end
            end
        end
    endgenerate

    // commit: ROB => regfile
    generate
        for (j=0; j<(1<<5); j=j+1)
        begin: rob_to_regfile
            wire [32-1:0] tmp_regfile[(1<<3):0];
            wire write_reg_before[(1<<3):0];
            assign tmp_regfile[0] = 0;
            assign write_reg_before[0] = 0;

            

            for (i=0; i<(1<<3); i=i+1)
            begin: write_reg_inner
                assign write_reg_before[i+1] = (!flushbit[i] && commitbit[i] && rob_commit_addr[i] == j) || write_reg_before[i];
                assign tmp_regfile[i+1]      = (!flushbit[i] && commitbit[i] && rob_commit_addr[i] == j) ? rob[i] : tmp_regfile[i];
            end

            always@(posedge clk or negedge nrst)
            begin
                if (!nrst)
                    regfile[j] <= j;
                else if (write_reg_before[(1<<3)])
                    regfile[j] <= tmp_regfile[(1<<3)];
            end
        end
    endgenerate

endmodule // vreg
/*
 * Versioned Memory
 * includes ROB (Reorder Buffer) and Memory File.
 *
 * ROB: each entry in instruction queue has a 32-bit buffer for saving temporary result between Finish and Commit.
 */














 //  `ifndef COMMON_H
module vmem(ctl_bus, addr, iq_pos, read_en, read_data, write_en, write_data);
wire [(1<<3)-1:0] headbit, tailbit, flushbit, commitbit, insertbit;
wire clk, nrst;
assign headbit = ctl_bus[0+1+1+3+3+(1<<3)-1:0+1+1+3+3];
assign tailbit = ctl_bus[0+1+1+3+3+(1<<3)+(1<<3)-1:0+1+1+3+3+(1<<3)];
assign flushbit = ctl_bus[0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)-1:0+1+1+3+3+(1<<3)+(1<<3)];
assign commitbit = ctl_bus[0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)+(1<<3)-1:0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)];
assign insertbit = ctl_bus[0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)+(1<<3)+(1<<3)-1:0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)+(1<<3)];
assign clk = ctl_bus[0+1+1-1:0+1];
assign nrst = ctl_bus[0+1-1:0];

    input [0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)+(1<<3)+(1<<3)-1:0] ctl_bus;

    input [5-1:0] addr;
    input [3-1:0] iq_pos;
    input read_en, write_en;
    output [32-1:0] read_data;
    input [32-1:0] write_data;

    reg [32-1:0] rob[(1<<3)-1:0];                   // result buffer
    reg [(1<<3)-1:0] rob_valid;                             // is rob valid? (between Finish and Commit)
    reg [5-1:0] rob_commit_addr[(1<<3)-1:0]; // write address for each instruction in IQ when it commits

    wire phy_read_en, phy_write_en;
    wire [32-1:0] phy_read_data, phy_write_data;

    // === BEGIN read data ===
    // read data from rob if the latest value of the given address is in rob, otherwise from memory

    wire [32-1:0] tmp_read_data[(1<<3):0];
    wire before_rob[(1<<3):0]; // memory address matches before index
    wire [3-1:0] latest_iq_pos[(1<<3):0]; // in case two robs have same memory address
    wire rewrite_here[(1<<3)-1:0];
    assign tmp_read_data[0] = 0;
    assign before_rob[0] = 0;
    assign latest_iq_pos[0] = 0;

    genvar i;

    generate
        for (i=0; i<(1<<3); i=i+1)
        begin: get_read_data
            assign before_rob[i+1] = before_rob[i] || (rob_valid[i] && addr == rob_commit_addr[i]);
            // direction note: commit from head, add instruction to tail, (tail < iq_pos < head)
            // if no address have been matched, or the current buffer is newer (farer from head), rewrite it
            assign rewrite_here[i] = before_rob[i+1] &&
                (!before_rob[i] || (ctl_bus[0+1+1+3-1:0+1+1] - i > ctl_bus[0+1+1+3-1:0+1+1] - latest_iq_pos[i]));
            assign latest_iq_pos[i+1] = rewrite_here[i] ? i : latest_iq_pos[i];
            assign tmp_read_data[i+1] = rewrite_here[i] ? rob[i] : tmp_read_data[i];
        end
    endgenerate

    assign phy_read_en = before_rob[(1<<3)];
    assign read_data = phy_read_en ? tmp_read_data[(1<<3)] : phy_read_data;

    // === END read data ===

    // write data to ROB
    generate
        for (i=0; i<(1<<3); i=i+1)
        begin: update_rob
            always@(posedge clk or negedge nrst)
            begin
                if (!nrst)
                begin
                    rob[i] <= 32'hbeefdead;
                    rob_commit_addr[i] <= 0;
                end
                else if (!(ctl_bus[0+1+1+3+3+(1<<3)+(1<<3)+(1<<3)-1:0+1+1+3+3+(1<<3)+(1<<3)] != 0) && write_en && iq_pos == i)
                begin
                    rob[i] <= write_data;
                    rob_commit_addr[i] <= addr;
                end
            end
        end
    endgenerate

    // commit data from ROB to memory
    wire tmp_write_en [(1<<3):0];
    wire [32-1:0] tmp_write_data [(1<<3):0];
    assign tmp_write_en[0] = 0;
    assign tmp_write_data[0] = 0;
    generate
        for (i=0; i<(1<<3); i=i+1)
        begin: rob_to_phy_mem
            always@(posedge clk or negedge nrst)
//                if (!nrst || flushbit[i] || commitbit[i])
//                    rob_valid[i] <= 0;
                if (!nrst)
                    rob_valid[i] <= 0;
                else if (flushbit[i] || commitbit[i])
                    rob_valid[i] <= 0;
            assign tmp_write_en[i+1] = tmp_write_en[i]
                || (commitbit[i] && !flushbit[i]);
            assign tmp_write_data[i+1] = tmp_write_data[i] 
                || (commitbit[i] && !flushbit[i] ? rob[i] : 0);
        end
    endgenerate
    assign phy_write_en = tmp_write_en[(1<<3)];
    assign phy_write_data = tmp_write_data[(1<<3)];

    // physical memory operations are at clock negedge
    //phy_mem(
    dmem dmem(
            ~(ctl_bus[0+1+1-1:0+1]) & ctl_bus[0+1-1:0],
            addr,
            phy_read_en, phy_read_data,
            phy_write_en, phy_write_data
           );

endmodule // vmem
/*
 * data memory
 * The clock is reversed from global clock to make sure memory operations are at negedge.
 */
module dmem(clk, addr, read_en, read_data, write_en, write_data);
    input  clk;
    input  [5-1:0] addr;
    input  read_en;
    output [32-1:0] read_data;
    reg    [32-1:0] read_data;
    input  write_en;
    input  [32-1:0] write_data;

    
    reg [32-1:0] mem [(1<<5)-1:0];
    
    // write
    genvar i;
    generate
        for (i=0; i<(1<<5); i=i+1)
        begin: dmem_write
            always@(posedge clk)
            begin
                if (write_en && i == addr)
                    mem[i] <= write_data;
            end
        end
    endgenerate

    // read
    wire [32-1:0] tmp_read_data [(1<<5):0];
    assign tmp_read_data[0] = 0;
    generate
        for (i=0; i<(1<<5); i=i+1)
        begin: dmem_read
            assign tmp_read_data[i+1] = (i == addr) ? mem[i] : tmp_read_data[i];
        end
    endgenerate
    always@(posedge clk)
        if (read_en)
            read_data <= tmp_read_data[(1<<5)];

endmodule // dmem

module main();
endmodule // main
