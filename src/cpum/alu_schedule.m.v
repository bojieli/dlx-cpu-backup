

generate
    for(ai=0; ai<`alu_num; ai=ai+1)
    begin: alu_schedule
        wire [`iq_addr_width-1:0] alu_sort [(`iq_addr_width+1) * `iq_size - 1:0];
        wire [`iq_addr_width-1:0] result;
        wire [`iq_size-1:0]       this_ready;
        assign alu_local_ready_next[ai] = this_ready;
        if(ai == 0)
            assign this_ready = ready_alu_next;
        else
            assign this_ready = alu_local_ready_next[ai-1] & ~(1<<alu_iqpos_next[ai-1]);

        for(di=0; di<=`iq_addr_width; di=di+1)
        begin: do_sort
            for(ci=0; ci<(`iq_size >> di); ci=ci+1)
            begin: do_sort_inst
                wire [`iq_addr_width-1:0] pos1_i, pos2_i, next_i;
                wire                      valid_1, valid_2;
                if(di == 0)
                    assign alu_sort[di*`iq_size + ci] = ci;
                else
                begin
                    assign pos1_i  = alu_sort[(di-1)*`iq_size + ci];
                    assign pos2_i  = alu_sort[(di-1)*`iq_size + ci + (`iq_size >> di)];
                    assign valid_1 = this_ready[pos1_i];
                    assign valid_2 = this_ready[pos2_i];
                    assign next_i  = afterbit[pos2_i][pos1_i] ? ( valid_1 ? pos1_i : pos2_i ) : ( valid_2 ? pos2_i : pos1_i );
                    assign alu_sort[di*`iq_size + ci] = next_i;
                end // else: !if(di == 0)
            end
        end
        assign result = alu_sort[`iq_addr_width*`iq_size];
        assign alu_iqpos_next[ai] = result;
        assign alu_valid[ai]      = this_ready[result];
    end
endgenerate


assign alu_issuebit = ready_alu_next & ~alu_local_ready_next[`alu_num-1];


