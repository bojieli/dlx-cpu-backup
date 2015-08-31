

generate
    for(mi=0; mi<1; mi=mi+1)
    begin: mem_schedule
        wire [`iq_addr_width-1:0] mem_sort [(`iq_addr_width+1) * `iq_size - 1:0];
        wire [`iq_addr_width-1:0] result;
        wire [`iq_size-1:0]       this_ready;
        assign this_ready = ready_mem_next;

        for(di=0; di<=`iq_addr_width; di=di+1)
        begin: do_sort
            for(ci=0; ci<(`iq_size >> di); ci=ci+1)
            begin: do_sort_inst
                wire [`iq_addr_width-1:0] pos1_i, pos2_i, next_i;
                wire                      valid_1, valid_2;
                if(di == 0)
                    assign mem_sort[di*`iq_size + ci] = ci;
                else
                begin
                    assign pos1_i  = mem_sort[(di-1)*`iq_size + ci];
                    assign pos2_i  = mem_sort[(di-1)*`iq_size + ci + (`iq_size >> di)];
                    assign valid_1 = this_ready[pos1_i];
                    assign valid_2 = this_ready[pos2_i];
                    assign next_i  = afterbit[pos2_i][pos1_i] ? ( valid_1 ? pos1_i : pos2_i ) : ( valid_2 ? pos2_i : pos1_i );
                    assign mem_sort[di*`iq_size + ci] = next_i;
                end // else: !if(di == 0)
            end
        end
        assign result = mem_sort[`iq_addr_width*`iq_size];
        assign mem_iqpos_next = result;
        assign mem_valid      = this_ready[result];
    end
endgenerate

assign mem_issuebit = ready_mem_next & (1<<mem_iqpos_next);

