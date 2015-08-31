

assign newhead = (head + head_move)&`iq_mask;
assign newtail = (tail + tail_move)&`iq_mask;

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

assign could_insert = !( ((tail + 1)&`iq_mask) == head );
assign need_insert  = could_insert && decoded_inst_valid;
assign head_move    = 0;
assign tail_move    = need_insert ? 1 : 0;
assign insertbit    = need_insert ? (1 << tail) : 0;


assign issuebit = alu_issuebit | mem_issuebit;

// assign finish = issuebit_prev;
assign finish = issuebit;


generate
    for(ci=0; ci<`iq_size; ci = ci+1)
    begin: afterbit_current
        for(ni=0; ni<`iq_size; ni = ni+1)
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
                        <= afterbit[(ci-head_move)&`iq_mask][(ni-head_move)&`iq_mask];
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
    

