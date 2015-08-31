
// iq-update
generate
    for( i = 0; i < `iq_size; i = i+1 )
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

