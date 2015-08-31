

always@(negedge nrst or posedge clk)
    if(~nrst)
        cycles <= 0;
    else
        cycles <= cycles + 1;

