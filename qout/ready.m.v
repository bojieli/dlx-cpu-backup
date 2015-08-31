

generate
    for(ci = 0; ci < `iq_size; ci = ci+1)
    begin: ready_current
        assign nodep_next[ci] = ((dep1bit[ci]|dep2bit[ci]) & ~finished_next) == 0;
    end
endgenerate

assign ready_next = nodep_next & validbit & ~finished;
assign ready_alu_next = ready_next & is_alujump;
assign ready_mem_next = ready_next & is_mem;


