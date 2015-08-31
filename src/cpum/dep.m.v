

function [`iq_addr_width-1:0] encode83;
    input [`iq_size-1:0] bits;
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
    for(ci = 0; ci < `iq_size; ci = ci+1)
    begin: dep_current
        for(ni = 0; ni < `iq_size; ni = ni+1)
        begin: dep_next
            wire [`iq_size-1:0] mask;
            assign fdep1bit[ci][ni] = validbit[ni] && afterbit[ci][ni] && use_rs1[ci] && use_rd[ni] && rs1_addr[ci] == rd_addr[ni];
            assign fdep2bit[ci][ni] = validbit[ni] && afterbit[ci][ni] && use_rs2[ci] && use_rd[ni] && rs2_addr[ci] == rd_addr[ni];
            if( ci < ni )
                assign mask = ( ((1<<ni)-1) & ((-1)<<(ci+1)) ) & `iq_mask;
            else
                assign mask = ( ((1<<ni)-1) | ((-1)<<(ci+1)) ) & `iq_mask;

            assign dep1bit[ci][ni] = (mask & fdep1bit[ci]) == 0 && fdep1bit[ci][ni];
            assign dep2bit[ci][ni] = (mask & fdep2bit[ci]) == 0 && fdep2bit[ci][ni];
        end // block: dep_next

        always@(dep1bit[ci])
            dep1pos[ci] <= encode83(dep1bit[ci]);
        always@(dep2bit[ci])
            dep2pos[ci] <= encode83(dep2bit[ci]);
        
    end // block: dep_current
endgenerate

