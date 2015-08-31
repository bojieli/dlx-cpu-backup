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
`include "common.h.v"
module vreg(ctl_bus, read_data, read_ctl, write);
    `include "global_wires.h.v"

    input `type_global(ctl_bus);

    output `type_readbus_data(read_data);
    input `type_readbus_ctl(read_ctl);
    input `type_writebus(write);

    reg [`word_size-1:0] rob[`iq_size-1:0] /* synthesis noprune */;                  // result buffer
    reg [`reg_addr_width-1:0] rob_commit_addr[`iq_size-1:0]; // write address for each instruction in IQ when it commits
    
    reg [`word_size-1:0] regfile[`reg_num-1:0] /* synthesis noprune */;             // register file after commit
    
    genvar i,j,k;

    // read: ROB/regfile => readbus
    assign  `readbus_data(read_data,0) = regfile[`readbus_ctl_addr(read_ctl,0)];
    generate
        for (i=1; i<`read_ports; i=i+1)
        begin: rob_to_readbus
            assign `readbus_data(read_data, i) = 
                `readbus_ctl_use_iq_pos(read_ctl, i) ?
                rob[`readbus_ctl_iq_pos(read_ctl, i)] : 
                regfile[`readbus_ctl_addr(read_ctl, i)];
        end
    endgenerate

    // finish: writebus => ROB
    generate
        for (i=0; i<`iq_size; i=i+1)
        begin: writebus_to_rob
            wire update_rob_before[`write_ports:0];
            wire [`word_size-1:0] tmp_rob[`write_ports:0];
            wire [`reg_addr_width-1:0] tmp_commit_addr[`write_ports:0];
            assign update_rob_before[0] = 0;
            assign tmp_rob[0] = 0;
            assign tmp_commit_addr[0] = 0;

            `define write_rob_here(iq,port) (`writebus_we(write, port) && `writebus_iq_pos(write, port) == iq)

            for (j=0; j<`write_ports; j=j+1)
            begin: waterfall_rob_inner
                assign update_rob_before[j+1] = `write_rob_here(i,j) || update_rob_before[j];
                assign tmp_rob[j+1] = `write_rob_here(i,j) ? `writebus_data(write, j) : tmp_rob[j];
                assign tmp_commit_addr[j+1] = `write_rob_here(i,j) ? `writebus_addr(write, j) : tmp_commit_addr[j];
            end

            always@(posedge clk or negedge nrst)
            begin
                if (!nrst)
                    rob[i] <= 32'hdeadbeef;
                else if (update_rob_before[`write_ports])
                begin
                    rob[i] <= tmp_rob[`write_ports];
                    rob_commit_addr[i] <= tmp_commit_addr[`write_ports];
                end
            end
        end
    endgenerate

    // commit: ROB => regfile
    generate
        for (j=0; j<`reg_num; j=j+1)
        begin: rob_to_regfile
            wire [`word_size-1:0] tmp_regfile[`iq_size:0];
            wire write_reg_before[`iq_size:0];
            assign tmp_regfile[0] = 0;
            assign write_reg_before[0] = 0;

            `define write_reg_here(i,j) (!flushbit[i] && commitbit[i] && rob_commit_addr[i] == j)

            for (i=0; i<`iq_size; i=i+1)
            begin: write_reg_inner
                assign write_reg_before[i+1] = `write_reg_here(i,j) || write_reg_before[i];
                assign tmp_regfile[i+1]      = `write_reg_here(i,j) ? rob[i] : tmp_regfile[i];
            end

            always@(posedge clk or negedge nrst)
            begin
                if (!nrst)
                    regfile[j] <= j;
                else if (write_reg_before[`iq_size])
                    regfile[j] <= tmp_regfile[`iq_size];
            end
        end
    endgenerate

endmodule // vreg
