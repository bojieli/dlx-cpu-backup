/*
 * Versioned Memory
 * includes ROB (Reorder Buffer) and Memory File.
 *
 * ROB: each entry in instruction queue has a 32-bit buffer for saving temporary result between Finish and Commit.
 */
`include "common.h.v"
module vmem(ctl_bus, addr, iq_pos, read_en, read_data, write_en, write_data);
    `include "global_wires.h.v"

    input `type_global(ctl_bus);

    input [`data_addr_width-1:0] addr;
    input [`iq_addr_width-1:0] iq_pos;
    input read_en, write_en /* synthesis keep */;
    output [`word_size-1:0] read_data /* synthesis keep */;
    input [`word_size-1:0] write_data /* synthesis keep */;

    reg [`word_size-1:0] rob[`iq_size-1:0];                   // result buffer
    reg [`iq_size-1:0] rob_valid;                             // is rob valid? (between Finish and Commit)
    reg [`data_addr_width-1:0] rob_commit_addr[`iq_size-1:0]; // write address for each instruction in IQ when it commits

    wire phy_read_en, phy_write_en /* synthesis keep */;
    wire [`word_size-1:0] phy_read_data, phy_write_data /* synthesis keep */;

    // === BEGIN read data ===
    // read data from rob if the latest value of the given address is in rob, otherwise from memory

    wire [`word_size-1:0] tmp_read_data[`iq_size:0];
    wire before_rob[`iq_size:0]; // memory address matches before index
    wire [`iq_addr_width-1:0] latest_iq_pos[`iq_size:0]; // in case two robs have same memory address
    wire rewrite_here[`iq_size-1:0];
    assign tmp_read_data[0] = 0;
    assign before_rob[0] = 0;
    assign latest_iq_pos[0] = 0;

    genvar i;

    generate
        for (i=0; i<`iq_size; i=i+1)
        begin: get_read_data
            assign before_rob[i+1] = before_rob[i] || (rob_valid[i] && addr == rob_commit_addr[i]);
            // direction note: commit from head, add instruction to tail, (tail < iq_pos < head)
            // if no address have been matched, or the current buffer is newer (farer from head), rewrite it
            assign rewrite_here[i] = before_rob[i+1] &&
                (!before_rob[i] || (`iq_head - i > `iq_head - latest_iq_pos[i]));
            assign latest_iq_pos[i+1] = rewrite_here[i] ? i : latest_iq_pos[i];
            assign tmp_read_data[i+1] = rewrite_here[i] ? rob[i] : tmp_read_data[i];
        end
    endgenerate

    assign phy_read_en = before_rob[`iq_size];
    assign read_data = phy_read_en ? tmp_read_data[`iq_size] : phy_read_data;

    // === END read data ===

    // write data to ROB
    generate
        for (i=0; i<`iq_size; i=i+1)
        begin: update_rob
            always@(posedge clk or negedge nrst)
            begin
                if (!nrst)
                begin
                    rob[i] <= 32'hbeefdead;
                    rob_commit_addr[i] <= 0;
                end
                else if (!`flush && write_en && iq_pos == i)
                begin
                    rob[i] <= write_data;
                    rob_commit_addr[i] <= addr;
                end
            end
        end
    endgenerate

    // commit data from ROB to memory
    wire tmp_write_en [`iq_size:0];
    wire [`word_size-1:0] tmp_write_data [`iq_size:0];
    assign tmp_write_en[0] = 0;
    assign tmp_write_data[0] = 0;
    generate
        for (i=0; i<`iq_size; i=i+1)
        begin: rob_to_phy_mem
            always@(posedge clk or negedge nrst)
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
    assign phy_write_en = tmp_write_en[`iq_size];
    assign phy_write_data = tmp_write_data[`iq_size];

    // physical memory operations are at clock negedge
    //phy_mem(
    dmem dmem(
            ~(`clk) & `nrst,
            addr,
            phy_read_en, phy_read_data,
            phy_write_en, phy_write_data
           );

endmodule // vmem
