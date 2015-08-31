`ifndef COMMONDEF_H
`define COMMONDEF_H

`define reg_addr_width 5
`define reg_num       (1<<`reg_addr_width)
`define word_size     32

// This part defines paras about iq.
`define iq_addr_width 3
`define iq_size       (1<<`iq_addr_width)
`define iq_mask       (`iq_size-1)

// This part defines paras about addressing space.
`define addressing_space_width 32
`define pc_width `addressing_space_width


`define newinst_size  1

`define data_addr_width 5
`define data_mem_size   (1<<`data_addr_width)
`define read_ports      (`issue_num*2)
`define write_ports     `issue_num
`define opcode_width    6
// `define imm_width       10

`define alu_num       2
`define issue_num     (`alu_num+1)

`endif //  `ifndef COMMONDEF_H

