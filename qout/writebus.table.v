`define width_writebus 0+`write_ports*1+`write_ports*`reg_addr_width+`write_ports*`iq_addr_width+`write_ports*`word_size
`define type_writebus(x) [0+`write_ports*1+`write_ports*`reg_addr_width+`write_ports*`iq_addr_width+`write_ports*`word_size-1:0] x
`define form_writebus [0+`write_ports*1+`write_ports*`reg_addr_width+`write_ports*`iq_addr_width+`write_ports*`word_size-1:0]
`define ref_writebus_we(x)  x[0+`write_ports*1-1:0]
`define refa1_writebus_we(x , a1) x[0 +1*(a1) +1-1: 0 +1*(a1)]
`define writebus_we(x,y) `refa1_writebus_we(x,y)
`define ref_writebus_addr(x)  x[0+`write_ports*1+`write_ports*`reg_addr_width-1:0+`write_ports*1]
`define refa1_writebus_addr(x , a1) x[0+`write_ports*1 +`reg_addr_width*(a1) +`reg_addr_width-1: 0+`write_ports*1 +`reg_addr_width*(a1)]
`define writebus_addr(x,y) `refa1_writebus_addr(x,y)
`define ref_writebus_iq_pos(x)  x[0+`write_ports*1+`write_ports*`reg_addr_width+`write_ports*`iq_addr_width-1:0+`write_ports*1+`write_ports*`reg_addr_width]
`define refa1_writebus_iq_pos(x , a1) x[0+`write_ports*1+`write_ports*`reg_addr_width +`iq_addr_width*(a1) +`iq_addr_width-1: 0+`write_ports*1+`write_ports*`reg_addr_width +`iq_addr_width*(a1)]
`define writebus_iq_pos(x,y) `refa1_writebus_iq_pos(x,y)
`define ref_writebus_data(x)  x[0+`write_ports*1+`write_ports*`reg_addr_width+`write_ports*`iq_addr_width+`write_ports*`word_size-1:0+`write_ports*1+`write_ports*`reg_addr_width+`write_ports*`iq_addr_width]
`define refa1_writebus_data(x , a1) x[0+`write_ports*1+`write_ports*`reg_addr_width+`write_ports*`iq_addr_width +`word_size*(a1) +`word_size-1: 0+`write_ports*1+`write_ports*`reg_addr_width+`write_ports*`iq_addr_width +`word_size*(a1)]
`define writebus_data(x,y) `refa1_writebus_data(x,y)
