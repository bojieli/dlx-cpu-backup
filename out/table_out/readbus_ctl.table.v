`define width_readbus_ctl 0+`read_ports*1+`read_ports*1+`read_ports*`reg_addr_width+`read_ports*`iq_addr_width
`define type_readbus_ctl(x) [0+`read_ports*1+`read_ports*1+`read_ports*`reg_addr_width+`read_ports*`iq_addr_width-1:0] x
`define form_readbus_ctl [0+`read_ports*1+`read_ports*1+`read_ports*`reg_addr_width+`read_ports*`iq_addr_width-1:0]
`define ref_readbus_ctl_use_addr(x)  x[0+`read_ports*1-1:0]
`define refa1_readbus_ctl_use_addr(x , a1) x[0 +1*(a1) +1-1: 0 +1*(a1)]
`define readbus_ctl_use_addr(x,y) `refa1_readbus_ctl_use_addr(x,y)
`define ref_readbus_ctl_use_iq_pos(x)  x[0+`read_ports*1+`read_ports*1-1:0+`read_ports*1]
`define refa1_readbus_ctl_use_iq_pos(x , a1) x[0+`read_ports*1 +1*(a1) +1-1: 0+`read_ports*1 +1*(a1)]
`define readbus_ctl_use_iq_pos(x,y) `refa1_readbus_ctl_use_iq_pos(x,y)
`define ref_readbus_ctl_addr(x)  x[0+`read_ports*1+`read_ports*1+`read_ports*`reg_addr_width-1:0+`read_ports*1+`read_ports*1]
`define refa1_readbus_ctl_addr(x , a1) x[0+`read_ports*1+`read_ports*1 +`reg_addr_width*(a1) +`reg_addr_width-1: 0+`read_ports*1+`read_ports*1 +`reg_addr_width*(a1)]
`define readbus_ctl_addr(x,y) `refa1_readbus_ctl_addr(x,y)
`define ref_readbus_ctl_iq_pos(x)  x[0+`read_ports*1+`read_ports*1+`read_ports*`reg_addr_width+`read_ports*`iq_addr_width-1:0+`read_ports*1+`read_ports*1+`read_ports*`reg_addr_width]
`define refa1_readbus_ctl_iq_pos(x , a1) x[0+`read_ports*1+`read_ports*1+`read_ports*`reg_addr_width +`iq_addr_width*(a1) +`iq_addr_width-1: 0+`read_ports*1+`read_ports*1+`read_ports*`reg_addr_width +`iq_addr_width*(a1)]
`define readbus_ctl_iq_pos(x,y) `refa1_readbus_ctl_iq_pos(x,y)
