`define width_readbus_data 0+`read_ports*`word_size
`define type_readbus_data(x) [0+`read_ports*`word_size-1:0] x
`define form_readbus_data [0+`read_ports*`word_size-1:0]
`define ref_readbus_data(x)  x[0+`read_ports*`word_size-1:0]
`define refa1_readbus_data(x , a1) x[0 +`word_size*(a1) +`word_size-1: 0 +`word_size*(a1)]
`define readbus_data(x,y) `refa1_readbus_data(x,y)
