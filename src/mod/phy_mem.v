/*
 * physical memory
 * The clock is reversed from global clock to make sure memory operations are at negedge.
 */
module phy_mem(clk, addr, read_en, read_data, write_en, write_data);
    `include "common.h.v"

   input  clk;
   input  [`data_addr_width-1:0] addr;
   input  read_en;
   output [`word_size-1:0] read_data;
   input  write_en;
   input  [`word_size-1:0] write_data;

   altsyncram ram(
			.wren_a(write_en),
			.data_a(write_data),
			.address_a(addr),
			.clock0(clk),
			.q_a(read_data)
			);
	defparam
		ram.operation_mode = "SINGLE_PORT",
		ram.width_a = `word_size,
		ram.widthad_a = `data_addr_width,
		ram.lpm_hint = "ENABLE_RUNTIME_MOD=YES, INSTANCE_NAME=data_ram",
		ram.init_file = "inst_contains.mif";

endmodule // phy_mem
