`include "architecture_specific_macro.h.v"


// Right now, module pc has no function about the branch, exception or
// interruption. It only increments to npc if the instruction queuen is not
// filled.
module pc
(
	input clk,
	input nreset,
	input need_insert,
	output reg[`addressing_space_width - 1:0] pc_value
);

	always@(posedge clk or negedge nreset)
		# 5
	begin
		if(~nreset)
			pc_value <= 0;
		else
		begin
            if(need_insert)
			begin
				pc_value <= pc_value + `inst_bytes;
			end
		end
	end // always@ (posedge clk, negedge nreset)
endmodule
