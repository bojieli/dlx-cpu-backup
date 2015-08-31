// A clock generator to generate clock for the dlx cpu.
// It has a counter to count the tick time. A parameter is given when
// instantiate the module, which decides when the simulation stops.

module clock_generator
(
	output reg clk
);
	parameter FINISH_TICK_TIME = 4;

	reg[FINISH_TICK_TIME -1:0] counter;
	
	initial
	begin
		counter = 1;
		clk = 0;
	end

	always
	begin
		#500
		clk = ~clk;
		counter = counter + 1;
		if(counter == 0)
			$finish;
	end
endmodule

