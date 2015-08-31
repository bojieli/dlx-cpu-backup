`include "architecture_specific_macro.h.v"


// This module depends on the actual type of memory or cache, thus up to now,
// it is almost empty, which just plays as a placeholder for the architecture
// of the overall cpu.
module imem
(
    input clk,
    input[`pc_width -1:0] pc_value,
	output reg[`inst_width - 1:0] inst
);

    reg [`pc_width -1:0] saved_pc;
    always@(posedge clk)
        saved_pc <= pc_value;
    
    always@(posedge clk)
        # 10
        case(saved_pc)
           0: inst <= 32'h20620003; // ADDI r2, r3,  3; /* r2 <= r3 +  3; */  // 001000 00011 00010 0000000000000011
		   4: inst <= 32'h2041ffff; // ADDI r1, r2, -1; /* r1 <= r2 -  1; */  // 001000 00010 00001 1111111111111111
		   8: inst <= 32'h00222020; // ADD  r4, r1, r2; /* r4 <= r1 + r2; */  // 000000 00001 00010 00100 00000.100000
          // 12: inst <= 32'h00000020; // ADD
		  12: inst <= 32'h00c53820; // ADD  r7, r6, r5; /* r7 <= r6 + r5; */  // 000000 00110 00101 00111 00000.100000
          16: inst <= 32'h8c000000; // LW
          20: inst <= 32'h888888;
          default:
              inst <= 32'hxxxx;
        endcase // case (pc_value)

/* Register layout of instructions
 *                     R1  R2  R3  R4  R5  R6  R7  R8
 * initial              1   2   3   4   5   6   7   0
 * ADDI r2, r3,  3;     1   6   3   4   5   6   7   0
 * ADDI r1, r2, -1;     5   6   3   4   5   6   7   0
 * ADD  r4, r1, r2;     5   6   3   B   5   6   7   0
 * ADD  r7, r6, r5;     5   6   3   B   5   6   B   0
 */

endmodule
