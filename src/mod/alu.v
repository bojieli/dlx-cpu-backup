`include "common.h.v"
module alu(opcode, in1, in2_is_imm, in2_imm, in2_readbus, out);

    input  [`opcode_width-1:0] opcode;
    input  [`word_size-1:0] in1, in2_readbus;
    input  in2_is_imm;
    input  [`imm_width-1:0] in2_imm;
    output [`word_size-1:0] out;

    wire   [`word_size-1:0] sign_extend_in2_imm;
    assign sign_extend_in2_imm[`imm_width-1:0] = in2_imm;
	genvar i;
    generate
        for (i=`imm_width; i<`word_size; i=i+1)
        begin: sign_extend
            assign sign_extend_in2_imm[i] = in2_imm[`imm_width-1];
        end
    endgenerate
     
    wire   is_sign_extend;
    assign is_sign_extend = 1;
    wire   [`word_size-1:0] in2;
    assign in2 = in2_is_imm ? (is_sign_extend ? sign_extend_in2_imm : in2_imm) : in2_readbus;

    assign out =
        (opcode == `ADD || opcode == `ADDI) ? in1 + in2 :
        (opcode == `AND || opcode == `ANDI) ? in1 & in2 :
        (opcode == `OR  || opcode == `ORI)  ? in1 | in2 :
        (opcode == `SUB || opcode == `SUBI) ? in1 - in2 :
        (opcode == `XOR || opcode == `XORI) ? in1 ^ in2 :
        0;

endmodule // alu
