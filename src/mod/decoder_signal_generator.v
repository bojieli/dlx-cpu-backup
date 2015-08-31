`include "architecture_specific_macro.h.v"


module decoder_signal_generator
(
    input[`inst_width - 1:0] inst,
    output reg use_rs1, use_rs2, use_rd, use_imm, read_mem, write_mem,
    output reg[`opcode_width - 1:0] decoded_opcode,
    output reg [`inst_type_signal_width - 1:0] inst_type,
    output[`reg_addr_size - 1:0] rs1_addr, rs2_addr,
    output reg [`reg_addr_size - 1:0] rd_addr,
    output[`imm_width - 1:0] imm,
    output[`value_width - 1:0] value
);

    wire[`opcode_width - 1:0] opcode;
    wire[`func_width - 1:0] func;


    assign rs1_addr = inst[`rs1_addr_high:`rs1_addr_low];
    assign rs2_addr = inst[`rs2_addr_high:`rs2_addr_low];
    assign imm = inst[`imm_addr_high:`imm_addr_low];
    assign value = inst[`value_addr_high:`value_addr_low];

    assign opcode = inst[`inst_width - 1:`inst_width - `opcode_width];
    assign func = inst[`func_width - 1:0];

    // Actual decoding part.
    always@(*)
        # 10
    begin
        case(opcode)
            // R type instructions.
            // ========================================
            6'h0:
            begin
                // This section is the decoding part of R type instructions.
            rd_addr = inst[`rd_addr_high_r:`rd_addr_low_r];
                case(func)
                    `ADD:
                    begin
                        use_rs1 = 1;
                        use_rs2 = 1;
                        use_rd = 1;
                        use_imm = 0;
                        read_mem = 0;
                        write_mem = 0;
                        decoded_opcode = func;
                        inst_type = `R_TYPE;
                    end
                    default:
                    begin
                        use_rs1 = 0;
                        use_rs2 = 0;
                        use_rd = 0;
                        use_imm = 0;
                        read_mem = 0;
                        write_mem = 0;
                        decoded_opcode = 0;
                        inst_type = `NOP;
                        // $display("Error: The R type opcode being tested is wrong!");
                        // $display("%u\n", func);
                    end
                endcase
            end
            // ========================================
            
            // I type alu instructions.
            // ========================================
            `ADDI:
            begin
                rd_addr = inst[`rd_addr_high_i:`rd_addr_low_i];
                use_rs1 = 1;
                use_rs2 = 0;
                use_rd = 1;
                use_imm = 1;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = `I_TYPE_ALU;
            end
            // ========================================

            // I type mem instructions.
            // ========================================
            `LW:
            begin
                rd_addr = inst[`rd_addr_high_i:`rd_addr_low_i];
                use_rs1 = 1;
                use_rs2 = 0;
                use_rd = 1;
                use_imm = 1;
                read_mem = 1;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = `I_TYPE_MEM;
            end
            `SW:
            begin
                rd_addr = inst[`rd_addr_high_i:`rd_addr_low_i];
                use_rs1 = 1;
                use_rs2 = 0;
                use_rd = 1;
                use_imm = 1;
                read_mem = 0;
                write_mem = 1;
                decoded_opcode = opcode;
                inst_type = `I_TYPE_MEM;
            end
            // ========================================
            
            // I type reg instructions.
            // ========================================
            `LHI:
            begin
                rd_addr = inst[`rd_addr_high_i:`rd_addr_low_i];
                use_rs1 = 0;
                use_rs2 = 0;
                use_rd = 1;
                use_imm = 1;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = `I_TYPE_ALU;
            end
            // ========================================
            
            // I type unconditional branch instructions.
            // ========================================
            `JALR:
            begin
                rd_addr = 5'bxxxxx;
                use_rs1 = 1;
                use_rs2 = 0;
                use_rd = 0;
                use_imm = 0;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = `J_TYPE_JALR;
            end
            `JR:
            begin
                rd_addr = 5'bxxxxx;
                use_rs1 = 1;
                use_rs2 = 0;
                use_rd = 0;
                use_imm = 0;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = `J_TYPE_JR;
            end
            // ========================================
            
            // J type Instructions.
            // ========================================
            `J:
            begin
                rd_addr = 5'bxxxxx;
                use_rs1 = 0;
                use_rs2 = 0;
                use_rd = 0;
                use_imm = 0;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = `J_TYPE_J;
            end
            `JAL:
            begin
                rd_addr = 5'bxxxxx;
                use_rs1 = 0;
                use_rs2 = 0;
                use_rd = 0;
                use_imm = 0;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = opcode;
                inst_type = `J_TYPE_JAL;
            end
            // ========================================
            default:
            begin
                rd_addr = inst[`rd_addr_high_i:`rd_addr_low_i];
                use_rs1 = 0;
                use_rs2 = 0;
                use_rd = 0;
                use_imm = 0;
                read_mem = 0;
                write_mem = 0;
                decoded_opcode = 0;
                inst_type = `NOP;
                // $display("Error: The opcode being tested is wrong!");
                // $display("%u\n", opcode);
            end
        endcase
    end

endmodule
