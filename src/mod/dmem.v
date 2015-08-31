/*
 * data memory
 * The clock is reversed from global clock to make sure memory operations are at negedge.
 */
module dmem(clk, addr, read_en, read_data, write_en, write_data);
    input  clk;
    input  [`data_addr_width-1:0] addr;
    input  read_en;
    output [`word_size-1:0] read_data;
    reg    [`word_size-1:0] read_data;
    input  write_en;
    input  [`word_size-1:0] write_data;

    `define dmem_size (1<<`data_addr_width)
    reg [`word_size-1:0] mem [`dmem_size-1:0];
    
    // write
    genvar i;
    generate
        for (i=0; i<`dmem_size; i=i+1)
        begin: dmem_write
            always@(posedge clk)
            begin
                if (write_en && i == addr)
                    mem[i] <= write_data;
            end
        end
    endgenerate

    // read
    wire [`word_size-1:0] tmp_read_data [`dmem_size:0];
    assign tmp_read_data[0] = 0;
    generate
        for (i=0; i<`dmem_size; i=i+1)
        begin: dmem_read
            assign tmp_read_data[i+1] = (i == addr) ? mem[i] : tmp_read_data[i];
        end
    endgenerate
    always@(posedge clk)
        if (read_en)
            read_data <= tmp_read_data[`dmem_size];

endmodule // dmem

