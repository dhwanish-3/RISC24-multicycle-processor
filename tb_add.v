// add rc, ra, rb;

// 0000 001 010 100 0 00 => 02a0
// 0000 100 001 001 0 00 => 0848

// Machine code
// 02a0
// 0848

module tb_add;
    reg clk, reset;
    wire [15:0] writedata, dataaddr, instr, result, aluout, srca, srcb;
    wire[1:0] state;
    wire memwrite, zero, carry;

    multi_cycle main(clk, reset, writedata, dataaddr, memwrite, instr, srca,srcb, result, aluout, state, zero, carry);

    initial
    begin
        reset <= 1;
        #21;
        reset <= 0;
        // #21;
        // reset <= 1;
        // #21;
        // reset <= 0;
        // #21;
    end

    initial
    begin
        clk <= 1;
        #5;
        clk <= 0;
        #5;
        clk <= 1;
        #5;
        clk <= 0;
        #5;
        clk <= 1;
        #5;
        clk <= 0;
        #5;
        clk <= 1;
        #5;
        clk <= 0;
        #5;
        clk <= 1;
        #5;
        clk <= 0;
        #5;
        clk <= 1;
        #5;
        clk <= 0;
        #5;
        clk <= 1;
        #5;
        clk <= 0;
        #5;
        clk <= 1;
        #5;
        clk <= 0;
        #5;
    end

    always @ (negedge clk)
        begin
            $display("Instruction: %h", instr);
            $display("State: %h", state);
            $display("Now: srca=%h, srcb=%h, aluout=%h, write_data=%h, result=%h, zero=%h, carry=%h",srca, srcb, aluout, writedata, result, zero, carry);
        end
endmodule