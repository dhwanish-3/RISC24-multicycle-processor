// add rc, ra, rb;

// 0000 001 010 100 0 00 => 02a0
// 0000 100 001 001 0 00 => 0848
// 0000 110 111 111 0 10 => 0dfa
// 0000 111 111 111 0 00 => 0ffa

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
    end
    integer i;
    initial
    begin
        clk <= 0;
        for (i = 0; i < 20; i = i + 1)
        begin
            #5 clk <= ~clk;
        end
    end

    always @ (negedge clk)
        begin
            $display("Instruction: %h", instr);
            $display("State: %h", state);
            $display("Now: srca=%h, srcb=%h, aluout=%h, write_data=%h, result=%h, zero=%h, carry=%h",srca, srcb, aluout, writedata, result, zero, carry);
        end
endmodule