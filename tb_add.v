// add rc, ra, rb

// add $1, $2, $4
// 0000 001 010 100 0 00 => 02a0

// add $4, $1, $1
// 0000 100 001 001 0 00 => 0848

// add $6, $7, $7
// 0000 110 111 111 0 00 => 0df8

// add $7, $7, $7
// 0000 111 111 111 0 00 => 0ff8

// Machine code
// 02a0
// 0848
// 0df8
// 0ff8

module tb_add;
    reg clk, reset;
    wire [15:0] writedata, dataaddr, instr, result, aluout, srca, srcb;
    wire[1:0] state;
    wire memwrite, zero, carry;

    multi_cycle main(clk, reset, writedata, dataaddr, memwrite, instr, srca,srcb, result, aluout, state, zero, carry);

    initial
    begin
        reset <= 1;
        #10;
        reset <= 0;
    end
    integer i;
    initial
    begin
        clk <= 0;
        for (i = 0; i < 30; i = i + 1)
        begin
            #10 clk <= ~clk;
        end
    end

    always @ (posedge clk)
        begin
            $display("Instruction: %h, state=%h", instr, state);
            $display("Now: time=%0d, srca=%0d, srcb=%0d, aluout=%0d, result=%0d, zero=%b, carry=%b\n",$time, srca, srcb, aluout, result, zero, carry);
        end
endmodule