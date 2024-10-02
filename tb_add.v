// add rc, ra, rb;

// 0000 001 010 100 0 00 => 02a0

// Machine code
// 02a0

module tb_add;
    reg clk, reset;
    wire [15:0] writedata, dataaddr, instr;
    wire[1:0] state;
    wire memwrite;

    multi_cycle main(clk, reset, writedata, dataaddr, memwrite, instr, state);

    initial
    begin
        reset <= 1;
        #22;
        reset <= 0;
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
    end

    always @ (negedge clk)
        begin
            $display("Instruction: %h", instr);
            $display("State: %h", state);
                if (writedata === 33)
                    $display("Add successfull");
                else 
                $display("Failed to Add with writedata=%h & dataaddr=%h", writedata, dataaddr);
        end
endmodule