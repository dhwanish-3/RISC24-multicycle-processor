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

// -------------------LOAD------------------------------- //
// load ra, rb, imm

// load $1, $2, 1
// 1010 001 010 000001 => a281

// load $2, $1, 20
// 1010 010 001 010100 => a454

// ------------------STORE---------------------------------- //

// store $6, $1, 12
// 1001 110 001 001100 => 9c4c

// store $7, $7, 10
// 1001 111 111 001010 => 9fca

// Machine code
// a28a
// a454
// 0df8
// 0ff8


// --------------- BEQ ---------------------//
// Machine code:
// b73c => 1011 011 100 111100 => beq $3, $4, 60
// b744 => 1011 011 101 000100 => beq $3, $5, 4
// 0000
// 0000
// 0000
// 20c9
// 20c8
// dffc

// --------------JAL ----------------------//
// Machine code:
// dff9 => 1101 111 111111001 => jal $7, -7


module tb_add;
    reg clk, reset;
    wire [15:0] writedata, readdata, instr, result, aluout, srca, srcb, pc, pcnext, pcbranch, signimmsh;
    wire[1:0] state;
    wire memwrite, regwrite, zero, carry;

    multi_cycle main(clk, reset, writedata,readdata,signimmsh, pcbranch, pc,pcnext, memwrite, regwrite, instr, srca,srcb, result, aluout, state, zero, carry);

    initial
    begin
        #1200 $finish;
    end

    initial
    begin
        reset = 1;
        clk = 0;
        #10 reset = 0;
        forever begin
            #10 clk = ~clk;
        end
    end

    always @ (posedge clk)
        begin
            // $display("Instruction: %h, state=%h", instr, state);
            // $display("Now: time=%0d, srca=%0d, srcb=%0d, aluout=%0d, result=%0d, mem_write=%0d, regwrite=%0d, write_data=%0d, read_data=%0d, zero=%b, carry=%b, signimmsh=%0d, pcbranch=%0d, pc=%0d, pcnext=%0d\n",$time, srca, srcb, aluout, result, memwrite, regwrite, writedata, readdata, zero, carry, signimmsh,pcbranch, pc, pcnext);
        end
endmodule