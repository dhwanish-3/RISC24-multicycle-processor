// 4-stage multicycle
// IF&ID, ALU, MEM, WB

//! Instantiation of modules inside always block is not allowed

// ! This thing only for ADD

module multi_cycle(input clk, reset, output [15:0] write_data, read_data, signimm_jal, pcbranch, pc, pcnext, output memwrite, regwrite, output [15:0] instr, srca, srcb, result, aluout, output reg [1:0]  state, output zero, carry);
	// wire [15:0] pc;
	// wire [15:0] instr;
	// wire [15:0] read_data;
	wire [1:0] alucontrol;

	//* state control
	// reg [1:0] state; // 00: IF & ID, 01: ALU, 10: MEM, 11: WB
	always @ (posedge reset)
	begin
		state <= 2'b00;
	end

	always @(posedge clk) begin
		if (state==2'b00)
		begin
        casex({instr[15:12], instr[1:0]})
            6'b000000: begin
                $display("Time: %0d ADD R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:3], zero, carry);
            end
            6'b000010: begin
                $display("Time: %0d ADC R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:3], zero, carry);
            end
            6'b001000: begin
                $display("Time: %0d NDU R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:3], zero, carry);
            end
            6'b001001: begin
                $display("Time: %0d NDZ R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:3], zero, carry);
            end
            6'b1010??: begin
                $display("Time: %0d LW R%d, R%d, Imm: %b  ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:0], zero, carry);
            end
            6'b1001??: begin
                $display("Time: %0d SW R%d, R%d, Imm: %b  ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:0], zero, carry);
            end
            6'b1011??: begin
                $display("Time: %0d BEQ R%d, R%d, Imm: %b  ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:0], zero, carry);
            end
            6'b1101??: begin
                $display("Time: %0d JAL R%d, Imm: %b ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:0], zero, carry);
            end
        endcase
		end
    end

    always @(posedge clk) begin
        $display("-----------------------------------------------");
    end

	always @ (posedge clk)
	begin
		if (state == 2'b00) 
			if (instr[15:12] == 4'b1101) // JAL
				state <= 2'b11;
			else
				state <= 2'b01;
		else if (state == 2'b01)
			if (instr[15:12] == 4'b1010 || instr[15:12] == 4'b1001) // LW & SW
				state <= 2'b10;
			else
				state <= 2'b11;
		else if (state == 2'b10)
			state <= 2'b11;
		else if (state == 2'b11)
			state <= 2'b00;
	end
	wire [2:0] writereg; //? remember rc or ra decided by regdest
	// wire [15:0] srca, srcb;
	wire [15:0] signimm, pcplus1;

	// Instruction fetch
	instr_memory imem(state,clk, pc, instr);

	// Instruction Decode 
	controller cntrl(state, instr[15:12], zero, memtoreg, memwrite, branch, pcsrc, alusrc, regdst, regwrite, jal, alucontrol);

	// next PC
	// next_pc_logic logic1(pc, instr, zero, branch, jal, pcnext);
	flipflop #(16) pc_reg(state, clk, reset, pcnext, pc); // updating pc
	adder pcadd1(pc, 16'b01, pcplus1); // *PC + 1
	sign_ext se(instr[5:0], signimm); // *sign extend
	sign_ext_jal se_jal(instr[8:0], signimm_jal);

	wire [15:0] immediate;
	mux2 #(16) jal_branch_mux(signimm, signimm_jal, jal, immediate);
	adder pcadd2(pc, immediate, pcbranch);
	mux2 #(16) pcbrmux(pcplus1, pcbranch,  pcsrc, pcnext);
	
	// Reg file
	wire [15:0] result_temp;
	mux2 #(3) writemux(instr[11:9], instr[5:3], regdst, writereg); // *decide write reg
	mux2 #(16) resultmux(aluout, read_data, memtoreg, result_temp); // decide data to be written back
	
	mux2 #(16) jalwritemux(result_temp, pcplus1, jal, result);

	wire adcwrite, ndcwrite;
	mux2 #(1) carrycheckmux(1'b0, 1'b1, !instr[15] & !instr[14] & !instr[13] & !instr[12] & instr[1], adcwrite); // *decide if adc
	mux2 #(1) zerocheckmux(1'b0, 1'b1, !instr[15] & !instr[14] & instr[13] & !instr[12] & instr[0], ndcwrite); // *decide if ndc
	mux2 #(1) decidewritemux(1'b0, 1'b1, (!adcwrite & !ndcwrite & regwrite) | (adcwrite & carry) | (ndcwrite & zero), regwrite); // decide regwrite
	regfile register(state, clk, reset, regwrite, instr[8:6], instr[11:9], writereg, result, pc, srca, write_data);

	// ALU
	mux2 #(16) srcbmux(write_data, signimm, alusrc, srcb); // !decides using alusrc b/w sign_ext(not there) & read data 2
	alu alu1(state, srca, srcb, alucontrol, aluout, zero, carry);

	// MEM
	data_memory dmem(state, clk, memwrite, aluout, write_data, read_data);

	// Write Back
	//! What should I do? Sol: Dont do anything regfile writes itself at the end of cycle 
	//? if I write reg instantiantion here, it will be another module not the one from above
endmodule

module data_memory(input [1:0] state, input clk, we, input [15:0] addr, wd, output [15:0] rd);
	reg [15:0] RAM[255:0];
	assign rd=RAM[addr];
	// integer i;
	// initial begin
	// 	for (i = 0; i < 32; i = i + 1)
	// 		RAM[i] <= 0;
	// end
	always @ (*) 
	begin
		// if (state == 2'b10)
		begin
			// rd <= RAM[addr];
			if (we)
				RAM[addr] <= wd;
		end
	end
endmodule


module instr_memory(input [1:0] state, input clk, input [15:0] addr, output reg [15:0] rd);
	reg [15:0] RAM[31:0]; // 32 registers of 16 bits
	initial
		begin
			$readmemh("memfile.dat", RAM);
		end
	always @ (*)
		if (state == 2'b00) // IF & ID
			rd = RAM[addr];
endmodule

module regfile(
		input [1:0] state,
		input clk, reset, we,
		input [2:0] ra1, ra2, wa,
		input [15:0] wd, pc,
		output reg [15:0] rd1, rd2);
	reg [15:0] register_file[7:0];
	//* NOTE: always block has only one statement, posedge of clk means at the end of the current cycle
	initial
	begin
		register_file[0] = 0;
		register_file[1] = 4;
		register_file[2] = 0;
		register_file[3] = 0;
		register_file[4] = 12;
		register_file[5] = 0;
		register_file[6] = 0;
		register_file[7] = 0;
	end
	
	always @ (posedge clk, state)
	begin
		if (state == 2'b00) // IF & ID or WB
		begin
			rd1 <= register_file[ra1];
			rd2 <= register_file[ra2];
		end
		if (state == 2'b11) // IF & ID or WB
			register_file[0] = pc;
			if (we) register_file[wa] <= wd;
	end

	always @(posedge clk) begin
		if (state == 2'b00)
    $display("Time: %0d Register values: R0 = %h, R1 = %h, R2 = %h, R3 = %h, R4 = %h, R5 = %h, R6 = %h, R7 = %h", 
                $time, register_file[0], register_file[1], register_file[2], register_file[3], register_file[4], register_file[5], register_file[6], register_file[7]);
end

endmodule

module controller (input [1:0] state, input [3:0] op,
					input zero,
					output memtoreg, memwrite, branch,
					output pcsrc, alusrc,
					output regdst, regwrite, jal,
					output [1:0] alucontrol);	
	decoder md (state, op, memtoreg, memwrite, memread, branch, alusrc, regdst, regwrite, jal);
	aludecoder ad (state, op, alucontrol);
	assign pcsrc = jal | (branch & zero);
endmodule

module decoder (input [1:0] state, input [3:0] op,
				output memtoreg, memwrite, memread,
				output branch, alusrc,
				output regdst, regwrite, jal);
				
	reg [7:0] controls;
	// regdst decides b/w ra(0) & rc(1) registers to be the destination register
	assign {regwrite, regdst, alusrc, branch, memwrite, memread, memtoreg, jal} = controls;
	always @ (*) // * can be replaced by op (since it is the only input)
		if (state == 2'b00) // IF & ID
			case(op)
				4'b0000: controls <= 8'b11000000; // Rtype
				4'b0010: controls <= 8'b11000000; // Rtype
				4'b1010: controls <= 8'b10100110; // LW
				4'b1001: controls <= 8'b00101000; // SW
				4'b1011: controls <= 8'b00010000; // BEQ
				4'b1101: controls <= 8'b10110001; //! JAL (this was updated)
				default: controls  <= 8'bxxxxxxxx; //???
			endcase
endmodule

module aludecoder (input [1:0] state, input [3:0] opcode,
				output reg [1:0] alucontrol);		
	always @ (*)
		if (state == 2'b00) // IF & ID
		  	case (opcode)
				4'b1010: alucontrol <= 2'b00; // add LW
				4'b1001: alucontrol <= 2'b00; // add SW
				4'b1011: alucontrol <= 2'b01; // sub BEQ
				4'b1101: alucontrol <= 2'b00; // add JAL (to get PC + imm)
				4'b0000: alucontrol <= 2'b00; // add RType
				4'b0010: alucontrol <= 2'b10; // nand RType
				default: alucontrol <= 2'bxx; // ???
			endcase
endmodule

module adder(input [15:0] a, b, output [15:0] y);
	assign y = a + b;
endmodule

module sl1 (input [15:0] a, output [15:0] y);
	assign y = {a[14:0], 1'b0};// changed from 2 to 1
endmodule

module sign_ext(input [5:0] a, output [15:0] y);
	assign y = {{10{a[5]}}, a};
endmodule

module sign_ext_jal(input [8:0] a, output [15:0] y);
	assign y = {{7{a[8]}}, a};
endmodule

module flipflop # (parameter WIDTH=8) (input [1:0] state, input clk, reset, input [WIDTH-1:0] d, output reg [WIDTH-1:0] q);
	always @ (negedge clk or posedge reset)
		if (reset) q <= 0;
		else if (state == 2'b00) q <= d;
endmodule

module mux2 # (parameter WIDTH = 8)(input [WIDTH-1:0] d0, d1,input s,output [WIDTH-1:0] y);
	assign y = s ? d1 : d0;
endmodule

module alu(
	input [1:0] state, input [15:0] i_data_A, i_data_B,
	input [1:0] i_alu_control, output reg [15:0] o_result,
	output reg o_zero_flag, o_carry_flag
);
	always @(*) begin
		if (state == 2'b01) // ALU
			case(i_alu_control)
				2'b00:	// ADD
					{o_carry_flag, o_result} = i_data_A + i_data_B;
				2'b01:	// SUB
					o_result = i_data_A - i_data_B;
				2'b10:	// NAND
					o_result = ~(i_data_A & i_data_B);
				default:
					o_result = {16{1'bx}};	// x-state, (nor 1, nor 0)
			endcase
			o_zero_flag = ~|o_result;
		end
endmodule