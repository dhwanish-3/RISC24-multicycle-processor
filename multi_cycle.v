// 4-stage multicycle
// IF&ID, ALU, MEM, WB

//! Instantiation of modules inside always block is not allowed

// ! This thing only for ADD


module multi_cycle(input clk, reset, output [15:0] write_data,inout [15:0] data_addr, output mem_write, output [15:0]  instr, srca, srcb, result, aluout, output reg [1:0]  state, output zero, carry);
	wire [15:0] pc;
	// wire [15:0] instr;
	wire [15:0] read_data;
	wire [1:0] alucontrol;

	//* state control
	// reg [1:0] state; // 00: IF & ID, 01: ALU, 10: MEM, 11: WB
	always @ (posedge reset)
	begin
		// pc <= 0;
		state <= 2'b00;
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
	// wire [15:0] writedata; // read data => not needed?
	wire [15:0] pcnext, signimm, pcplus2, pcbranch, signimmsh;

	// Instruction fetch
	instr_memory imem(state,clk, pc[5:1], instr);

	// Instruction Decode 
	controller cntrl(state, instr[15:12], zero, memtoreg, memwrite, pcsrc, alusrc, regdst, regwrite, alucontrol);

	// next PC
	flipflop #(16) pc_reg(state, clk, reset, pcnext, pc); // updating pc
	adder pcadd1(pc, 16'b10, pcplus2); // *PC + 2
	sign_ext se(instr[5:0], signimm); // *sign extend
	sl1 immsh(signimm, signimmsh); // shift left signimm
	adder pcadd2(pcplus2, signimmsh, pcbranch);
	mux2 #(16) pcbrmux(pcplus2, pcbranch, pcsrc, pcnext);
	
	// Reg file
	mux2 #(3) writemux(instr[11:9], instr[5:3], regdst, writereg); // *decide write reg
	mux2 #(16) resultmux(aluout, read_data, memtoreg, result);

	mux2 #(1) carrycheckmux(1'b0, 1'b1, !instr[13] & regwrite & (!instr[1] | carry), regwrite); // *decide regwrite for adc
	mux2 #(1) zerocheckmux(1'b0, 1'b1, instr[13] & regwrite & (!instr[0] | zero), regwrite); // *decide regwrite for ndc
	regfile _reg(state, clk, reset, regwrite, instr[11:9], instr[8:6], writereg, result, srca, write_data); // !have to change srcb to writereg

	// ALU
	mux2 #(16) srcbmux(write_data, signimm, alusrc, srcb); // !decides using alusrc b/w sign_ext(not there) & read data 2
	alu alu1(state, srca, srcb, alucontrol, aluout, zero, carry);

	// MEM
	data_memory dmem(state, clk, mem_write, data_addr, write_data, read_data);

	// Write Back
	//! What should I do? Sol: Dont do anything regfile writes itself at the end of cycle 
	//? if I write reg instantiantion here, it will be another module not the one from above
endmodule


module data_memory(input [1:0] state, input clk, we, input [15:0] addr, wd, output [15:0] rd);
	reg [15:0] RAM[31:0];
	assign rd = RAM[addr[15:2]];
	always @ (posedge clk) // ! may be wrong idk
		if (we && state == 2'b01)
			RAM[addr[15:2]] <= wd;
endmodule


module instr_memory(input [1:0] state, input clk, input [4:0] addr, output reg [15:0] rd);
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
		input [15:0] wd,
		output reg [15:0] rd1, rd2);
	reg [15:0] register_file[7:0];
	//* NOTE: always block has only one statement, posedge of clk means at the end of the current cycle
	initial
	begin
		register_file[0] = 0;
		register_file[1] = 16'b1111111111111111;
		register_file[2] = 16'b1111111111111111;
		register_file[3] = 33;
		register_file[4] = 44;
		register_file[5] = 55;
		register_file[6] = 65;
		register_file[7] = 10;
	end
	
	always @ (posedge clk, state)
	begin
		if (state == 2'b00) // IF & ID or WB
		begin
			rd1 <= register_file[ra1];
			rd2 <= register_file[ra2];
		end
		if (state == 2'b11) // IF & ID or WB
			if (we) register_file[wa] <= wd;
	end

endmodule

module controller (input [1:0] state, input [3:0] op,
					input zero,
					output memtoreg, memwrite,
					output pcsrc, alusrc,
					output regdst, regwrite,
					output [1:0] alucontrol);	
	wire branch;
	decoder md (state, op, memtoreg, memwrite, memread, branch, alusrc, regdst, regwrite);
	aludecoder ad (state, op, alucontrol);
	assign pcsrc = branch & zero;
endmodule

//! use muxes u see in datapath inside the modules 

module decoder (input [1:0] state, input [3:0] op,
				output memtoreg, memwrite, memread,
				output branch, alusrc,
				output regdst, regwrite);
				
	reg [6:0] controls;
	// regdst decides b/w ra(0) & rc(1) registers to be the destination register
	assign {regwrite, regdst, alusrc, branch, memwrite, memread, memtoreg} = controls;
	always @ (*) // * can be replaced by op (since it is the only input)
		if (state == 2'b00) // IF & ID
			case(op)
				4'b0000: controls <= 9'b1100000; // Rtype
				4'b0010: controls <= 9'b1100000; // Rtype
				4'b1010: controls <= 9'b1010011; // LW
				4'b1001: controls <= 9'b0010100; // SW
				4'b1011: controls <= 9'b0001000; // BEQ
				4'b1101: controls <= 9'b1011000; //! JAL (this was updated)
				default: controls  <= 9'bxxxxxxx; //???
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
	assign y = {{9{a[5]}}, a};
endmodule

module flipflop # (parameter WIDTH=8) (input [1:0] state, input clk, reset, input [WIDTH-1:0] d, output reg [WIDTH-1:0] q);
	always @ (posedge clk, posedge reset)
		if (reset) q <= 0;
		else q <= d;
endmodule

module mux2 # (parameter WIDTH = 8)(input [WIDTH-1:0] d0, d1,input s,output [WIDTH-1:0] y);
	assign y = s ? d1 : d0;
endmodule

module alu(state, i_data_A, i_data_B, i_alu_control, o_result, o_zero_flag, o_carry_flag);
	input [1:0] state;
	input [15:0] i_data_A;					// A operand 
	input [15:0] i_data_B;					// B operand
	output reg [15:0] o_result;				// ALU result
	input [1:0] i_alu_control;				// Control signal

	output reg o_zero_flag;				// Zero flag 
	output reg o_carry_flag;               // Carry flag

	always @(*) begin //? maybe change needed
		if (state == 2'b01) // ALU
			casex(i_alu_control)
				2'b00:	// ADD
					begin
						{o_carry_flag, o_result} = i_data_A + i_data_B;
						o_zero_flag = ~|o_result;
					end
				2'b01:	// SUB
					begin
						o_result = i_data_A - i_data_B;
						o_zero_flag = ~|o_result;
					end
				2'b10:	// NAND
					begin
						o_result = ~(i_data_A & i_data_B);
						o_zero_flag = ~|o_result;
					end
				default:
					begin
						o_result = {16{1'bx}};	// x-state, (nor 1, nor 0)
					end
			endcase
		end
endmodule