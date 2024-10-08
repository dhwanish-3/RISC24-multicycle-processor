// 4-stage multicycle
// IF&ID, ALU, MEM, WB

module multi_cycle(input clk, reset);
	wire [15:0] pc, instr, pcbranch, result;
	wire [15:0] read_data, write_data, aluout;
	wire memwrite, regwrite;
	wire [1:0] alucontrol;
	wire [1:0] state;

	// always @(negedge clk) begin
	// 	if (state == 2'b00)
	// 	begin
    //     casex({instr[15:12], instr[1:0]})
    //         6'b000000: begin
    //             $display("Time: %0d ADD R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:3], zero, carry);
    //         end
    //         6'b000010: begin
    //             $display("Time: %0d ADC R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:3], zero, carry);
    //         end
    //         6'b001000: begin
    //             $display("Time: %0d NDU R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:3], zero, carry);
    //         end
    //         6'b001001: begin
    //             $display("Time: %0d NDZ R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:3], zero, carry);
    //         end
    //         6'b1010??: begin
    //             $display("Time: %0d LW R%d, R%d, Imm: %b  ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:0], zero, carry);
    //         end
    //         6'b1001??: begin
    //             $display("Time: %0d SW R%d, R%d, Imm: %b  ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:0], zero, carry);
    //         end
    //         6'b1011??: begin
    //             $display("Time: %0d BEQ R%d, R%d, Imm: %b  ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:6], instr[5:0], zero, carry);
    //         end
    //         6'b1101??: begin
    //             $display("Time: %0d JAL R%d, Imm: %b ZERO: %b, CARRY: %b", $time, instr[11:9], instr[8:0], zero, carry);
    //         end
    //     endcase
	// 	end
    // end
	
	//* state control
	state_logic statecontrol(clk, reset, instr, state);

	// Instruction Fetch
	instr_memory imem(clk, zero, carry, state, pc, instr);

	// Instruction Decode 
	controller cntrl(clk, state, instr[15:12], instr[1:0], zero, memtoreg, memwrite, branch, pcsrc, alusrc, regdst, regwrite, jal, alucontrol, adc, ndz);
	
	wire [2:0] writereg; //? remember rc or ra decided by regdest
	wire [15:0] srca, srcb;
	wire [15:0] pcnext, pcplus1, signimm;

	// next PC
	flipflop #(16) pc_reg(state, clk, reset, pcnext, pc); // updating pc
	next_pc pc_logic(clk, reset, jal, pcsrc, state, pc, instr, pcplus1, signimm, pcnext);

	// Reg file
	wire [15:0] result_temp;
	mux2 #(3) writemux(instr[11:9], instr[5:3], regdst, writereg); // *decide write reg
	mux2 #(16) resultmux(aluout, read_data, memtoreg, result_temp); // decide data to be written back
	
	mux2 #(16) jalwritemux(result_temp, pcplus1, jal, result);

	// always @ (*)
	// begin
	// 	$display("Regwrite %b, ndz=%b, zero=%b, adc=%b, carry=%b. final=%b", regwrite,ndz, zero, adc, carry, regwriteFinal);
	// end

	wire temp, regwriteFinal;
	mux2 #(1) adccheckmux(regwrite, carry, adc, temp);
	mux2 #(1) ndzcheckmux(temp, zero, ndz, regwriteFinal);
	regfile register(state, clk, reset, regwriteFinal, instr[8:6], instr[11:9], writereg, result, pcnext, srca, write_data);

	// ALU
	mux2 #(16) srcbmux(write_data, signimm, alusrc, srcb); // !decides using alusrc b/w sign_ext(not there) & read data 2
	alu alu1(clk, state, srca, srcb, alucontrol, aluout, zero, carry);

	// MEM
	data_memory dmem(state, clk, memwrite, aluout, write_data, read_data);

endmodule


module state_logic(
		input clk, reset,
		input [15:0] instr,
		output reg [1:0] state
	);
	always @ (posedge clk, posedge reset)
	begin
		if (reset)
			state <= 2'b00;
		else begin
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
	end
endmodule


module next_pc(
		input clk, reset, jal, pcsrc,
		input [1:0] state, input [15:0] pc, instr,
		output [15:0] pcplus1, signimm, pcnext
	);
	wire [15:0] immediate, signimm_jal, pcbranch;
	adder pcadd1(pc, 16'b01, pcplus1); // *PC + 1
	sign_ext se(instr[5:0], signimm); // *sign extend for beq
	sign_ext_jal se_jal(instr[8:0], signimm_jal); // *sign extend for jal
	mux2 #(16) jal_branch_mux(signimm, signimm_jal, jal, immediate);
	adder pcadd(pc, immediate, pcbranch);
	mux2 #(16) pcbrmux(pcplus1, pcbranch,  pcsrc, pcnext);
endmodule


module data_memory(
		input [1:0] state, input clk, we,
		input [15:0] addr, wd,
		output reg [15:0] rd
	);
	reg [15:0] RAM[255:0];
	always @ (posedge clk) 
	begin
		if (state == 2'b10)
		begin
			if (we) begin
				RAM[addr] <= wd;
				$display("Time: %0d, MEM access: writing RAM[%0d]=%0d", $time, addr, wd);
			end
			else begin
				rd <= RAM[addr];
				$display("Time: %0d, MEM Access: read addr=%0d, value read=%0d", $time, addr, RAM[addr]);
			end
		end
	end
endmodule


module instr_memory(
		input clk, zero, carry,
		input [1:0] state,
		input [15:0] addr,
		output reg [15:0] rd
	);
	reg [15:0] RAM[31:0]; // 32 registers of 16 bits
	initial
		begin
			$readmemh("memfile.dat", RAM);
			
		end
	always @ (addr)
		if (state == 2'b00) // IF & ID
		begin
			rd = RAM[addr];
			casex({rd[15:12], rd[1:0]})
            6'b000000: begin
                $display("Time: %0d, PC: %0d => ADD R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, addr, rd[11:9], rd[8:6], rd[5:3], zero, carry);
            end
            6'b000010: begin
                $display("Time: %0d, PC: %0d => ADC R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, addr, rd[11:9], rd[8:6], rd[5:3], zero, carry);
            end
            6'b001000: begin
                $display("Time: %0d, PC: %0d => NDU R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, addr, rd[11:9], rd[8:6], rd[5:3], zero, carry);
            end
            6'b001001: begin
                $display("Time: %0d, PC: %0d => NDZ R%d, R%d, R%d ZERO: %b, CARRY: %b", $time, addr, rd[11:9], rd[8:6], rd[5:3], zero, carry);
            end
            6'b1010??: begin
                $display("Time: %0d, PC: %0d => LW R%d, R%d, Imm: %b  ZERO: %b, CARRY: %b", $time, addr, rd[11:9], rd[8:6], rd[5:0], zero, carry);
            end
            6'b1001??: begin
                $display("Time: %0d, PC: %0d => SW R%d, R%d, Imm: %b  ZERO: %b, CARRY: %b", $time, addr, rd[11:9], rd[8:6], rd[5:0], zero, carry);
            end
            6'b1011??: begin
                $display("Time: %0d, PC: %0d => BEQ R%d, R%d, Imm: %b  ZERO: %b, CARRY: %b", $time, addr, rd[11:9], rd[8:6], rd[5:0], zero, carry);
            end
            6'b1101??: begin
                $display("Time: %0d, PC: %0d => JAL R%d, Imm: %b ZERO: %b, CARRY: %b", $time, addr, rd[11:9], rd[8:0], zero, carry);
            end
        endcase
		end
endmodule


module regfile(
		input [1:0] state,
		input clk, reset, we,
		input [2:0] ra1, ra2, wa,
		input [15:0] wd, pc,
		output reg [15:0] rd1, rd2
	);

	reg [15:0] register_file[7:0];
	always @ (posedge reset)
	begin
		register_file[0] = 0;
		register_file[1] = 3;
		register_file[2] = 4;
		register_file[3] = 9;
		register_file[4] = 24;
		register_file[5] = 0;
		register_file[6] = 65535;
		register_file[7] = 65535;
	end
	// reg pcplusone;
	always @ (posedge clk)
	begin
		if (state == 2'b00) // IF & ID or WB
		begin
			rd1 <= register_file[ra1];
			rd2 <= register_file[ra2];
    		$display("Time: %0d, Register values: R0 = %0d, R1 = %0d, R2 = %0d, R3 = %0d, R4 = %0d, R5 = %0d, R6 = %0d, R7 = %0d", 
                $time, register_file[0], register_file[1], register_file[2], register_file[3], register_file[4], register_file[5], register_file[6], register_file[7]);
			$display("Time: %0d, Registers read rd1: %0d, rd2: %0d", $time, register_file[ra1], register_file[ra2]);
		end
		if (state == 2'b11) // IF & ID or WB
		begin
			if (we && wa != 0) begin
				register_file[wa] <= wd;
				$display("Time: %0d, Writing Back: R[%0d] = %0d", $time, wa, wd);
			end
			register_file[0] <= pc;
			$display("Time: %0d, Updating pc=%0d", $time, pc);
			$display("----------------------------------------------------------");
		end
	end
endmodule


module controller (
		input clk,
		input [1:0] state, input [3:0] op, input [1:0] cz,
		input zero,
		output memtoreg, memwrite, branch,
		output pcsrc, alusrc,
		output regdst, regwrite, jal,
		output [1:0] alucontrol,
		output adc, ndz
	);	
	decoder md (clk, state, op, cz, memtoreg, memwrite, memread, branch, alusrc, regdst, regwrite, jal, adc, ndz);
	aludecoder ad (state, op, alucontrol);
	assign pcsrc = jal | (branch & zero);
endmodule


module decoder (
		input clk,
		input [1:0] state, input [3:0] op, input [1:0] cz,
		output memtoreg, memwrite, memread,
		output branch, alusrc,
		output regdst, regwrite, jal,
		output reg adc, ndz
	);
				
	reg [7:0] controls;
	// regdst decides b/w ra(0) & rc(1) registers to be the destination register
	assign {regwrite, regdst, alusrc, branch, memwrite, memread, memtoreg, jal} = controls;
	always @ (posedge clk) // * can be replaced by op (since it is the only input)
	begin
		if (state == 2'b00) // IF & ID
		begin
			adc <= 0;
			ndz <= 0;
			case(op)
				4'b0000: begin
					controls <= 8'b11000000; // Rtype
					case(cz)
						2'b10: adc <= 1;
					endcase
				end
				4'b0010: begin
					controls <= 8'b11000000; // Rtype
					case(cz)
						2'b01: ndz <= 1;
					endcase
				end
				4'b1010: controls <= 8'b10100110; // LW
				4'b1001: controls <= 8'b00101000; // SW
				4'b1011: controls <= 8'b00010000; // BEQ
				4'b1101: controls <= 8'b10110001; //! JAL (this was updated)
				default: controls  <= 8'bxxxxxxxx; //???
			endcase
		end
	end
endmodule


module aludecoder (
		input [1:0] state, input [3:0] opcode,
		output reg [1:0] alucontrol
	);
	always @ (*)
		if (state == 2'b00) // IF & ID
		  	case (opcode)
				4'b1010: alucontrol <= 2'b00; // add LW
				4'b1001: alucontrol <= 2'b00; // add SW
				4'b1011: alucontrol <= 2'b01; // sub BEQ
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
		input clk,
		input [1:0] state, input [15:0] i_data_A, i_data_B,
		input [1:0] i_alu_control, output reg [15:0] o_result,
		output reg o_zero_flag, o_carry_flag
	);
	always @(posedge clk) begin
		if (state == 2'b01) // ALU
		begin
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
			$display("Time: %0d, ALU Operation: A: %0d, B: %0d, result: %0d, Zero: %b, Carry: %b", $time, i_data_A, i_data_B, o_result, o_zero_flag, o_carry_flag);
		end
	end
endmodule