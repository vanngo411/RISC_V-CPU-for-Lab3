module RICSV_TOP(
	input clk, rst_n,
    output[7:0] led,
    input usb_rx,
    output usb_tx
	);

	wire [31:0] instruction_outTop, read_data1Top, read_data2Top, ALUresultTop, toALU, Data_outTop, writeBackTop, im_genTop, jmp_addrTop;
	wire [3:0] ALUControl_outTop; ///??????????????
	wire RegWriteTop, MemWriteTop, MemReadTop, ALUSrcTop, MemtoRegTop, ZeroTop, AndGate_outTop, branch_outTop;
	wire [1:0] ALUOpTop;
	wire [31:0] PCtop, NextToPCtop, Mux3_outTop;
	
	
//	wire [7:0] led_outTop;
	wire [7:0] data_to_LED_Top;
	wire [2:0] read_address_LED;
	reg [2:0] read_address_for_LED;
	wire [2:0] counter_outTop, counter_out_slowerTop;
	integer count;
	wire clkx4;
	
	//wire for IF/ID register
	wire [31:0] o_if_id_pcTop, o_if_id_instructionTop;
	//wire for ID/EX register
	wire [31:0] o_id_ex_pcTop;
	wire [1:0] o_id_ex_ALUOpTop;
	wire o_id_ex_ALUSrcTop, o_id_ex_BranchTop, o_id_ex_MemReadTop, o_id_ex_MemWriteTop, o_id_ex_RegWriteTop, o_id_ex_MemToRegTop;
	wire [31:0] o_id_ex_im_genTop, o_id_ex_Read_data1Top, o_id_ex_Read_data2Top; 
	wire [4:0] o_id_ex_Rs1Top, o_id_ex_RdTop, o_id_ex_Rs2Top; 
	wire [31:0] o_id_ex_to_ALUControlTop; //still full instruction, cut off when connect to ALUControl
	//wire for EX/MEM
	wire o_ex_mem_BranchTop, o_ex_mem_MemReadTop, o_ex_mem_MemWriteTop, o_ex_mem_RegWriteTop, o_ex_mem_MemToRegTop, o_ex_mem_ReadData2Top;
	wire [31:0] o_ex_mem_Jmp_addrTop,o_ex_mem_ALU_resultTop; 
	wire o_ex_mem_zeroTop;
	wire [4:0] o_ex_mem_RdTop;
	//wire for MEM/WB
	wire o_mem_wb_RegWriteTop, o_mem_wb_MemToRegTop;
	wire [31:0] o_mem_wb_Read_DataTop, o_mem_wb_ALU_resultTop;
	wire [4:0] o_mem_wb_RdTop;
	//wire for forwarding
	wire [1:0] o_forwardATop;
	wire [1:0] o_forwardBTop;
	//wire for Mux forwarding
	wire [31:0] o_Mux_ForwardA_outTop;
	wire [31:0] o_Mux_ForwardB_outTop;
	// wire for hazard detection
	wire o_hazard_detection_controlTop, o_hazard_detection_PCWriteTop, o_hazard_detection_IFIDWriteTop;
	wire [1:0] o_Mux4_ALUOpTop;
	wire o_Mux4_ALUSrcTop, o_Mux4_branch_outTop, o_Mux4_MemReadTop, o_Mux4_MemWriteTop, o_Mux4_RegWriteTop, o_Mux4_MemtoRegTop;
	
	// New adding for lab2
	// Register IF/ID
	if_id_reg if_id_reg
	(
    .i_clk(clk),
	 .i_reset(reset), 
	 .i_IFIDWrite(o_hazard_detection_IFIDWriteTop),
//	 .i_we(), 
//	 .i_flush(),
//    .i_if_p4(), 
	 .i_if_pc(PCtop), 
	 .i_if_instr(instruction_outTop),
//    .o_id_p4(), 
	 .o_id_pc(o_if_id_pcTop), 
    .o_id_instr(o_if_id_instructionTop)
	 );
	 
	// Register ID/EX
	id_exe_reg id_exe_reg
	(
    .i_clk(clk), 
	 .i_reset(reset),
    // Control signals from ID stage
	 // Control signals for for EX block
	 .i_id_ex_ALUOp(o_Mux4_ALUOpTop), 
	 .i_id_ex_ALUSrc(o_Mux4_ALUSrcTop), 
	 // Control signals for for M block 
	 .i_id_ex_Branch(o_Mux4_branch_outTop),
	 .i_id_ex_MemRead(o_Mux4_MemReadTop), 
	 .i_id_ex_MemWrite(o_Mux4_MemWriteTop),
	 // Control signals for for WB block 
	 .i_id_ex_RegWrite(o_Mux4_RegWriteTop),
	 .i_id_ex_MemToReg(o_Mux4_MemtoRegTop),  
    // Data from ID stage 
	 .i_id_ex_PC(o_if_id_pcTop),
	 .i_id_ex_Read_data1(read_data1Top), 
	 .i_id_ex_Read_data2(read_data2Top),
	 .i_id_ex_Rs1(o_if_id_instructionTop[19:15]),
	 .i_id_ex_Rs2(o_if_id_instructionTop[24:20]),
    .i_id_ex_Rd(o_if_id_instructionTop[11:7]),
	 .i_id_ex_im_gen(im_genTop), 
	 .i_id_ex_to_ALUControl(o_if_id_instructionTop),
    // Control signals out of EX stage
	 // Control signals for for EX block
	 .o_id_ex_ALUOp(o_id_ex_ALUOpTop), 
	 .o_id_ex_ALUSrc(o_id_ex_ALUSrcTop), 
	 // Control signals for for M block 
	 .o_id_ex_Branch(o_id_ex_BranchTop),
	 .o_id_ex_MemRead(o_id_ex_MemReadTop), 
	 .o_id_ex_MemWrite(o_id_ex_MemWriteTop),
	 // Control signals for for WB block 
	 .o_id_ex_RegWrite(o_id_ex_RegWriteTop),
	 .o_id_ex_MemToReg(o_id_ex_MemToRegTop),  
    // Data from ID stage
	 .o_id_ex_PC(o_id_ex_pcTop),
	 .o_id_ex_Read_data1(o_id_ex_Read_data1Top), 
	 .o_id_ex_Read_data2(o_id_ex_Read_data2Top),
	 .o_id_ex_Rs1(o_id_ex_Rs1Top), // wait for forward
	 .o_id_ex_Rs2(o_id_ex_Rs2Top), // wait for forward
    .o_id_ex_Rd(o_id_ex_RdTop),
	 .o_id_ex_im_gen(o_id_ex_im_genTop),
	 .o_id_ex_to_ALUControl(o_id_ex_to_ALUControlTop) //still full instruction
	);
	
	ex_mem_reg ex_mem_reg
	(
    .i_clk(clk), 
	 .i_reset(reset),
    // Control signals from ID stage
	 // Control signals for for M block 
	 .i_ex_mem_Branch(o_id_ex_BranchTop),
	 .i_ex_mem_MemRead(o_id_ex_MemReadTop), 
	 .i_ex_mem_MemWrite(o_id_ex_MemWriteTop),
	 // Control signals for for WB block 
	 .i_ex_mem_RegWrite(o_id_ex_RegWriteTop),
	 .i_ex_mem_MemToReg(o_id_ex_MemToRegTop),  
    // Data from ID stage 		  
	 .i_ex_mem_Jmp_addr(jmp_addrTop), 
	 .i_ex_mem_ALU_result(ALUresultTop),
	 .i_ex_mem_zero(ZeroTop),
	 .i_ex_mem_ReadData2(o_forwardBTop),
	 .i_ex_mem_Rd(o_id_ex_RdTop),
	 
    // Control signals out of EX stage
	 // Control signals for for M block 
	 .o_ex_mem_Branch(o_ex_mem_BranchTop),
	 .o_ex_mem_MemRead(o_ex_mem_MemReadTop), 
	 .o_ex_mem_MemWrite(o_ex_mem_MemWriteTop),
	 // Control signals for for WB block 
	 .o_ex_mem_RegWrite(o_ex_mem_RegWriteTop),
	 .o_ex_mem_MemToReg(o_ex_mem_MemToRegTop),  
    // Data from ID stage 		 
	 .o_ex_mem_Jmp_addr(o_ex_mem_Jmp_addrTop), 
	 .o_ex_mem_ALU_result(o_ex_mem_ALU_resultTop),
	 .o_ex_mem_zero(o_ex_mem_zeroTop),
	 .o_ex_mem_ReadData2(o_ex_mem_ReadData2Top),
	 .o_ex_mem_Rd(o_ex_mem_RdTop) 
	);
	
	mem_wb_reg mem_wb_reg
	(
    .i_clk(clk), 
	 .i_reset(reset),
    // Control signals from MEM stage
	 // Control signals for for WB block 
	 .i_mem_wb_RegWrite(o_ex_mem_RegWriteTop),
	 .i_mem_wb_MemToReg(o_ex_mem_MemToRegTop),  
    // Data from ID stage 		  
	 .i_mem_wb_Read_Data(Data_outTop),
	 .i_mem_wb_ALU_result(o_ex_mem_ALU_resultTop),
	 .i_mem_wb_Rd(o_ex_mem_RdTop),
	 
    // Control signals out of MEM stage
	 // Control signals for for WB block 
	 .o_mem_wb_RegWrite(o_mem_wb_RegWriteTop),
	 .o_mem_wb_MemToReg(o_mem_wb_MemToRegTop),  
    // Data from ID stage 
	 .o_mem_wb_Read_Data(o_mem_wb_Read_DataTop),
	 .o_mem_wb_ALU_result(o_mem_wb_ALU_resultTop),
	 .o_mem_wb_Rd(o_mem_wb_RdTop)
	);
	
	
	
	forwarding forwarding 
	(
	.i_clk(clk), 
	.i_reset(reset),
	.i_id_ex_Rs1(o_id_ex_Rs1Top),
	.i_id_ex_Rs2(o_id_ex_Rs2Top),
	.i_mem_wb_Rd(o_mem_wb_RdTop),
	.i_ex_mem_Rd(o_ex_mem_RdTop),
	.i_mem_wb_RegWrite(o_mem_wb_RegWriteTop),
	.i_ex_mem_RegWrite(o_ex_mem_RegWriteTop),
	.o_forwardA(o_forwardATop),
	.o_forwardB(o_forwardBTop)
	);
	
	
	
	Mux_ForwardA Mux_ForwardA
	(
	.Sel(o_forwardATop),
	.A1(o_id_ex_Read_data1Top), 
	.B1(writeBackTop),
	.C1(o_ex_mem_ALU_resultTop),
	.Mux_ForwardA_out(o_Mux_ForwardA_outTop)
	);
	
	Mux_ForwardB Mux_ForwardB
	(
	.Sel(o_forwardBTop),
	.A1(o_id_ex_Read_data2Top), 
	.B1(writeBackTop),
	.C1(o_ex_mem_ALU_resultTop),
	.Mux_ForwardB_out(o_Mux_ForwardB_outTop)
	);
	
	
	
	// hazard detection
	hazard_detection_unit hazard_detection_unit(
    .ins(o_if_id_instructionTop),
    .rd(o_id_ex_RdTop),
    .memrd(o_id_ex_MemReadTop),
    .control(o_hazard_detection_controlTop),
    .PCWrite(o_hazard_detection_PCWriteTop),
    .IFIDWrite(o_hazard_detection_IFIDWriteTop)
    );
	 
	
	
	//Mux 4 from control to ID/EX register
	Mux4_ForControl Mux4_ForControl
	(
	.Sel(o_hazard_detection_controlTop),
	.i_Mux4_ALUOp(ALUOpTop), 
	.i_Mux4_ALUSrc(ALUSrcTop), 
	.i_Mux4_branch_out(branch_outTop),
	.i_Mux4_MemRead(MemReadTop), 
	.i_Mux4_MemWrite(MemWriteTop),
	.i_Mux4_RegWrite(RegWriteTop),
	.i_Mux4_MemtoReg(MemtoRegTop), 
	.o_Mux4_ALUOp(o_Mux4_ALUOpTop), 
	.o_Mux4_ALUSrc(o_Mux4_ALUSrcTop), 
	.o_Mux4_branch_out(o_Mux4_branch_outTop),
	.o_Mux4_MemRead(o_Mux4_MemReadTop), 
	.o_Mux4_MemWrite(o_Mux4_MemWriteTop),
	.o_Mux4_RegWrite(o_Mux4_RegWriteTop),
	.o_Mux4_MemtoReg(o_Mux4_MemtoRegTop)
	);	
	
	
	// PC +4
	PCplus4 PCplus4 
	(
	.fromPC(PCtop), 
	.nextToPC(NextToPCtop)
	);
	
	//PC
	program_counter program_counter 
	(.clk(clk), 
	.reset(reset), 
	.pc_in(Mux3_outTop), 
	.i_PCWrite(o_hazard_detection_PCWriteTop),
	.i_IFID_PC(o_if_id_pcTop),
	.pc_out(PCtop)
	);
	
	//Memory of instruction
	instruction_memory instruction_memory 
	(
	.read_addr(PCtop), 
	.instruction(instruction_outTop),
	.clk(clk),
	.reset(reset));
	
	// Register File
	register_file register_file 
	(
	.clk(clk), 
	.reset(reset), 
	.Rs1(o_if_id_instructionTop[19:15]), 
	.Rs2(o_if_id_instructionTop[24:20]), 
	.Rd(o_mem_wb_RdTop),      // Next to fix, not take value from instrctrion, should be from MEM/WR registor
	.Write_data(writeBackTop), // skipped MUX
	.RegWrite(o_mem_wb_RegWriteTop), 
	.Read_data1(read_data1Top), 
	.Read_data2(read_data2Top) // review again
	);
	
	//ALU
	alu alu
	(
	.A(o_Mux_ForwardA_outTop), 
	.B(toALU), 
	.zero(ZeroTop), 
	.ALUControl_in(ALUControl_outTop), 
	.ALU_result(ALUresultTop)
	);
	
	//Mux1
	Mux1 Mux1
	(
	.Sel(o_id_ex_ALUSrcTop) , 
	.A1(o_Mux_ForwardB_outTop),  // verify
	.B1(o_id_ex_im_genTop), 
	.Mux1_out(toALU)
	);
	
	// ALU control
	ALUControl ALUControl 
	(
	.ALUOp_in(o_id_ex_ALUOpTop), 
	.func7(o_id_ex_to_ALUControlTop[31:25]), //cut off from full instruction, not fix to much on base code
	.func3(o_id_ex_to_ALUControlTop[14:12]), //cut off from full instruction, not fix to much on base code
	.ALUControl_out(ALUControl_outTop) //ALUControl_outTop is needed verify 32 or 4 bits
	);
	
	
	data_memory data_memory
	(
	.clk(clk), 
	.reset(reset), 
	.MemWrite(o_ex_mem_MemWriteTop), 
	.MemRead(o_ex_mem_MemReadTop), 
	.Address(o_ex_mem_ALU_resultTop), 
	.write_data(o_ex_mem_ReadData2Top), 
	.Read_Data(Data_outTop)
	);
	
	Mux2 Mux2
	(
	.Sel(o_mem_wb_MemToRegTop), 
	.A2(o_mem_wb_ALU_resultTop), 
	.B2(o_mem_wb_Read_DataTop), 
	.Mux2_out(writeBackTop)
	);
	

	control control 
	(
	.reset(reset),
	.OPcode(o_if_id_instructionTop[6:0]), 
	.branch(branch_outTop), 
	.MemRead(MemReadTop), 
	.MemtoReg(MemtoRegTop), 
	.MemWrite(MemWriteTop), 
	.ALUSrc(ALUSrcTop), 
	.RegWrite(RegWriteTop), 
	.ALUOp_out(ALUOpTop)
	);
	
	// Immediate Generator 32 - 32
	immediate_Generator immediate_Generator
	(
	.reset(reset),
	.instruction(o_if_id_instructionTop),
	.im_gen(im_genTop)
	);
	
	// Jump adder
	jmp_adder jmp_adder
	(
    .reset(reset),
    .read_addr(o_id_ex_pcTop),
    .im_gen(o_id_ex_im_genTop),
    .jmp_addr(jmp_addrTop)
	);
	
	Mux3 Mux3
	(
	.Sel(AndGate_outTop), 
	.A3(NextToPCtop), 
	.B3(o_ex_mem_Jmp_addrTop), 
	.Mux3_out(Mux3_outTop)
	);
	
	AndGate AndGate
	(
	.zero(o_ex_mem_zeroTop), 
	.branch(o_ex_mem_BranchTop),
	.pc_sel(AndGate_outTop)
	);	
	
	// LED showing RAM
	RAM_for_LEDShowing RAM_for_LEDShowing
	(
	.clk(clk), 
	.reset(reset), 
	.write_signal(o_mem_wb_RegWriteTop), 
	.read_address(counter_out_slowerTop), 
	.write_address(counter_outTop), 
	.write_data(writeBackTop), 
	.Read_Data(data_to_LED_Top)
	);
	
	Clock_divider Clock_divider
	(
	.clock_in(clk),
	.clock_out(clkx4)
    );
	
	up_counter up_counter
	(
	.clk(clk), 
	.reset(reset), 
	.counter(counter_outTop)
    );
	 
	 up_counter up_counter_slower
	(
	.clk(clkx4), 
	.reset(reset), 
	.counter(counter_out_slowerTop)
    );
    
    assign led = data_to_LED_Top;
 
    assign usb_tx = usb_rx;

endmodule 

//RISC_V 
module instruction_memory (read_addr, instruction, clk, reset);
	input clk, reset;
	input [31:0] read_addr;
	output [31:0] instruction;
	
	reg [31:0] Memory [63:0];
	integer k;
	
	assign instruction = Memory[read_addr];

	always @(posedge clk)
	begin
		if (reset == 1'b1)
		begin
			for (k=0; k<64; k=k+1) 
			begin// here Ou changes k=0 to k=16
				Memory[k] = 32'h00000000;
			end
		end
		else if (reset == 1'b0)
		begin
			// test for hazard detection
//			Memory[4] = 32'b0000000_11001_01010_000_01010_0110011; // add x10, x10, x25	- ALU result: x10=125
//			Memory[8] = 32'b00000001010000001011000100000011; //lw x2, 20(x1)  				- ALU result: x2=20
//			Memory[12] = 32'b00000000010100010111001000110011; //and x4, x2, x5				- ALU result: x4=28
			
			// test for forwarding module
			Memory[0] = 32'b0000000_11001_01010_000_01010_0110011; // add x10, x10, x25	
			Memory[4] = 32'b0000000_11001_01010_000_01010_0110011; // add x10, x10, x25	- ALU result: x10=125
			Memory[8] = 32'b00000000010101010000010100110011; 			//add x10, x10, x5	- ALU result: x10=133
			Memory[12] = 32'b00000000101000111000011110110011;		//add x15, x7, x10		- ALU result: x10=200
			
		end
		
		
	end
endmodule

module Clock_divider(clock_in,clock_out);
	input clock_in; // input clock on FPGA
	output reg clock_out; // output clock after dividing the input clock by divisor
	reg[29:0] counter=30'd0;
	parameter DIVISOR = 30'd400000000; // 30'd400000000 for hardware testing
	
	always @(posedge clock_in)
	begin
		counter <= counter + 6'd1;
		if(counter>=(DIVISOR-1))
			counter <= 6'd0;
			clock_out <= (counter<DIVISOR/2)?1'b1:1'b0;
		end
endmodule 

//RICS_V
module register_file (clk, reset, Rs1, Rs2, Rd, Write_data, RegWrite, Read_data1, Read_data2);
	input clk, reset, RegWrite; //single bit
	input [4:0] Rs1, Rs2, Rd; //5bits - Read register
	input [31:0] Write_data; // 32 bits
	
	output [31:0] Read_data1, Read_data2;
	
	reg [31:0] Registers [31:0]; //32 register each of 32 bit wide
	
	initial 
	begin
		Registers[0] = 32'd0;
		Registers[1] = 32'd0;
		Registers[2] = 6;
		Registers[3] = 34;
		Registers[4] = 5;
		Registers[5] = 8;
		Registers[6] = 2;
		Registers[7] = 67;
		Registers[8] = 56;
		Registers[9] = 45;
		Registers[10] = 50;
		Registers[11] = 41;
		Registers[12] = 24;
		Registers[13] = 23;
		Registers[14] = 24;
		Registers[15] = 35;
		Registers[16] = 46;
		Registers[17] = 57;
		Registers[18] = 68;
		Registers[19] = 29;
		Registers[20] = 30;
		Registers[21] = 41;
		Registers[22] = 52;
		Registers[23] = 53;
		Registers[24] = 44;
		Registers[25] = 75;
		Registers[26] = 56;
		Registers[27] = 57;
		Registers[28] = 48;
		Registers[29] = 39;
		Registers[30] = 80;
		Registers[31] = 91;
	end
	integer k;
	
	always @(posedge clk)
	begin
			if (reset == 1'b1)
			begin
				for (k=0; k<32; k=k+1)
				begin
					Registers[k]=32'h0;
				end
					
			end
			else if (RegWrite == 1'b1)
			begin
				Registers[Rd] = Write_data;
			end
	end
	
	assign Read_data1 = Registers[Rs1];
	assign Read_data2 = Registers[Rs2];
	
endmodule

module program_counter (clk, reset, pc_in, i_PCWrite, i_IFID_PC, pc_out);
	input clk, reset, i_PCWrite;
	input [31:0] pc_in, i_IFID_PC;
	output [31:0] pc_out;
	
	reg [31:0] pc_out;
	always @ (posedge clk or posedge reset )
	
	begin
		if(reset )
			pc_out<=32'h00000000;
		else if (~i_PCWrite)
			pc_out <= i_IFID_PC;
		else
			pc_out<=pc_in;
	end
endmodule

module data_memory(clk, reset, MemWrite, MemRead, Address, write_data, Read_Data);
	input clk, reset, MemWrite, MemRead;
	input [31:0] Address, write_data;
	
	
	output [31:0] Read_Data;
	
	reg [31:0] Dmemory [63:0];
	integer k;
	
	assign Read_Data = (MemRead) ? Dmemory[Address] : 32'h0;
	always @ (posedge clk or posedge reset)
	begin
		if (reset == 1'b1) 
		begin
			for (k=0; k<64; k=k+1)
			begin
				Dmemory[k] = 32'h0;
			end
		end
		else if (MemWrite)
		begin
			Dmemory[Address] = write_data;
		end
	end
endmodule

module control (reset, OPcode, branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, ALUOp_out);
	input reset;
	input [6:0] OPcode;
	output reg branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
	output reg [1:0] ALUOp_out;
	
	always @(*) 
	begin
		if (reset)
		begin
			{branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, ALUOp_out} <= 7'b0000000;
		end
		else
		case (OPcode)
			7'b0110011: // For  R_Type instruction
			begin
				branch <= 0;
				MemRead <=0;
				MemtoReg <=0;
				MemWrite <=0;
				ALUSrc <=0;
				RegWrite <= 1;
				ALUOp_out <= 2'b10;
			end
			7'b0000011: // Load instruction
			begin
				ALUSrc <=1;
				MemtoReg <=0;    //FIxed 1->0 for lw
				RegWrite <= 1;
				MemRead <=1;
				MemWrite <=0;
				branch <= 0;											
				ALUOp_out <= 2'b00;
			end
			7'b0010011: // Use for Immediate Addi, 
			begin
				ALUSrc <=1;
				MemtoReg <=0;
				RegWrite <= 1;
				MemRead <=0;
				MemWrite <=0;
				branch <= 0;											
				ALUOp_out <= 2'b10;
			end
			7'b0100011: // Store instruction
			begin
				ALUSrc <=1;
				MemtoReg <=0;
				RegWrite <= 0;
				MemRead <=0;
				MemWrite <=1;
				branch <= 0;											
				ALUOp_out <= 2'b00;
			end
			7'b1100011: // Branch-equal instruction
			begin
				ALUSrc <=0;
				MemtoReg <=0;
				RegWrite <= 0;
				MemRead <=0;
				MemWrite <=0;
				branch <= 1;											
				ALUOp_out <= 2'b01;
			end
			default: // same R-type
			begin
				ALUSrc <=0;
				MemtoReg <=0;
				RegWrite <= 0;
				MemRead <=0;
				MemWrite <=0;
				branch <= 0;											
				ALUOp_out <= 2'b10;
			end
		endcase
	end
	
endmodule  

module ALUControl (ALUOp_in, func7, func3, ALUControl_out);
	input [1:0] ALUOp_in;
	input [31:25] func7;
	input [14:12] func3;
	
	output reg [3:0] ALUControl_out;
	
	always @ (*) 
	begin
		case ({ALUOp_in, func7, func3})
			12'b00_xxxxxxx_xxx : ALUControl_out = 4'b0010; //12'b00_xxxxxxx_xxx
			12'b00_0000000_000 : ALUControl_out = 4'b0010;
			12'b01_0000000_000 : ALUControl_out = 4'b0110; //12'b01_xxxxxxx_xxx
			12'b10_0000000_000 : ALUControl_out = 4'b0010; 
			12'b10_0100000_000 : ALUControl_out = 4'b0110; // not the same with book
			12'b10_0000000_111 : ALUControl_out = 4'b0000;
			12'b10_0000000_110 : ALUControl_out = 4'b0001;
			default 				: ALUControl_out = 4'bxxxx; 
		endcase
	end

endmodule 

module alu(A, B, zero, ALUControl_in, ALU_result);
	input [31:0] A,B;
	input [3:0] ALUControl_in;
	output reg [31:0] ALU_result;
	output reg zero;
	
	always @ (ALUControl_in or A or B)
	begin
		case (ALUControl_in)
			4'b0000: begin zero<=0; ALU_result <= A & B; end
			4'b0001: begin zero<=0; ALU_result <= A | B; end
			4'b0010: begin zero<=0; ALU_result <= A + B; end
			4'b0110: begin if (A==B) zero<= 1; else zero<=0; ALU_result <= A - B; end
			default: begin zero<=0; ALU_result <= A; end
		endcase
		
	end

endmodule 

module PCplus4 (fromPC, nextToPC);
	input [31:0] fromPC;
	output [31:0] nextToPC;
	
	assign nextToPC = fromPC +32'h00000004;
endmodule 

module Adder(in_1, in_2, sum);
	input [31:0] in_1, in_2;
	output [31:0] sum;
	
	assign sum = in_1 + in_2;
endmodule 

`define RTYPE 51
`define ITYPE 3
`define STYPE 35
`define SBTYPE 99

module immediate_Generator(
  input wire reset,
  input wire [31:0] instruction,
  output reg [31:0] im_gen
);
  reg [19:0] pad;

  always @(reset, instruction) begin
    if(reset == 1) begin
      im_gen<=0;
    end
    if (reset == 0) begin
      if (instruction[6:0] == `ITYPE) begin
        im_gen[12-1:0] <= instruction[31:20];
		end else if (instruction[6:0] == 19) begin
        im_gen[12-1:0] <= instruction[31:20];
      end else if (instruction[6:0] == `STYPE) begin
        im_gen[11:5] <= instruction[31:25];
        im_gen[4:0] <= instruction[11:7];
      end else if (instruction[6:0] == `SBTYPE) begin // shift left 1 bit here
        im_gen[13] <= instruction[31];
        im_gen[11:6] <= instruction[30:25];
        im_gen[12] <= instruction[7];
        im_gen[6-1:2] <= instruction[11:8];
        im_gen[1] <= 0;
		  im_gen[0] <= 0;
		  
//		  im_gen[12] <= instruction[31]; 	//without shift left 1 bit
//        im_gen[10:5] <= instruction[30:25];
//        im_gen[11] <= instruction[7];
//        im_gen[5-1:1] <= instruction[11:8];
//        im_gen[0] <= 0;
      end
      if (instruction[31] == 0) begin
        pad <= 0;
        im_gen[31:12] <= pad;
      end else begin
        pad = 20'hfffff;
        im_gen[31:12] <= pad;
      end
    end
  end
endmodule 

module Mux1(Sel , A1, B1, Mux1_out);
	input Sel;
	input [31:0] A1, B1;
	output [31:0] Mux1_out;
	
	assign Mux1_out = (Sel == 1'b0) ? A1: B1;
endmodule 

module Mux2(Sel , A2, B2, Mux2_out);
	input Sel;
	input [31:0] A2, B2;
	output [31:0] Mux2_out;
	
	assign Mux2_out = (Sel == 1'b0) ? A2: B2;
endmodule 

module jmp_adder
	(
    input reset,
    input [31:0] read_addr,
    input [31:0] im_gen,
    output reg [31:0] jmp_addr
	);

	always @(read_addr, reset, im_gen) 
	begin
		if (reset == 1) 
		begin
			jmp_addr<=0;
		end   
		if (reset == 0) 
		begin
			jmp_addr <= (read_addr + im_gen);
		end
  end
endmodule 

module AndGate(
	input zero, 
	input branch,
	output reg pc_sel);

	always @(branch, zero) 
	begin
		if (zero && branch)
        pc_sel <= 1;
		else
        pc_sel <= 0;
	end

endmodule 

module Mux3(Sel , A3, B3, Mux3_out);
	input Sel;
	input [31:0] A3, B3;
	output [31:0] Mux3_out;
	
	assign Mux3_out = (Sel == 1'b1) ? B3: A3;
endmodule 

module RAM_for_LEDShowing(clk, reset, write_signal, read_address,  write_address, write_data, Read_Data);
	input clk, reset, write_signal;
	input [31:0]  write_data;
	input [2:0] write_address, read_address;
	
	
	output [7:0] Read_Data;
	
	reg [31:0] memory [63:0];
	integer k;
	
	assign Read_Data = memory[read_address];
	
	always @ (posedge clk or posedge reset)
	begin
		if (reset == 1'b1) 
		begin
			for (k=0; k<64; k=k+1)
			begin
				memory[k] = 32'h0;
			end
		end
		else if (write_signal)
		begin
			memory[write_address] = write_data;
		end
	end
endmodule  
	
module up_counter(input clk, reset, output[2:0] counter
    );
	reg [2:0] counter_up;

// up counter
	always @(posedge clk or posedge reset)
	begin
		if(reset)
			counter_up <= 3'd0;
		else
			counter_up <= counter_up + 3'd1;
	end 
	assign counter = counter_up;
endmodule 

module if_id_reg(
    input  wire i_clk, i_reset, i_IFIDWrite,
//	 input  wire i_we, i_flush,
//    input  wire [31:0] i_if_p4, 
	 input  wire [31:0] i_if_pc, i_if_instr,
    output reg  [31:0] o_id_pc, 
    output reg [31:0] o_id_instr
);

    reg [31:0] id_instr;

    always @(posedge i_clk or posedge i_reset )
    begin
        if(i_reset )
        begin
//            o_id_p4    <= 'b0;
            o_id_pc    <= 'b0;
            o_id_instr <= 'b0;
        end

        else if (~i_IFIDWrite)
        begin
//            o_id_p4    <= 'b0;
            o_id_pc    <= i_if_pc; 
            o_id_instr <= 32'h00007013; // andi x0, x0, 0
        end

        else
        begin
            // Not sure we using, don't wire out <if(i_we)>
//                o_id_p4    <= i_if_p4;
                o_id_pc    <= i_if_pc;
                o_id_instr <= i_if_instr;
           
        end
    end

    // assign o_id_instr = (i_flush) ? 32'h00007013 : id_instr; // if flush -> andi x0, x0, 0 | else -> id instr gets if instr

endmodule 

module id_exe_reg(
    input wire i_clk, i_reset,
    // Control signals from ID stage
	 // Control signals for for EX block
	 input wire [1:0] i_id_ex_ALUOp, 
	 input wire i_id_ex_ALUSrc, 
	 // Control signals for for M block 
	 input wire i_id_ex_Branch,
	 input wire i_id_ex_MemRead, 
	 input wire i_id_ex_MemWrite,
	 // Control signals for for WB block 
	 input wire i_id_ex_RegWrite,
	 input wire i_id_ex_MemToReg, 
	 // Program counter
	 input wire [31:0] i_id_ex_PC,
    // Data from ID stage 		 
	 input wire [31:0] i_id_ex_Read_data1, 
	 input wire [31:0] i_id_ex_Read_data2,
	 input wire [4:0] i_id_ex_Rs1,
	 input wire [4:0] i_id_ex_Rs2,
    input wire [4:0] i_id_ex_Rd,
	 input wire [31:0] i_id_ex_im_gen, 
	 input wire [31:0] i_id_ex_to_ALUControl,
    // Control signals out of EX stage
	 // Control signals for for EX block
	 output reg [1:0] o_id_ex_ALUOp, 
	 output reg o_id_ex_ALUSrc, 
	 // Control signals for for M block 
	 output reg o_id_ex_Branch,
	 output reg o_id_ex_MemRead, 
	 output reg o_id_ex_MemWrite,
	 // Control signals for for WB block 
	 output reg o_id_ex_RegWrite,
	 output reg o_id_ex_MemToReg,
	 // Program counter
	 output reg [31:0] o_id_ex_PC,
    // Data from ID stage 		 
	 output reg [31:0] o_id_ex_Read_data1, 
	 output reg [31:0] o_id_ex_Read_data2,
	 output reg [4:0] o_id_ex_Rs1,
	 output reg [4:0] o_id_ex_Rs2,
    output reg [4:0] o_id_ex_Rd,
	 output reg [31:0] o_id_ex_im_gen,
	 output reg [31:0] o_id_ex_to_ALUControl
);

    always @(posedge i_clk or posedge i_reset)
    begin
        if (i_reset)
        begin
				o_id_ex_ALUOp <= 2'b00;
				o_id_ex_ALUSrc <= 0;
				// Control signals for for M block 
				o_id_ex_Branch <= 0;
				o_id_ex_MemRead <= 0;
				o_id_ex_MemWrite <= 0;
				// Control signals for for WB block 
				o_id_ex_RegWrite <= 0;
				o_id_ex_MemToReg <= 0;				
				// Program counter
				o_id_ex_PC <= 0;
				// Data from ID stage
				o_id_ex_Read_data1 <= 0;
				o_id_ex_Read_data2 <= 0;
				o_id_ex_Rs1 <= 0;
				o_id_ex_Rs2 <= 0;
				o_id_ex_Rd <= 0;
				o_id_ex_im_gen <= 0;
				o_id_ex_to_ALUControl <= 0;
        end

        else
        begin
				o_id_ex_ALUOp <= i_id_ex_ALUOp;
				o_id_ex_ALUSrc <= i_id_ex_ALUSrc;
				// Control signals for for M block 
				o_id_ex_Branch <= i_id_ex_Branch;
				o_id_ex_MemRead <= i_id_ex_MemRead;
				o_id_ex_MemWrite <= i_id_ex_MemWrite;
				// Control signals for for WB block 
				o_id_ex_RegWrite <= i_id_ex_RegWrite;
				o_id_ex_MemToReg <= i_id_ex_MemToReg;
				// Program counter
				o_id_ex_PC <= i_id_ex_PC;
				// Data from ID stage 		 
				o_id_ex_Read_data1 <= i_id_ex_Read_data1;
				o_id_ex_Read_data2 <= i_id_ex_Read_data2;
				o_id_ex_Rs1 <= i_id_ex_Rs1;
				o_id_ex_Rs2 <= i_id_ex_Rs2;
				o_id_ex_Rd <= i_id_ex_Rd;
				o_id_ex_im_gen <= i_id_ex_im_gen;
				o_id_ex_to_ALUControl <= o_id_ex_to_ALUControl;
        end
    end

endmodule 

module mem_wb_reg(
    input wire i_clk, i_reset,
    // Control signals from MEM stage
	 // Control signals for for WB block 
	 input wire i_mem_wb_RegWrite,
	 input wire i_mem_wb_MemToReg,  
    // Data from ID stage 		  
	 input wire [31:0] i_mem_wb_Read_Data,
	 input wire [31:0] i_mem_wb_ALU_result,
	 input wire [4:0] i_mem_wb_Rd,
	 
    // Control signals out of MEM stage
	 // Control signals for for WB block 
	 output reg o_mem_wb_RegWrite,
	 output reg o_mem_wb_MemToReg,  
    // Data from ID stage 
	 output reg [31:0] o_mem_wb_Read_Data,
	 output reg [31:0] o_mem_wb_ALU_result,
	 output reg [4:0] o_mem_wb_Rd
	);

    always @(posedge i_clk or posedge i_reset)
    begin
        if (i_reset)
        begin
				// Control signals for for WB block 
				o_mem_wb_RegWrite <= 0;
				o_mem_wb_MemToReg <= 0;
				// Data from EX stage 		 
				o_mem_wb_Read_Data <= 0;
				o_mem_wb_ALU_result <= 0;
				o_mem_wb_Rd <= 0;
        end

        else
        begin
				// Control signals for for WB block 
				o_mem_wb_RegWrite <= i_mem_wb_RegWrite;
				o_mem_wb_MemToReg <= i_mem_wb_MemToReg;
				// Data from EX stage 		 
				o_mem_wb_Read_Data <= i_mem_wb_Read_Data;
				o_mem_wb_ALU_result <= i_mem_wb_ALU_result;
				o_mem_wb_Rd <= i_mem_wb_Rd;
        end
    end

endmodule

module ex_mem_reg(
    input wire i_clk, i_reset,
    // Control signals from ID stage
	 // Control signals for for M block 
	 input wire i_ex_mem_Branch,
	 input wire i_ex_mem_MemRead, 
	 input wire i_ex_mem_MemWrite,
	 // Control signals for for WB block 
	 input wire i_ex_mem_RegWrite,
	 input wire i_ex_mem_MemToReg,  
    // Data from ID stage 		  
	 input wire [31:0] i_ex_mem_Jmp_addr, 
	 input wire [31:0] i_ex_mem_ALU_result,
	 input wire i_ex_mem_zero,
	 input wire [31:0] i_ex_mem_ReadData2,
	 input wire [4:0] i_ex_mem_Rd,
	 
    // Control signals out of EX stage
	 // Control signals for for M block 
	 output reg o_ex_mem_Branch,
	 output reg o_ex_mem_MemRead, 
	 output reg o_ex_mem_MemWrite,
	 // Control signals for for WB block 
	 output reg o_ex_mem_RegWrite,
	 output reg o_ex_mem_MemToReg,  
    // Data from ID stage 		 
	 output reg [31:0] o_ex_mem_Jmp_addr, 
	 output reg [31:0] o_ex_mem_ALU_result,
	 output reg o_ex_mem_zero,
	 output reg [31:0] o_ex_mem_ReadData2,
	 output reg [4:0] o_ex_mem_Rd 
	);

    always @(posedge i_clk or posedge i_reset)
    begin
        if (i_reset)
        begin
				// Control signals for for M block 
				o_ex_mem_Branch <= 0;
				o_ex_mem_MemRead <= 0;
				o_ex_mem_MemWrite <= 0;
				// Control signals for for WB block 
				o_ex_mem_RegWrite <= 0;
				o_ex_mem_MemToReg <= 0;
				// Data from EX stage 		 
				o_ex_mem_Jmp_addr <= 0;
				o_ex_mem_ALU_result <= 0;
				o_ex_mem_zero <= 0;
				o_ex_mem_ReadData2 <= 0;
				o_ex_mem_Rd <= 0;
        end

        else
        begin
				// Control signals for for M block 
				o_ex_mem_Branch <= i_ex_mem_Branch;
				o_ex_mem_MemRead <= i_ex_mem_MemRead;
				o_ex_mem_MemWrite <= i_ex_mem_MemWrite;
				// Control signals for for WB block 
				o_ex_mem_RegWrite <= i_ex_mem_RegWrite;
				o_ex_mem_MemToReg <= i_ex_mem_MemToReg;
				// Data from EX stage 		 
				o_ex_mem_Jmp_addr <= i_ex_mem_Jmp_addr;
				o_ex_mem_ALU_result <= i_ex_mem_ALU_result;
				o_ex_mem_zero <= i_ex_mem_zero;
				o_ex_mem_ReadData2 <= i_ex_mem_ReadData2;
				o_ex_mem_Rd <= i_ex_mem_Rd;
        end
    end

endmodule 

module forwarding 
	(
	input wire i_clk, i_reset,
	input wire [4:0] i_id_ex_Rs1,
	input wire [4:0] i_id_ex_Rs2,
	input wire [4:0] i_mem_wb_Rd,
	input wire [4:0] i_ex_mem_Rd,
	input wire i_mem_wb_RegWrite,
	input wire i_ex_mem_RegWrite,
	
	output reg [1:0] o_forwardA,
	output reg [1:0] o_forwardB
	);
	
	always @(i_ex_mem_RegWrite or i_mem_wb_RegWrite)
	begin
		if (i_reset)
		begin
			o_forwardA = 2'b00;
			o_forwardB = 2'b00;
		end
		else 
		begin
			// EX hazard
			if (i_ex_mem_RegWrite)
			begin
				if (i_ex_mem_Rd != 0)
				begin
					if (i_ex_mem_Rd == i_id_ex_Rs1)
					begin
						o_forwardA = 2'b10;
						o_forwardB = 2'b00;
					end
					else if (i_ex_mem_Rd == i_id_ex_Rs2)
					begin
						o_forwardA = 2'b00;
						o_forwardB = 2'b10;
					end
				end
			end
			else if (i_mem_wb_RegWrite)
			begin
				if (i_mem_wb_Rd != 0)
				begin
					if (i_mem_wb_Rd == i_id_ex_Rs1)
					begin
						o_forwardA = 2'b01;
						o_forwardB = 2'b00;
					end
					else if (i_mem_wb_Rd == i_id_ex_Rs2)
					begin
						o_forwardA = 2'b00;
						o_forwardB = 2'b01;
					end
				end
			end
			else 
			begin
				o_forwardA = 2'b00;
				o_forwardB = 2'b00;
			end
		end
	end
endmodule 

module Mux_ForwardA(Sel , A1, B1, C1, Mux_ForwardA_out);
	input [1:0] Sel;
	input [31:0] A1, B1, C1;
	output [31:0] Mux_ForwardA_out;
	
	assign Mux_ForwardA_out = Sel[1] ? (Sel[0] ? A1 : C1) : (Sel[0] ? B1 : A1);
endmodule 

module Mux_ForwardB(Sel , A1, B1, C1, Mux_ForwardB_out);
	input [1:0] Sel;
	input [31:0] A1, B1, C1;
	output [31:0] Mux_ForwardB_out;
	
	assign Mux_ForwardB_out = Sel[1] ? (Sel[0] ? A1 : C1) : (Sel[0] ? B1 : A1);
endmodule 

module hazard_detection_unit(
    input [31:0] ins,
    input [4:0] rd,
    input memrd,
    output control,
    output PCWrite,
    output IFIDWrite
    );
    wire w1, w2, w3, out;
    rs2check r2(ins[6:0], ins[24:20], rd, w2);
    rs1check r1(ins[6:0], ins[19:15], rd, w1);
    assign w3 = w1 | w2;
    assign out = w3 & memrd;
    assign control = out;
    assign PCWrite = ~out;
    assign IFIDWrite = ~out;
endmodule 

module check1(
    input [6:0] op,
    output rs1
    );
    assign rs1 = ((op[6] & (~op[3])) | (~op[2]));
endmodule

module rs1check(
    input [6:0] op,
    input [4:0] rs1,
    input [4:0] rd,
    output o
    );
    wire w1,w2;
    check1 c1(op, w1);			// check for load instruction
    assign w2 = ~(rs1^rd); // rs1 = rd
    assign o = w1 & w2;
endmodule

module check2(
    input [6:0] op,
    output rs2
    );
    assign rs2 = (~op[2])&(op[5])&((~op[6]) | (~op[4]));
endmodule

module rs2check(
    input [6:0] op,
    input [4:0] rs2,
    input [4:0] rd,
    output o
    );
    wire w1,w2;
    check2 c2(op, w1);
    assign w2 = ~(rs2^rd);
    assign o = w1 & w2;
endmodule 

module Mux4_ForControl
	(
	input Sel,
	// Control signals for for EX block
	input [1:0] i_Mux4_ALUOp, 
	input i_Mux4_ALUSrc, 
	 // Control signals for for M block 
	input i_Mux4_branch_out,
	input i_Mux4_MemRead, 
	input i_Mux4_MemWrite,
	 // Control signals for for WB block 
	input i_Mux4_RegWrite,
	input i_Mux4_MemtoReg, 
	
	output [1:0] o_Mux4_ALUOp, 
	output o_Mux4_ALUSrc, 
	 // Control signals for for M block 
	output o_Mux4_branch_out,
	output o_Mux4_MemRead, 
	output o_Mux4_MemWrite,
	 // Control signals for for WB block 
	output o_Mux4_RegWrite,
	output o_Mux4_MemtoReg
	);		
	
	
	assign o_Mux4_ALUOp = (Sel == 1'b1) ? 2'b0: i_Mux4_ALUOp;
	assign o_Mux4_ALUSrc = (Sel == 1'b1) ? 1'b0: i_Mux4_ALUSrc;
	assign o_Mux4_branch_out = (Sel == 1'b1) ? 1'b0: i_Mux4_branch_out;
	assign o_Mux4_MemRead = (Sel == 1'b1) ? 1'b0: i_Mux4_MemRead;
	assign o_Mux4_MemWrite = (Sel == 1'b1) ? 1'b0: i_Mux4_MemWrite;
	assign o_Mux4_RegWrite = (Sel == 1'b1) ? 1'b0: i_Mux4_RegWrite;
	assign o_Mux4_MemtoReg = (Sel == 1'b1) ? 1'b0: i_Mux4_MemtoReg;
endmodule 





