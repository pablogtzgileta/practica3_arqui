/**********************
* Description
*	This is the top-level of a MIPS processor
* This processor is written Verilog-HDL. Also, it is synthesizable into hardware.
* Parameter MEMORY_DEPTH configures the program memory to allocate the program to
* be execute. If the size of the program changes, thus, MEMORY_DEPTH must change.
* This processor was made for computer organization class at ITESO.
**********************/


module MIPS_Processor
#(
parameter MEMORY_DEPTH = 128
)

(
// Inputs
input clk,
input reset,
input [7:0] PortIn,
// Output
output [31:0] ALUResultOut,
output [31:0] PortOut
);
//******************************************************************/
//******************************************************************/


//******************************************************************/
//******************************************************************/
// Data types to connect modules
wire ALUSrc_wire;
wire BranchNE_wire;
wire BranchEQ_wire;
wire Bubble_wire;
wire Jump_wire;
wire JR_wire;
wire MemRead_wire;
wire MemWrite_wire;
wire MemtoReg_wire;
wire RegDst_wire;
wire RegWrite_wire;
wire Zero_wire;
wire ALUSrc_EX_wire;
wire RegWrite_EX_wire;
wire Jump_EX_wire;
wire MemRead_EX_wire;
wire MemtoReg_EX_wire;
wire MemWrite_EX_wire;
wire RegDst_EX_wire;
wire PCEnable_Wire;
wire IFFlush_wire;
wire IF_ID_Write_wire;
wire RegWrite_MEM_wire;
wire Jump_MEM_wire;
wire MemRead_MEM_wire;
wire MemtoReg_MEM_wire;
wire MemWrite_MEM_wire;
wire MemtoReg_WB_wire;
wire RegWrite_WB_wire;
wire Jump_WB_wire;

wire [3:0] ALUOp_wire;
wire [3:0] ALUOperation_wire;
wire [3:0] ALUOp_EX_wire;
wire [4:0] WriteRegister_EX_wire;
wire [4:0] WriteRegister_wire;
wire [4:0] RegisterEX_RT_wire;
wire [4:0] Register_EX_RD_wire;
wire [4:0] RS_EX_wire;
wire [4:0] shamt_EX;
wire [4:0] WriteRegister_MEM_wire;
wire [4:0] WriteRegister_WB_wire;

wire [10:0] Control_wire;
wire [9:0] Control_ID_wire;

wire [31:0] PC_4_EX_wire;
wire [31:0] ReadData1_EX_wire;
wire [31:0] ReadData2_EX_wire;
wire [31:0] InmmediateExtend_EX_wire;
wire [31:0] ALUResult_wire;
wire [31:0] PC_Inmmediate_wire;
wire [31:0] MUX_PC_Inmmediate_wire;
wire [31:0] Instruction_wire;
wire [31:0] Instruction_ID_wire;
wire [31:0] InmmediateExtend_wire;
wire [31:0] MUX_Jump_wire;
wire [31:0] Mux_WriteData_wire;
wire [31:0] Jump_Address_wire;
wire [31:0] MUX_PC_wire;
wire [31:0] MemOut_wire;
wire [31:0] MUX_MemToRegister_wire;
wire [31:0] MemoryAddressx4_wire;
wire [31:0] MemoryAddress_wire;
wire [31:0] PC_wire;
wire [31:0] PC_4_wire;
wire [31:0] PC_4_ID_wire;
wire [31:0] ReadData1_wire;
wire [31:0] ReadData2_wire;
wire [31:0] ReadData2OrInmmediate_wire;
wire [31:0] MUXFwdA1_Output_wire;
wire [31:0] MUXFwdA2_Output_wire;
wire [31:0] MUXFwdB1_Output_wire;
wire [31:0] MUXFwdB2_Output_wire;
wire [31:0] ReadData1_MEM_wire;
wire [31:0] ALUResult_MEM_wire;
wire [31:0] ReadData2_MEM_wire;
wire [31:0] MemOut_WB_wire;
wire [31:0] ALUResult_WB_wire;
wire [31:0] PC_4_MEM_wire;
wire [31:0] PC_4_WB_wire;

wire [63:0] ID_wire;
wire [135:0] WB_wire;
wire [204:0] MEM_wire;
wire [193:0] EX_wire;

wire [1:0] A;
wire [1:0] B;


integer ALUStatus;

assign  PortOut = PC_wire;

//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/

Control
ControlUnit
(
	.OP(Instruction_ID_wire[31:26]),
	.ALUFunction(Instruction_ID_wire[5:0]),
	.RegDst(RegDst_wire),
	.BranchNE(BranchNE_wire),
	.BranchEQ(BranchEQ_wire),
	.ALUOp(ALUOp_wire),
	.ALUSrc(ALUSrc_wire),
	.RegWrite(RegWrite_wire),
	.Jump(Jump_wire),
	.MemRead(MemRead_wire),
	.MemtoReg(MemtoReg_wire),
	.MemWrite(MemWrite_wire),
	.JR(JR_wire),
	.IFFlush(IFFlush_wire)
);

PC_Register
#(
 .N(32)
)
program_counter
(
	.clk(clk),
	.reset(reset),
	.enable(PCEnable_Wire),
	.NewPC(MUX_PC_wire),
	.PCValue(PC_wire)
);

ProgramMemory
#(
	.MEMORY_DEPTH(MEMORY_DEPTH)
)
ROMProgramMemory
(
	.Address(PC_wire),
	.Instruction(Instruction_wire)
);

//******************************************************************/
//***************************** MUX ********************************/
//******************************************************************/


Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForAddress
(
	.Selector(Zero_wire),
	.MUX_Data0(PC_4_wire),
	.MUX_Data1(PC_Inmmediate_wire),

	.MUX_Output(MUX_PC_Inmmediate_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForJump
(
	.Selector(Jump_wire),
	.MUX_Data0(MUX_PC_Inmmediate_wire),
	.MUX_Data1(Jump_Address_wire),

	.MUX_Output(MUX_Jump_wire)
);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForJR
(
	.Selector(JR_wire),
	.MUX_Data0(MUX_Jump_wire),
	.MUX_Data1(ReadData1_wire),

	.MUX_Output(MUX_PC_wire)
);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForReadDataAndImmediate // 3
(
	.Selector(ALUSrc_EX_wire),
	.MUX_Data0(MUXFwdB2_Output_wire),
	.MUX_Data1(InmmediateExtend_EX_wire),

	.MUX_Output(ReadData2OrInmmediate_wire)

);

Multiplexer2to1
#(
	.NBits(5)
)
MUX_ForRTypeAndIType // 1
(
	.Selector(RegDst_EX_wire),
	.MUX_Data0(RegisterEX_RT_wire),
	.MUX_Data1(Register_EX_RD_wire),

	.MUX_Output(WriteRegister_EX_wire)

);

Multiplexer2to1
#(
.NBits(32)
)
MUX_ForFwdA1 // 9-1
(
.Selector(A[0]),
.MUX_Data0(ALUResult_MEM_wire),
.MUX_Data1(Mux_WriteData_wire),
.MUX_Output(MUXFwdA1_Output_wire)
);

Multiplexer2to1
#(
.NBits(32)
)
MUX_ForFwdA2 // 9-2
(
.Selector(A[1]),
.MUX_Data0(ReadData1_EX_wire),
.MUX_Data1(MUXFwdA1_Output_wire),
.MUX_Output(MUXFwdA2_Output_wire)
);

Multiplexer2to1
#(
.NBits(32)
)
MUX_ForFwdB1 // 10-1
(
.Selector(B[0]),
.MUX_Data0(ALUResult_MEM_wire),
.MUX_Data1(Mux_WriteData_wire),
.MUX_Output(MUXFwdB1_Output_wire)
);

Multiplexer2to1
#(
.NBits(32)
)
MUX_ForFwdB2 // 10-2
(
.Selector(B[1]),
.MUX_Data0(ReadData2_EX_wire),
.MUX_Data1(MUXFwdB1_Output_wire),
.MUX_Output(MUXFwdB2_Output_wire)
);

assign Control_wire = {
RegDst_wire,
ALUOp_wire,
ALUSrc_wire,
RegWrite_wire,
MemRead_wire,
MemtoReg_wire,
MemWrite_wire
};

Multiplexer2to1
#(
.NBits(10)
)
MUX_ForControls
(
.Selector(Bubble_wire | Zero_wire),
.MUX_Data0(Control_wire),
.MUX_Data1(10'b0),
.MUX_Output(Control_ID_wire)
);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForMemtoRegister // 7
(
	.Selector(MemtoReg_WB_wire),
	.MUX_Data0(ALUResult_WB_wire),
	.MUX_Data1(MemOut_WB_wire),

	.MUX_Output(MUX_MemToRegister_wire)

);

Multiplexer2to1
#(
	.NBits(5)
)
MUX_ForWriteRegister // 2
(
	.Selector(Jump_WB_wire),
	.MUX_Data0(WriteRegister_WB_wire),
	.MUX_Data1(5'b11111),

	.MUX_Output(WriteRegister_wire)
);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForWriteData // 8
(
	.Selector(Jump_WB_wire),
	.MUX_Data0(MUX_MemToRegister_wire),
	.MUX_Data1(PC_4_WB_wire),

	.MUX_Output(Mux_WriteData_wire)

);


//******************************************************************/
//******************************************************************/
//******************************************************************/

RegisterFile
Register_File
(
	.clk(clk),
	.reset(reset),
	.RegWrite(RegWrite_WB_wire),
	.WriteRegister(WriteRegister_wire),
	.ReadRegister1(Instruction_ID_wire[25:21]),
	.ReadRegister2(Instruction_ID_wire[20:16]),
	.WriteData(Mux_WriteData_wire),
	.ReadData1(ReadData1_wire),
	.ReadData2(ReadData2_wire)

);

Equals
CompareForBranch
(
	.ReadData1(ReadData1_wire),
	.ReadData2(ReadData2_wire),
	.BranchEQ(BranchEQ_wire),
	.BranchNE(BranchNE_wire),
	.ORForBranch(Zero_wire)
);

SignExtend
SignExtend
(
	.DataInput(Instruction_ID_wire[15:0]),
   .SignExtendOutput(InmmediateExtend_wire)
);

Adder32bits
PC_Plus_4
(
	.Data0(PC_wire),
	.Data1(4),

	.Result(PC_4_wire)
);

Adder32bits
PC_Plus_Immediate
(
	.Data0(PC_4_ID_wire),
	.Data1({InmmediateExtend_wire, 2'b00}),

	.Result(PC_Inmmediate_wire)
);

Adder32bits
PC_Address_Jump
(
	.Data0(32'hFFC00000),
	.Data1({PC_4_ID_wire[31:28], Instruction_ID_wire[25:0], 2'b00}),

	.Result(Jump_Address_wire)
);

Adder32bits
SubstractToMemoryAddress
(
	.Data0(ALUResult_MEM_wire),
	.Data1(32'hEFFF0000),

	.Result(MemoryAddressx4_wire)
);

assign MemoryAddress_wire = {1'b0, 1'b0, MemoryAddressx4_wire[31:2]};

//******************************************************************/
//***************************** ALU ********************************/
//******************************************************************/


ALUControl
ArithmeticLogicUnitControl
(
	.ALUOp(ALUOp_EX_wire),
	.ALUFunction(InmmediateExtend_EX_wire[5:0]),

	.ALUOperation(ALUOperation_wire)
);

ALU
Arithmetic_Logic_Unit
(
	.ALUOperation(ALUOperation_wire),
	.A(MUXFwdA2_Output_wire),
	.B(ReadData2OrInmmediate_wire),
	.shamt(shamt_EX),
	.ALUResult(ALUResult_wire)
);

assign ALUResultOut = ALUResult_wire;

//******************************************************************/
//***************************** RAM ********************************/
//******************************************************************/

DataMemory
#(
	.DATA_WIDTH(32),
	.MEMORY_DEPTH(1024)
)
RAM_Memory
(
	.WriteData(ReadData2_MEM_wire),
	.Address(MemoryAddress_wire),
	.MemWrite(MemWrite_MEM_wire),
	.MemRead(MemRead_MEM_wire),
	.clk(clk),
	.ReadData(MemOut_wire)
);

//******************************************************************/
//**************************** Hazard ******************************/
//******************************************************************/

Hazard
HazardUnit
(
	.Instruction_ID(Instruction_ID_wire[25:16]),
	.RT_EX(RegisterEX_RT_wire),
	.MemRead_EX(MemRead_EX_wire),

	.IF_ID_Write(IF_ID_Write_wire),
	.PCWrite(PCEnable_Wire),
	.Bubble(Bubble_wire)
);

//******************************************************************/
//*********************** Forwarding Unit **************************/
//******************************************************************/

ForwardingUnit
ForwandingUnit
(
	.MEM_WB_RegWrite(RegWrite_WB_wire),
	.EX_MEM_RegWrite(RegWrite_MEM_wire),
	.ID_Ex_RegisterRs(RS_EX_wire),
	.ID_E_RegisterRt(RegisterEX_RT_wire),
	.EX_MEM_RegisterRd(WriteRegister_MEM_wire),
	.MEM_WB_RegisterRd(WriteRegister_wire),
	.A(A),
	.B(B)
);

//******************************************************************/
//************************** Pipelines *****************************/
//******************************************************************/

Pipeline
#(
.N(64)
)
IF_ID_Pipeline
(
.clk(clk),
.reset(reset),
.enable(IF_ID_Write_wire),
.Flush(IFFlush_wire | Zero_wire),
.DataInput({
    PC_4_wire,
    Instruction_wire
}),
.DataOutput(ID_wire)

);

assign PC_4_ID_wire = ID_wire [63:32];
assign Instruction_ID_wire = ID_wire [31:0];


Pipeline
#(
.N(160)
)
ID_EX_Pipeline
(
.clk(clk),
.reset(reset),
.enable(1'b1),
.DataInput({
    Jump_wire,
    Instruction_ID_wire[25:21],
    PC_4_ID_wire,
    Control_ID_wire,
    ReadData1_wire,
    ReadData2_wire,
    InmmediateExtend_wire,
    Instruction_ID_wire[20:16],
    Instruction_ID_wire[15:11],
    Instruction_ID_wire[10:6]
}),
.DataOutput(EX_wire),
.Flush(1'b0)
);

assign shamt_EX = EX_wire[4:0];
assign Register_EX_RD_wire = EX_wire[9:5];
assign RegisterEX_RT_wire = EX_wire[14:10];
assign InmmediateExtend_EX_wire = EX_wire[46:15];
assign ReadData2_EX_wire = EX_wire[78:47];
assign ReadData1_EX_wire = EX_wire[110:79];
assign MemWrite_EX_wire = EX_wire[111];
assign MemtoReg_EX_wire = EX_wire[112];
assign MemRead_EX_wire = EX_wire[113];
assign RegWrite_EX_wire = EX_wire[114];
assign ALUSrc_EX_wire = EX_wire[115];
assign ALUOp_EX_wire = EX_wire[119:116];
assign RegDst_EX_wire = EX_wire[120];
assign PC_4_EX_wire = EX_wire[152:121];
assign RS_EX_wire = EX_wire[157:153];
assign Jump_EX_wire = EX_wire[158];


Pipeline
#(
.N(138)
)
EX_MEM_Pipeline
(
.clk(clk),
.reset(reset),
.enable(1'b1),
.DataInput({
    PC_4_EX_wire,
    RegWrite_EX_wire,
    Jump_EX_wire,
    MemRead_EX_wire,
    MemtoReg_EX_wire,
    MemWrite_EX_wire,
    ALUResult_wire,
    ReadData2_EX_wire,
    WriteRegister_EX_wire
}),
.DataOutput(MEM_wire),
.Flush(1'b0)
);

assign WriteRegister_MEM_wire = MEM_wire[4:0];
assign ReadData2_MEM_wire = MEM_wire[36:5];
assign ALUResult_MEM_wire = MEM_wire[68:37];
assign MemWrite_MEM_wire = MEM_wire[69];
assign MemtoReg_MEM_wire = MEM_wire[70];
assign MemRead_MEM_wire = MEM_wire[71];
assign Jump_MEM_wire = MEM_wire[72];
assign RegWrite_MEM_wire = MEM_wire[73];
assign PC_4_MEM_wire = MEM_wire[105:74];

Pipeline
#(
.N(104)
)
MEM_WB_Pipeline
(
.clk(clk),
.reset(reset),
.enable(1'b1),
.DataInput({
    PC_4_MEM_wire,
    MemtoReg_MEM_wire,
    RegWrite_MEM_wire,
    Jump_MEM_wire,
    MemOut_wire,
    ALUResult_MEM_wire,
    WriteRegister_MEM_wire
}),
.DataOutput(WB_wire),
.Flush(1'b0)
);

assign WriteRegister_WB_wire = WB_wire[4:0];
assign ALUResult_WB_wire = WB_wire[36:5];
assign MemOut_WB_wire = WB_wire[68:37];
assign Jump_WB_wire = WB_wire[69];
assign RegWrite_WB_wire = WB_wire[70];
assign MemtoReg_WB_wire = WB_wire[71];
assign PC_4_WB_wire = WB_wire[103:72];

endmodule
