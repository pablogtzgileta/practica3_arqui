module ForwardingUnit
(
	input MEM_WB_RegWrite,
	input EX_MEM_RegWrite,
	input [4:0] ID_Ex_RegisterRs,
	input [4:0] ID_E_RegisterRt,
	input [4:0] EX_MEM_RegisterRd,
	input [4:0] MEM_WB_RegisterRd,
	output [1:0] A,
	output [1:0] B

);

assign A = 	(EX_MEM_RegWrite == 1'b1 &&  EX_MEM_RegisterRd != 0 && ID_Ex_RegisterRs == EX_MEM_RegisterRd)?										2'b10:
				(MEM_WB_RegWrite == 1'b1 && MEM_WB_RegisterRd != 0 && (EX_MEM_RegisterRd != ID_Ex_RegisterRs | EX_MEM_RegWrite == 0) && ID_Ex_RegisterRs == MEM_WB_RegisterRd)?			2'b11:
																																														2'b00;

assign B = 	(EX_MEM_RegWrite == 1'b1 && EX_MEM_RegisterRd != 0 && ID_E_RegisterRt == EX_MEM_RegisterRd)?	    								2'b10:
				(ID_E_RegisterRt == MEM_WB_RegisterRd && MEM_WB_RegWrite == 1'b1 && (EX_MEM_RegisterRd != ID_E_RegisterRt | EX_MEM_RegWrite == 0) && MEM_WB_RegisterRd != 0)?			2'b11:
																																														2'b00;

endmodule
