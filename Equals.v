module Equals
(
    input [31:0] ReadData1,
    input [31:0] ReadData2,
    input BranchEQ,
    input BranchNE,
	output ORForBranch
);

wire Zero;
wire ZeroANDBrachEQ;
wire NotZeroANDBrachNE;

assign Zero = (ReadData1 == ReadData2) ? 1'b1 : 1'b0;

ANDGate
Gate_BranchEQANDZero
(
	.A(BranchEQ),
	.B(Zero),
	.C(ZeroANDBrachEQ)
);

ANDGate
Gate_BranchNEANDZero
(
	.A(BranchNE),
	.B(!Zero),
	.C(NotZeroANDBrachNE)
);

assign ORForBranch =  NotZeroANDBrachNE | ZeroANDBrachEQ;

endmodule
