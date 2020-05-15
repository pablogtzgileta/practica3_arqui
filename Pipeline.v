module Pipeline
#(
	parameter N=128
)
(
	input clk,
	input reset,
	input enable,
	input  [N-1:0] DataInput,
	input Flush,

	output reg [N-1:0] DataOutput
);

always@(negedge reset or negedge clk) begin
	if(reset==0)
		DataOutput <= 0;
	else if(Flush==1)
			DataOutput <= 0;
	else if(enable==1)
				DataOutput<=DataInput;
end

endmodule
//register//
