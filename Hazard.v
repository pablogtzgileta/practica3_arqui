module Hazard
(
input [9:0] Instruction_ID,
input [4:0] RT_EX,
input MemRead_EX,
output IF_ID_Write,
output PCWrite,
output Bubble
);

wire [4:0] RS;
wire [4:0] RT;
assign RS = Instruction_ID[9:5];
assign RT = Instruction_ID[4:0];

assign IF_ID_Write = (MemRead_EX == 1'b1 && (RT_EX == RT || RT_EX == RS))? 1'b0: 1'b1;

assign PCWrite = IF_ID_Write;

assign Bubble = !PCWrite;


endmodule
