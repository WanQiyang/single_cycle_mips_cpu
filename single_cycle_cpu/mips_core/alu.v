`ifdef PRJ1_FPGA_IMPL
	// the board does not have enough GPIO, so we implement a 4-bit ALU
    `define DATA_WIDTH 4
`else
    `define DATA_WIDTH 32
`endif

module alu(
	input [`DATA_WIDTH - 1:0] A,
	input [`DATA_WIDTH - 1:0] B,
	input [2:0] ALUop,
	output Overflow,
	output CarryOut,
	output Zero,
	output reg [`DATA_WIDTH - 1:0] Result
);

	// TODO: insert your code

	wire OF, CF;
    wire [`DATA_WIDTH - 1:0] B_OP, Result_AND, Result_OR, Result_PLUS;
    
    always@(*)
    begin
       case(ALUop)
       3'b000: Result<=Result_AND;
       3'b001: Result<=Result_OR;
       3'b010, 3'b110: Result<=Result_PLUS;
       3'b111: Result<=OF;
       default: Result<=`DATA_WIDTH'b0;
       endcase
    end
    
    assign Result_AND=A&B;
    assign Result_OR=A|B;
    assign B_OP=B^{`DATA_WIDTH{ALUop[2]}};
    assign {CF, OF, Result_PLUS}={A[`DATA_WIDTH-1], A}+{B_OP[`DATA_WIDTH-1], B_OP}+ALUop[2];
    assign Overflow=OF^Result_PLUS[`DATA_WIDTH-1];
    assign CarryOut=CF^ALUop[2];
    assign Zero=~|Result_PLUS;

endmodule
/*
    always@(*)
    begin
    case(ALUop)
        3'b000: Result<=A&B;
        3'b001: Result<=A|B;
        3'b010: Result<=A+B;
        3'b110: Result<=A-B;
        3'b111: Result<=($signed(A)<$signed(B));
        default: Result<=`DATA_WIDTH'b0;
    endcase
    end
    
    assign Zero=(A==B);

endmodule
*/