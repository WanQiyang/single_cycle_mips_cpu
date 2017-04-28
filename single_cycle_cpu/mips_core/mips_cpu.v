module mips_cpu(
	input  rst,
	input  clk,

	output reg [31:0] PC,
	input  [31:0] Instruction,

	output [31:0] Address,
	output MemWrite,
	output [31:0] Write_data,

	input  [31:0] Read_data,
	output MemRead
);

	//TODO: Insert your design of single cycle MIPS CPU here
    
    wire [31:0] alu_in_A, alu_in_B;
    wire [2:0] alu_in_ALUop;
    wire alu_out_Overflow, alu_out_CarryOut, alu_out_Zero;
    wire [31:0] alu_out_Result;
    
    wire [4:0] rf_in_waddr, rf_in_raddr1, rf_in_raddr2;
    wire rf_in_wen;
    wire [31:0] rf_in_wdata;
    wire [31:0] rf_out_rdata1, rf_out_rdata2;
    
    wire [5:0] cu_in_Opcode;
    wire cu_out_RegDst, cu_out_ALUSrc, cu_out_MemtoReg, cu_out_RegWrite, cu_out_MemRead, cu_out_MemWrite, cu_out_Branch, cu_out_ALUop1, cu_out_ALUop0;
    
    wire [5:0] ac_in_Funct;
    wire ac_in_ALUop1, ac_in_ALUop0;
    wire [2:0] ac_out_ALUop;
    
    wire [5:0] bc_in_Opcode;
    wire [31:0] bc_in_ALUResult;
    wire bc_in_Overflow, bc_in_CarryOut, bc_in_Zero;
    wire bc_out_Result;
    
    wire [31:0] PC_next;
    wire [31:0] Sign_extend;
    
    always @(posedge clk)
        if(rst)
            PC<=0;
        else
            PC<=PC_next;
    
    alu alu_i(.A(alu_in_A), .B(alu_in_B), .ALUop(alu_in_ALUop), .Overflow(alu_out_Overflow), .CarryOut(alu_out_CarryOut), .Zero(alu_out_Zero), .Result(alu_out_Result));
    reg_file rf_i(.rst(rst), .clk(clk), .waddr(rf_in_waddr), .raddr1(rf_in_raddr1), .raddr2(rf_in_raddr2), .wen(rf_in_wen), .wdata(rf_in_wdata), .rdata1(rf_out_rdata1), .rdata2(rf_out_rdata2));
    control_unit cu(.Opcode(cu_in_Opcode), .RegDst(cu_out_RegDst), .ALUSrc(cu_out_ALUSrc), .MemtoReg(cu_out_MemtoReg), .RegWrite(cu_out_RegWrite), .MemRead(cu_out_MemRead), .MemWrite(cu_out_MemWrite), .Branch(cu_out_Branch), .ALUop1(cu_out_ALUop1), .ALUop0(cu_out_ALUop0));
    alu_control ac(.Funct(ac_in_Funct), .ALUop1(ac_in_ALUop1), .ALUop0(ac_in_ALUop0), .ALUop(ac_out_ALUop));
    branch_control bc(.Opcode(bc_in_Opcode), .ALUResult(bc_in_ALUResult), .Overflow(bc_in_Overflow), .CarryOut(bc_in_CarryOut), .Zero(bc_in_Zero), .BranchResult(bc_out_Result));
    
    assign cu_in_Opcode=Instruction[31:26];
    assign rf_in_raddr1=Instruction[25:21];
    assign rf_in_raddr2=Instruction[20:16];
    assign rf_in_waddr=cu_out_RegDst?Instruction[15:11]:Instruction[20:16];
    assign rf_in_wdata=cu_out_MemtoReg?Read_data:alu_out_Result;
    assign rf_in_wen=cu_out_RegWrite;
    assign alu_in_A=rf_out_rdata1;
    assign alu_in_B=cu_out_ALUSrc?Sign_extend:rf_out_rdata2;
    assign alu_in_ALUop=ac_out_ALUop;
    assign ac_in_Funct=Instruction[5:0];
    assign ac_in_ALUop1=cu_out_ALUop1;
    assign ac_in_ALUop0=cu_out_ALUop0;
    assign bc_in_Opcode=Instruction[31:26];
    assign bc_in_ALUResult=alu_out_Result;
    assign bc_in_Overflow=alu_out_Overflow;
    assign bc_in_CarryOut=alu_out_CarryOut;
    assign bc_in_Zero=alu_out_Zero;
    assign Sign_extend={{16{Instruction[15]}}, Instruction[15:0]};
    assign PC_next=(cu_out_Branch&&bc_out_Result)?PC+32'd4+(Sign_extend<<2):PC+32'd4; // general branch
    //assign PC_next=(cu_out_Branch&&(!alu_out_Zero))?PC_next_2:PC_next_1; // bne
    assign Address=alu_out_Result;
    assign MemWrite=cu_out_MemWrite;
    assign Write_data=rf_out_rdata2;
    assign MemRead=cu_out_MemRead;
    
endmodule

module control_unit(
    input [5:0] Opcode,
    output RegDst,
    output ALUSrc,
    output MemtoReg,
    output RegWrite,
    output MemRead,
    output MemWrite,
    output Branch,
    output ALUop1,
    output ALUop0
);

    reg [8:0] CtrlResult;
    
    always@(*)
    begin
        case(Opcode)
        6'b001001: CtrlResult<=9'b010100000; // ADDIU
        6'b000000: CtrlResult<=9'b100100010; // R-format
        6'b100011: CtrlResult<=9'b011110000; // lw
        6'b101011: CtrlResult<=9'b010001000; // sw
        6'b000100, 6'b000101: CtrlResult<=9'b000000101; // beq, bne
        default: CtrlResult<=9'b0;
        endcase
    end
    
    assign {RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUop1, ALUop0}=CtrlResult;
    
endmodule

module alu_control(
    input [5:0] Funct,
    input ALUop1,
    input ALUop0,
    output [2:0] ALUop
);

    assign ALUop[2]=ALUop0|(ALUop1&Funct[1]);
    assign ALUop[1]=(~ALUop1)|(~Funct[2]);
    assign ALUop[0]=ALUop1&(Funct[3]|Funct[0]);
    /*
    LUT3 #(.INIT(8'hEA)) LUT3_inst_2 (.O(ALUop[2]), .I0(ALUop0), .I1(ALUop1), .I2(Funct[1]));
    LUT2 #(.INIT(4'h7)) LUT2_inst_1 (.O(ALUop[1]), .I0(ALUop1), .I1(Funct[2]));
    LUT3 #(.INIT(8'hA8)) LUT3_inst_0 (.O(ALUop[0]), .I0(ALUop1), .I1(Funct[0]), .I2(Funct[3]));
    */
endmodule

module branch_control(
    input [5:0] Opcode,
    input [31:0] ALUResult,
    input Overflow,
    input CarryOut,
    input Zero,
    output reg BranchResult
);

    always@(*)
    begin
        case(Opcode)
        6'b000100: BranchResult<=Zero; // beq
        6'b000101: BranchResult<=~Zero; // bne
        default: BranchResult<=1'b0;
        endcase
    end

endmodule
