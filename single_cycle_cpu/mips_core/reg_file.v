`ifdef PRJ1_FPGA_IMPL
	// the board does not have enough GPIO, so we implement 4 4-bit registers
    `define DATA_WIDTH 4
	`define ADDR_WIDTH 2
`else
    `define DATA_WIDTH 32
	`define ADDR_WIDTH 5
`endif

module reg_file(
	input clk,
	input rst,
	input [`ADDR_WIDTH - 1:0] waddr,
	input [`ADDR_WIDTH - 1:0] raddr1,
	input [`ADDR_WIDTH - 1:0] raddr2,
	input wen,
	input [`DATA_WIDTH - 1:0] wdata,
	output [`DATA_WIDTH - 1:0] rdata1,
	output [`DATA_WIDTH - 1:0] rdata2
);

	// TODO: insert your code
	
    //reg [`DATA_WIDTH - 1:0] mem_1 [0:(1<<`ADDR_WIDTH) - 1];
    //reg [`DATA_WIDTH - 1:0] mem_2 [0:(1<<`ADDR_WIDTH) - 1];
    reg [`DATA_WIDTH - 1:0] mem [0:(1<<`ADDR_WIDTH) - 1];
    //reg [`ADDR_WIDTH:0] i;
    always@(posedge clk)
    begin
        /*
        if(rst)
            for(i=0;i<(1<<`ADDR_WIDTH);i=i+1)
                begin mem_1[i]<=`DATA_WIDTH'b0; mem_2[i]<=`DATA_WIDTH'b0; end
        else */if(wen)
            //begin mem_1[waddr]<=wdata; mem_2[waddr]<=wdata; end
            mem[waddr]<=wdata;
        /* else
            begin mem_1[waddr]<=mem_1[waddr]; mem_2[waddr]<=mem_2[waddr]; end */
    end
    
    //assign rdata1=(|raddr1)?mem_1[raddr1]:32'd0;
    //assign rdata2=(|raddr2)?mem_2[raddr2]:32'd0;
    assign rdata1={`DATA_WIDTH{|raddr1}}&mem[raddr1];
    assign rdata2={`DATA_WIDTH{|raddr2}}&mem[raddr2];
    
    /*
    generate
    genvar i;
    for(i=0;i<32;i=i+1)
    begin: mem_gen
    FDRE FDRE_inst_1 (
    .Q(rdata1[i]), // 1-bit Data output
    .C(clk), // 1-bit Clock input
    .CE(1'b1), // 1-bit Clock enable input
    .R(rst), // 1-bit Synchronous reset input
    .D(wdata[i]) // 1-bit Data input
    );
    FDRE FDRE_inst_2 (
    .Q(rdata2[i]), // 1-bit Data output
    .C(clk), // 1-bit Clock input
    .CE(1'b1), // 1-bit Clock enable input
    .R(rst), // 1-bit Synchronous reset input
    .D(wdata[i]) // 1-bit Data input
    );
    end
    endgenerate
    */
endmodule
