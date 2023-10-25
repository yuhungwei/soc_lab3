module axilite
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    //Read address channel(RA)
    output  wire                     arready,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    input   wire                     arvalid, 
    //Read data channel(RD)
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,
    input   wire                     rready,
    //Write address channel(WA)
    output  wire                     awready,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     awvalid,
    //Write data channel(WD)
    output  wire                     wready,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,

    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

reg                     write_counter;  // let awready follows awvalid's second clock and pulls up to 1, 
                                        // then pulls down to 0, wready follows wvalid's second clock and pulls up to 1, 
                                        // then pulls down to 0
reg [1:0]               read_counter;                                        
reg                     awready_reg;    // write address control : if awready is 1, tap_A = awaddr
reg                     wready_reg;

reg [(pADDR_WIDTH-1):0] tap_A_reg;
reg [(pDATA_WIDTH-1):0] tap_Di_reg;
reg [3:0]               tap_WE_reg;

reg                     arready_reg;
reg                     rvalid_reg;

/*************write_counter*************/
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) write_counter <= 1'b0;
    else if (write_counter == 1'b0 && awvalid == 1'b1) write_counter <= 1'b1;
    else if (write_counter == 1'b1 && awvalid == 1'b1) write_counter <= 1'b0;
end

/*************read_counter*************/
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) read_counter <= 1'b0;
    else if (read_counter == 2'b00 && arvalid == 1'b1) read_counter <= 2'b01;
    else if (read_counter == 2'b01 && arvalid == 1'b1) read_counter <= 2'b10;
    else if (read_counter == 2'b10 && arvalid == 1'b1) read_counter <= 2'b00;
end

/*************Write address channel(WA)*************/
assign awready = (write_counter)? 1:0;

/*************Write data channel(WA)*************/
assign wready = (write_counter)? 1:0;

/*************tap bram *************/
assign tap_EN = 1'b1;
assign tap_A = tap_A_reg;
assign tap_Di = tap_Di_reg;
assign tap_WE = tap_WE_reg;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_A_reg <= 12'h000;
    else if (awvalid && awready == 1) begin
        if(awaddr > 12'h020) tap_A_reg <= awaddr - 12'h020;
        else if(awaddr == 12'h010) tap_A_reg <= awaddr;
        else tap_A_reg <= 12'h000;
    end
    else if (arvalid == 1) begin
        if(araddr > 12'h020) tap_A_reg <= araddr - 12'h020;
        else if(araddr == 12'h010) tap_A_reg <= araddr;
        else tap_A_reg <= 12'h000;
    end
    else tap_A_reg <= 12'h000;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_Di_reg <= 32'h0000_0000;
    else if (wvalid && wready == 1) tap_Di_reg <= wdata;
    else tap_Di_reg <= 32'h0000_0000;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_WE_reg <= 4'b0000;
    else if ((wvalid && wready == 1) && (awaddr != 12'h000)) tap_WE_reg <= 4'b1111;
    else tap_WE_reg <= 4'b0000;
end

/*************read address channel(WA)*************/
// assign arready = arready_reg;

// always @(posedge axis_clk or negedge axis_rst_n) begin
//     if(~axis_rst_n) arready_reg <= 1'b0;
//     else arready_reg <= arvalid;
// end

assign arready = (read_counter == 2'b10)? 1:0; 
/*************read data channel(WA)*************/
assign rvalid = rvalid_reg;
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) rvalid_reg <= 1'b0;
    else rvalid_reg <= arready;
end


endmodule