`timescale 1ns / 1ps
module fir 
#(  
parameter pADDR_WIDTH = 12,
parameter pDATA_WIDTH = 32,
parameter Tape_Num    = 11
)
(
output  wire                     awready,
output  wire                     wready,
input   wire                     awvalid,
input   wire [(pADDR_WIDTH-1):0] awaddr,
input   wire                     wvalid,
input   wire [(pDATA_WIDTH-1):0] wdata,
output  wire                     arready,
input   wire                     rready,
input   wire                     arvalid,
input   wire [(pADDR_WIDTH-1):0] araddr,
output  wire                     rvalid,
output  wire [(pDATA_WIDTH-1):0] rdata,    
input   wire                     ss_tvalid, 
input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
input   wire                     ss_tlast, 
output  wire                     ss_tready, 
input   wire                     sm_tready, 
output  wire                     sm_tvalid, 
output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
output  wire                     sm_tlast, 

// bram for tap RAM
output  wire [3:0]               tap_WE,
output  wire                     tap_EN,
output  wire [(pDATA_WIDTH-1):0] tap_Di,
output  wire [(pADDR_WIDTH-1):0] tap_A,
input   wire [(pDATA_WIDTH-1):0] tap_Do,

// bram for data RAM
output  wire [3:0]               data_WE,
output  wire                     data_EN,
output  wire [(pDATA_WIDTH-1):0] data_Di,
output  wire [(pADDR_WIDTH-1):0] data_A,
input   wire [(pDATA_WIDTH-1):0] data_Do,

input   wire                     axis_clk,
input   wire                     axis_rst_n
);

/////////////////////////////////////////////
///FSM parameter
/////////////////////////////////////////////
parameter IDLE 		= 2'd0;
parameter WRITE     = 2'd1;
parameter WAIT_READ = 2'd2;
parameter READ  	= 2'd3;

/////////////////////////////////////////////
///register declaration
/////////////////////////////////////////////

//write control
reg                     awready_reg;    
reg                     wready_reg;
//read control
reg                     arready_reg;
reg                     rvalid_reg;
//tap reg
reg [3:0]               tap_WE_reg;
reg [(pADDR_WIDTH-1):0] tap_A_reg;
reg [(pDATA_WIDTH-1):0] tap_Di_reg;
//state
reg   [1:0]             c_state, n_state;


/////////////////////////////////////////////
///combinational logic
/////////////////////////////////////////////
/*************tap bram *************/
assign tap_EN = 1'b1;
assign tap_WE = tap_WE_reg;
assign tap_A = tap_A_reg;
assign tap_Di = tap_Di_reg;

assign awready = (n_state == WRITE)? 1:0;
assign wready = (n_state == WRITE)? 1:0;

assign rvalid = rvalid_reg;
assign arready = (n_state == READ)? 1:0;
/////////////////////////////////////////////
///FSM
/////////////////////////////////////////////
always @(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		c_state <= IDLE;
	end
	else begin
		c_state <= n_state;
	end
end

/////////////////////////////////////////////
///NEXT STATE BLOCK
/////////////////////////////////////////////
always @(*) begin
    case(c_state)
        IDLE:begin
            if(awvalid)
                n_state = WRITE;
            else  
                n_state = IDLE;
        end
        WRITE:begin
            if(arvalid)
                n_state = READ;
            else
                n_state = WRITE;
        end
        WAIT_READ:begin
			if(arready)
                n_state = READ;
			else 
				n_state = WAIT_READ;
        end
        READ:
                n_state = IDLE;
        default:n_state = IDLE;
    endcase
end


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
    if(~axis_rst_n) tap_WE_reg <= 4'h0;
    else if ((wvalid && wready == 1'b1) && (awaddr != 12'h000)) tap_WE_reg <= 4'hf;
    else tap_WE_reg <= 4'h0;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) rvalid_reg <= 1'b0;
    else rvalid_reg <= arready;
end





endmodule



