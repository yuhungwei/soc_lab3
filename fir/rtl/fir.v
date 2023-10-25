`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    //read
    input   wire                     arvalid, 
    input   wire                     rready,
    output  wire                     arready,
    output  wire                     rvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire [(pDATA_WIDTH-1):0] rdata,

    //write
    input   wire                     awvalid,
    input   wire                     wvalid,
    output  wire                     awready,
    output  wire                     wready,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire [(pDATA_WIDTH-1):0] wdata,


    //stream
    output  wire                     ss_tready,
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    input   wire                     sm_tready, 

    
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
parameter                   IDLE = 2'b00;
parameter                   WAIT_DATA = 2'b01;
parameter                   CALC = 2'b10;
parameter                   RESET = 2'b11;
reg         [1:0]           n_state;
reg         [1:0]           c_state;
/////////////////////////////////////////////
///AXILITE  
/////////////////////////////////////////////
//tap
reg [3:0]                   tap_addr_cnt;                                         
reg [(pADDR_WIDTH-1):0]     tap_A_reg;
reg [(pDATA_WIDTH-1):0]     tap_Di_reg;
reg [3:0]                   tap_WE_reg;

reg                         awready_reg;
reg                         wready_reg;

/////////////////////////////////////////////
///AXISTREAM
/////////////////////////////////////////////
//data
reg [(pDATA_WIDTH-1):0]     data_Di_reg;
reg [(pADDR_WIDTH-1):0]     data_A_reg;
reg [3:0]                   data_WE_reg;

/////////////////////////////////////////////
///CALC
/////////////////////////////////////////////
//for read to wait one cycle
reg [1:0]                   read_stage;
reg [(pDATA_WIDTH-1):0]     configuration;
reg [3:0]                   calc_cnt;
reg [3:0]                   pattern_cycle;
reg [9:0]                   pattern_number;
reg [(pDATA_WIDTH-1):0]     stream_data;
reg [(pDATA_WIDTH-1):0]     stream_data_next;
reg [(pDATA_WIDTH-1):0]     coefficient_data;
reg [(pDATA_WIDTH-1):0]     Xn;
reg [(pDATA_WIDTH-1):0]     mult;
reg [(pDATA_WIDTH-1):0]     Yn;
wire                        single_pattern_done;
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
            if(wdata == 32'd600)
                n_state = WAIT_DATA;
            else  
                n_state = IDLE;
        end
        WAIT_DATA:begin
            if(awaddr == 12'h000 && wdata == 32'd1)
                n_state = CALC;
            else
                n_state = WAIT_DATA; 
        end
        CALC:begin
            if(sm_tlast && sm_tready && sm_tvalid)
                n_state = RESET;
            else
                n_state = CALC;
        end
        RESET:begin
            if(rdata == 32'd2)
                n_state = IDLE;
            else
                n_state = RESET; 
        end
        default:begin
                n_state = IDLE;
        end
    endcase
end
/////////////////////////////////////////////
///CONFUGURATION
/////////////////////////////////////////////
// configuration and data length storage
always@(posedge axis_clk or negedge axis_rst_n)begin
    if(!axis_rst_n)begin
        configuration <= 32'h0000_0000;
    end
    else begin
        if(n_state == WAIT_DATA) configuration <= 32'h0000_0001;
        else if(c_state == RESET) configuration <= 32'h0000_0002;
        else configuration <= 32'b0;
    end
end

/////////////////////////////////////////////
///AXILITE TAP
/////////////////////////////////////////////
assign tap_EN = 1'b1;
assign tap_A = tap_A_reg;
assign tap_Di = tap_Di_reg;
assign tap_WE = tap_WE_reg;

//count 11 taps  address for input
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_addr_cnt <= 4'b0;
    else if (c_state == WAIT_DATA) tap_addr_cnt <= 4'b0;
    else if(c_state == CALC)begin
        if(data_WE == 4'hf || tap_addr_cnt == 4'd10) tap_addr_cnt <= 4'b0;
        else tap_addr_cnt <= tap_addr_cnt + 1'b1; 
    end
    else tap_addr_cnt <= tap_addr_cnt;
end


// input the tap address 
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_A_reg <= 12'h000;
    else if (awvalid == 1 && awready == 1 && c_state == WAIT_DATA) begin
        if(awaddr >= 12'h020) tap_A_reg <= awaddr - 12'h020;
        else if(awaddr == 12'h010) tap_A_reg <= awaddr;
        else tap_A_reg <= 12'h000;
    end
    else if (arvalid == 1 && c_state == WAIT_DATA) begin
        if(araddr >= 12'h020) tap_A_reg <= araddr - 12'h020;
        else if(araddr == 12'h000) tap_A_reg <= 12'h000;
        else tap_A_reg <= tap_A_reg;
    end
    else if (c_state == CALC) begin    
        if(tap_addr_cnt == 6'd0) tap_A_reg <= 12'h000;
        else if (tap_addr_cnt == 6'd1) tap_A_reg <= 12'h004;
        else if (tap_addr_cnt == 6'd2) tap_A_reg <= 12'h008;
        else if (tap_addr_cnt == 6'd3) tap_A_reg <= 12'h00C;
        else if (tap_addr_cnt == 6'd4) tap_A_reg <= 12'h010;
        else if (tap_addr_cnt == 6'd5) tap_A_reg <= 12'h014;
        else if (tap_addr_cnt == 6'd6) tap_A_reg <= 12'h018;
        else if (tap_addr_cnt == 6'd7) tap_A_reg <= 12'h01C;
        else if (tap_addr_cnt == 6'd8) tap_A_reg <= 12'h020;
        else if (tap_addr_cnt == 6'd9) tap_A_reg <= 12'h024;
        else if (tap_addr_cnt == 6'd10) tap_A_reg <= 12'h028;
        else tap_A_reg <= tap_A_reg;
    end
    else tap_A_reg <= tap_A_reg;
end

//write tap
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_Di_reg <= 32'b0;
    else if (wvalid && wready) tap_Di_reg <= wdata;
    else tap_Di_reg <= tap_Di_reg;
end

//control the tap BRAM WE 
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_WE_reg <= 4'b0;
    else if ((wvalid && wready) && (awaddr != 12'h000)) tap_WE_reg <= 4'hf;
    else tap_WE_reg <= 4'b0;
end


/////////////////////////////////////////////
///AXILITE HANDSHAKE
/////////////////////////////////////////////
assign awready = awready_reg;
assign arready = (read_stage == 2'b01)? 1:0; 
assign wready = wready_reg;
assign rvalid = (~c_state == IDLE)?     1:
                (read_stage == 2'b01)?  1:0;
assign rdata =  (rvalid && c_state == WAIT_DATA)?   tap_Do:
                (rvalid && c_state == CALC)?        32'd0:
                (rvalid && c_state == RESET)?       configuration:
                                                    32'd4;
//write
always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n) awready_reg <= 1'b0;
    else if(c_state == WAIT_DATA)begin
        if(awvalid && awready) awready_reg <= 1'b0;
        else if((awvalid == 1'b1) && (awready == 1'b0)) awready_reg <= 1'b1;    
        else awready_reg <= 1'b0;
    end
    else awready_reg <= 1'b0;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n) wready_reg <= 1'b0;
    else if(c_state == WAIT_DATA)begin
        if(wvalid && wready) wready_reg <= 1'b0;
        else if((wvalid == 1'b1) && (wready == 1'b0)) wready_reg <= 1'b1;
    else wready_reg <= 1'b0;
    end
    else wready_reg <= 1'b0;
end

//read
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) read_stage <= 1'b0;
    else if (read_stage == 2'b00 && arvalid == 1'b1) read_stage <= 2'b01;
    else if (read_stage == 2'b01 && arvalid == 1'b1) read_stage <= 2'b10;
    else if (read_stage == 2'b10 && arvalid == 1'b1) read_stage <= 2'b00;
    else read_stage <= read_stage;
end

/////////////////////////////////////////////
///STREAM DATA
/////////////////////////////////////////////
assign data_EN = 1'b1;
assign data_WE = data_WE_reg;
assign data_Di = data_Di_reg;
assign data_A = data_A_reg;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) data_WE_reg <= 4'b0;
    else if(c_state == WAIT_DATA && wvalid && wready) data_WE_reg <= 4'hf;
    else if(c_state == CALC && ss_tready) data_WE_reg <= 4'hf;
    else data_WE_reg <= 4'b0;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) data_Di_reg <= 32'b0;
    else if(c_state == CALC) data_Di_reg <= ss_tdata;
    else data_Di_reg <= 32'b0;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) data_A_reg <= 12'h000;
    else begin
        case(c_state)
            WAIT_DATA:begin
                if(awvalid && awready)begin
                    if(awaddr != 12'h010) data_A_reg <= awaddr - 12'h020;
                    else if(arvalid) data_A_reg <= araddr - 12'h020;
                    else data_A_reg <= 12'h000;
                end
            end
            CALC:begin
                if(ss_tready == 1 && pattern_cycle < 12'd11) data_A_reg <= (pattern_cycle << 2);    // write data
                else if(ss_tready == 1 && pattern_cycle == 12'd11) data_A_reg <= 0;                 // wrtie data
                else if(ss_tready == 0) begin
                    if(data_A_reg == 12'h000) data_A_reg <= 12'h028;                                // read data
                    else data_A_reg <= data_A_reg - 12'h004;
                end
            end
        endcase
    end 
end

/////////////////////////////////////////////
///CALCULATE 
/////////////////////////////////////////////
assign ss_tready = (c_state == CALC && calc_cnt == 0)? 1:0;
assign single_pattern_done = (tap_addr_cnt == 2)? 1:0;
assign sm_tvalid = (pattern_number != 10'd1)? single_pattern_done:0;
assign sm_tdata = Yn;
assign sm_tlast = (pattern_number == 600 && single_pattern_done == 1)? 1:0;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) calc_cnt <= 4'b0;
    else if(c_state == CALC && calc_cnt <= 4'd11) calc_cnt <= calc_cnt + 4'b0001;
    else calc_cnt <= 4'b0;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) pattern_cycle <= 4'd0;
    else if(c_state == CALC && ss_tready && pattern_cycle < 4'd11) pattern_cycle <= pattern_cycle + 1;
    else if (pattern_cycle == 4'd11 && ss_tready == 1) pattern_cycle <= 1;
    else pattern_cycle <= pattern_cycle;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) pattern_number <= 10'b0;
    else if(pattern_number >= 10'd601) pattern_number <= 10'd0;
    else if(c_state == CALC && single_pattern_done == 1) pattern_number <= pattern_number + 10'd1;
    else pattern_number <= pattern_number;
end


always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) coefficient_data <= 0;
    else if (tap_A <= 12'h028) coefficient_data <= tap_Do;
    else coefficient_data <= coefficient_data;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) stream_data <= 0;
    else if (data_WE == 4'b1111) stream_data <= 0;
    else if (data_WE == 4'b0000) stream_data <= data_Do;
    else stream_data <= stream_data;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) stream_data_next <= 0;
    else stream_data_next <= stream_data;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) Xn <= 0;
    else Xn <= stream_data_next;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) mult <= 0;
    else mult <= Xn * coefficient_data;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) Yn <= 0;
    else if (c_state == CALC) begin
        if (single_pattern_done == 0) Yn <= Yn + mult;
        else if (single_pattern_done == 1) Yn <= 0;
        else Yn <= Yn;
    end
    else Yn <= Yn;
end












endmodule



