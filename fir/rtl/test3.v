/***********************************************
File: fir.v
Author: 張耀明
description: SOC design HW3
Implement a fir filter using only 1 multiplier
and 1 adder.
***********************************************/
`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    //axilite interface==============================
    //write(input)--
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    //read(output)---
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,
    //stream slave (input data)=========================
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    //stream master (output data)=======================
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

/*
Overall system design:
1. Only one multiplier and adder can be used, so if takes about 11~13(?) clk cycles to generated one output data,
2. Read and store all tap into bram. Once input data received, read all taps in the fir and store them in a tap_buffer 
   to prevent additional SRAM access.
3. After takeing about 11~13(?) clk cycles to generated one output data, axis_write outputs it. the calculation process don't need
   to wait for outputs process to finish.
*/

// =====FSM design=========== //
localparam STAT_IDLE = 3'd0;
localparam STAT_STORE_1_INPUT = 3'd1;
localparam STAT_CAL = 3'd2;
localparam STAT_FINISH = 3'd3;
reg [3-1:0]state, next_state;
wire one_input_finish;
// =====Axilite ctrl========= //
//fsm of axilite
localparam LITE_idle = 3'd0;
// for axilite write
localparam LITE_wfinish = 3'd1;
// for axilite read
localparam LITE_arready = 3'd2;
localparam LITE_rreq = 3'd3;
localparam LITE_read = 3'd4;
reg [3-1:0] lite_state, next_lite_state;
//axilite read module
reg arready_reg, rvalid_reg;
reg [(pADDR_WIDTH-1):0] araddr_buf;
//axilite write module
reg awready_reg,wready_reg;
reg [(pADDR_WIDTH-1):0] awaddr_buf;
reg [(pDATA_WIDTH-1):0] wdata_buf;
//axilite to config_ctrl module
reg [1:0] axilite_req; //(req)
reg [(pADDR_WIDTH-1):0] config_addr;
reg [(pDATA_WIDTH-1):0] w_data;
// =====Config ctrl=====//
//config_ctrl to axilite
wire [(pDATA_WIDTH-1):0] r_data;
//internal signal
reg desti; //destination 0:config_reg 1.tap_ram
reg desti_delay; //for r_data
reg W_EN; // indicate whether change the value of tap_ram or config reg
//confit ctrl to tap_ram_ctrl
wire[3:0]  config_tap_WE;
reg[(pDATA_WIDTH-1):0] config_tap_Di;
reg[(pADDR_WIDTH-1):0] config_tap_A;
reg[(pDATA_WIDTH-1):0] config_tap_Do;

//config ctrl to cfg_reg_ctrl
wire config_ctrl_reg_wen;
wire [8-1:0]config_ctrl_reg_in;
// =====taps_ram_ctrl======== //
reg [3:0]               tap_WE_reg;
reg [(pDATA_WIDTH-1):0] tap_Di_reg;
reg [(pADDR_WIDTH-1):0] tap_A_reg;
// =====Cfg_reg_ctrl=====//
wire [8-1:0]config_ctrl_reg_out;

// =====Config_reg=====//
reg [8-1:0] reg_in, reg_wmask;
reg reg_wen;
wire [8-1:0]reg_out;
reg [8-1:0]config_reg_buff, config_reg_buff_next;
wire ap_idle, ap_done, ap_start;

// =====Axis-out to fir_dataflow====== //
wire axis_finish;
wire [(pDATA_WIDTH-1):0] strm_data;
wire strm_valid;


// =====fir dataflow===== //
// fir_dataflow_cfg_ctrl
reg [7:0]fir_cfg_reg_in, fir_cfg_reg_mask;
reg fir_cfg_reg_wen;
//data ram ctrl
wire [(pADDR_WIDTH-1):0]data_ram_addr;
wire [(pDATA_WIDTH-1):0]data_ram_in;
wire [(pDATA_WIDTH-1):0]data_ram_out;
//data_ram addr ctrl
reg data_ram_we;
wire signed[4:0]data_ram_addr_pre;
wire signed[4:0]data_ram_addr_plus;
reg signed[4:0]data_ram_addr_start;
reg signed[4:0]ata_ram_addr_end;
//tap ram
wire [(pADDR_WIDTH-1):0]tap_ram_addr;
wire signed[(pDATA_WIDTH-1):0]tap_ram_out;
//output handshake
wire [(pDATA_WIDTH-1):0]fir_data;
reg fir_last;
//input handshake
wire fir_ready;
reg fir_valid;
//operation_cnt
reg signed[4:0]op_cnt,op_cnt_next,op_end;
//equl_10
wire equal_10_out;
//dataflow
reg [(pDATA_WIDTH-1):0]current_data_in;
wire signed[(pDATA_WIDTH-1):0]data_in;
wire signed[(pDATA_WIDTH-1):0]mul_out,adder_out;
//psum_buffer
reg[(pDATA_WIDTH-1):0] psum_buffer, psum_buffer_in;
wire[(pDATA_WIDTH-1):0]psum_buffer_out;
//fsm signal
reg last_data_processing,last_data_processing_next;



//******************************//
// FSM                          //
//******************************//
/*
FSM design:
state
0.idle  : (1)wait for coefficients(wait for axilite arvalid), if arvalid=1 go to state 1
          (2)wait for ap_start. If ap_start is set, go to state 1.
1.store_1_input: clean ap_start and set ap_idle
wait for 1 data input from axis. If 1 input is received, go to state 2.
2.calculation: Do the fir calculation and output 1 data. if tlast==1 go to state 3. else go to state 1.
3.finish: send axilite signal (ap_done, ap_idle) to testbench.
*/
always@*
    case(state)
        STAT_IDLE:
            if(ap_start)
                next_state = STAT_STORE_1_INPUT;
            else
                next_state = STAT_IDLE;
        STAT_STORE_1_INPUT:
            next_state = STAT_CAL;
        
        STAT_CAL:
        //TODO need modify
            if(one_input_finish & last_data_processing)
                next_state = STAT_FINISH;
            else if(one_input_finish & ~last_data_processing)
                next_state = STAT_STORE_1_INPUT;
            else 
                next_state = STAT_CAL;
        STAT_FINISH:
            next_state = STAT_IDLE;
        default:
            next_state = STAT_IDLE;
    endcase
always@(posedge axis_clk or axis_rst_n)
    if(~axis_rst_n)
        state = STAT_IDLE;
    else
        state = next_state;

assign one_input_finish = equal_10_out;
//******************************//
// AxiLite Controller           //
//******************************//


//=====lite fsm=====
always@*
    case(lite_state)
        LITE_idle:
            if(arvalid)
                next_lite_state = LITE_arready;
            else if(wready_reg && awready_reg) 
                next_lite_state = LITE_wfinish;
            else
                next_lite_state = LITE_idle;
        LITE_wfinish:// by the time, axilite has already received awaddr and wdata 
            next_lite_state = LITE_idle;
        LITE_arready:
            if(arready && arvalid)
                next_lite_state = LITE_rreq;
            else
                next_lite_state = LITE_arready;
        LITE_rreq:
            if(rready)
                next_lite_state = LITE_read;
            else
                next_lite_state = LITE_rreq;
        LITE_read:
            next_lite_state = LITE_idle;
        default:
            next_lite_state = LITE_idle;
    endcase
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        lite_state <= LITE_idle;
    else
        lite_state <= next_lite_state;
//===axilite_to_config===
always@* begin
    case(lite_state)
        LITE_wfinish:
            config_addr = awaddr_buf; // for write
        LITE_rreq:
            config_addr = araddr_buf;

        default:
            config_addr = 12'd1; // addr not being used
    endcase
end
always@*  begin
    case(lite_state)
        LITE_wfinish:
            axilite_req = 2'd1; //write
        LITE_rreq:
            axilite_req = 2'd2;
        default:
            axilite_req = 2'd0; // no operation
    endcase
end

always@*  begin
    case(lite_state)
        LITE_wfinish:
            w_data = wdata_buf;
        default:
            w_data = {pDATA_WIDTH{1'b0}};
    endcase
end




//===lite_write=====

assign awready = (lite_state == LITE_idle)? awready_reg : 1'b0;
assign wready =  (lite_state == LITE_idle)? wready_reg : 1'b0;
//hand shake block
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        awready_reg <= 1'b0;
    else
        case(lite_state)
            LITE_idle:
                if(awvalid)
                    awready_reg <= 1'b1;
                else
                    awready_reg <= awready_reg;
            default:
                awready_reg <= 1'b0;
        endcase
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        wready_reg <=1'b0;
    else
        case(lite_state)
            LITE_idle:
                if(wvalid)
                    wready_reg <= 1'b1;
                else
                    wready_reg <= wready_reg;
            default:
                wready_reg <= 1'b0;
        endcase
//data_addr block
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        awaddr_buf <= {pADDR_WIDTH{1'b0}};
    else
        case(lite_state)
            LITE_idle:
                if(awready)
                    awaddr_buf <= awaddr;
                else
                    awaddr_buf <= awaddr_buf;
            LITE_wfinish:
                // clean the buffer
                awaddr_buf <= {pADDR_WIDTH{1'b0}};
            default:
                awaddr_buf <= {pADDR_WIDTH{1'b0}};
        endcase
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        wdata_buf <= {pDATA_WIDTH{1'b0}};
    else
        case(lite_state)
            LITE_idle:
                if(wready)
                    wdata_buf <= wdata;
                else
                    wdata_buf <= wdata_buf;
            LITE_wfinish:
                // clean the buffer
                wdata_buf <= {pDATA_WIDTH{1'b0}};
            default:
                wdata_buf <= {pDATA_WIDTH{1'b0}};
        endcase



//===lite_read======
//wiring

assign arready = arready_reg;
assign rvalid = rvalid_reg;

// handshake block
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        arready_reg <= 1'b0;
    else
        case(lite_state)
            LITE_idle:
                if(arvalid)
                    arready_reg <= 1'b1;
                else
                    arready_reg <= 1'b0;
            default:
                awready_reg <= 1'b0;
        endcase
always@*
    if(~axis_rst_n)
        rvalid_reg <= 1'b0;
    else
        case(lite_state)
            LITE_read:
                rvalid_reg<=1'b1;
            default:
                rvalid_reg<=1'b0;
        endcase
// data_addr block
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        araddr_buf <= {pADDR_WIDTH{1'b0}};
    else
        case(lite_state)
            LITE_arready:
                if(arready)
                    araddr_buf <= araddr;
                else
                    araddr_buf <= araddr_buf;
            LITE_rreq:
                araddr_buf <= araddr_buf;
            LITE_read:
                araddr_buf <= {pADDR_WIDTH{1'b0}};
            default:
                araddr_buf <= {pADDR_WIDTH{1'b0}};
        endcase

assign rdata = (lite_state == LITE_read)? 
                        r_data: {pDATA_WIDTH{1'b0}};
//******************************//
//Config ctrl                   //
//******************************//
//===internal signal=======
//desti
always@*begin
    if(config_addr >= 12'h20)// taps
        desti = 1;
    else if(config_addr == 12'h10)// data_length(taps)
        desti = 1;
    else// config reg
        desti = 0;
end
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        desti_delay <= 1'b0;
    else
        desti_delay <= desti;
//W_EN (write or not)
always@*
    case(axilite_req)
        2'b01://write
            W_EN = 1'b1;
        default:// preserved
            W_EN = 1'b0;
    endcase
//===destination: tap_ram====
//config_tap_A
always@*begin
    if(config_addr >= 12'h20)// taps
        config_tap_A = config_addr - 12'h20; //transform to tap_addr
    else if(config_addr == 12'h10)// data_length
        config_tap_A = 12'h10<<2;
    else
        config_tap_A = 12'h10<<2;
end
//config_tap_WE
assign config_tap_WE = (desti)? {4{W_EN}} : 4'd0;
//config_tap_Di
always@*
    config_tap_Di = w_data;
always@*
    config_tap_Do = tap_Do;
  
//===destination: config_reg====
//config_ctrl_reg_in
assign config_ctrl_reg_in = w_data[8-1:0];
//config_ctrl_reg_wen
assign config_ctrl_reg_wen = (desti == 0)? W_EN:1'b0;
//config_ctrl_reg_out
    //寫在Cfg_reg_ctrl  

//===config_ctrl to axilite
//rdata
assign r_data = (desti_delay)?config_tap_Do:config_ctrl_reg_out; //這邊要寫config_ctrl_reg_out 但先這樣寫


//******************************//
// taps_ram_ctrl                //
//******************************//
assign tap_WE = tap_WE_reg;
assign tap_Di = tap_Di_reg;
assign tap_A = tap_A_reg;
assign tap_EN = 1'b1;
always@* begin
    case(state)
        STAT_IDLE:begin
            tap_WE_reg = config_tap_WE;
            tap_A_reg = config_tap_A;
            tap_Di_reg = config_tap_Di;
        end
        
        STAT_STORE_1_INPUT:begin
            tap_WE_reg = 1'b0;
            tap_A_reg = tap_ram_addr;
            tap_Di_reg = {pDATA_WIDTH{1'b0}};
        end
        STAT_CAL:begin
            tap_WE_reg = 1'b0;
            tap_A_reg = tap_ram_addr;
            tap_Di_reg = {pDATA_WIDTH{1'b0}};
        end
        STAT_FINISH:begin
            tap_WE_reg = 1'b0;
            tap_A_reg = tap_ram_addr;
            tap_Di_reg = {pDATA_WIDTH{1'b0}};
        end
        
        default:begin
            tap_WE_reg = 0;
            tap_A_reg = 0;
            tap_Di_reg = 0;
        end
    endcase
end
//******************************//
// Cfg_reg_ctrl                 //
//******************************//

assign config_ctrl_reg_out = reg_out;
always@*
    case(state)
        STAT_IDLE:
            // testbench can only write ap_start to 1, so the mask is always 00000001
            reg_wmask = 8'b0000_0001;
        STAT_STORE_1_INPUT:
            reg_wmask = fir_cfg_reg_mask;
        STAT_CAL:
            reg_wmask = fir_cfg_reg_mask;
        STAT_FINISH:
            reg_wmask = fir_cfg_reg_mask;
        default:
            reg_wmask = 8'b0000_0000;
    endcase

always@*
    case(state)
        STAT_IDLE:
            reg_wen = config_ctrl_reg_wen;
        STAT_STORE_1_INPUT:
            reg_wen = fir_cfg_reg_wen;
        STAT_CAL:
            reg_wen = fir_cfg_reg_wen;
        STAT_FINISH:
            reg_wen = fir_cfg_reg_wen;
        default:
            reg_wen = 0;
    endcase

always@*
    case(state)
        STAT_IDLE:
            reg_in = config_ctrl_reg_in;
        STAT_STORE_1_INPUT:
            reg_in = fir_cfg_reg_in;
        STAT_CAL:
            reg_in = fir_cfg_reg_in;
        STAT_FINISH:
            reg_in = fir_cfg_reg_in;
        default:
            reg_in = 8'd0;
    endcase
//******************************//
// Config_reg                   //
//******************************//

assign reg_out = config_reg_buff;
// block level protocal
assign ap_start = reg_out[0];
assign ap_done = reg_out[1];
assign ap_idle = reg_out[2];
//store the config state
integer NANDECODDA;
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        config_reg_buff <= 8'd4; //ap_idle = 0000_0100
    else
        if(reg_wen)
            for(NANDECODDA=0; NANDECODDA<8; NANDECODDA=NANDECODDA+1)
                if(reg_wmask[NANDECODDA])
                    config_reg_buff[NANDECODDA] <= reg_in[NANDECODDA];
                else
                    config_reg_buff[NANDECODDA] <= config_reg_buff[NANDECODDA];
        else
            config_reg_buff <= config_reg_buff;
//******************************//
// axis_ in                     //
//******************************//

axis_in axisin(
    .tvalid(ss_tvalid),
    .tdata(ss_tdata),
    .tlast(ss_tlast),
    .tready(ss_tready),

    // axis_in <-> fir_dataflow
    .strm_data(strm_data),
    .strm_valid(strm_valid),
    .fir_ready(fir_ready),

    //signal
    .axis_finish(axis_finish), 
    .ap_start(ap_start),

    //clk rst
    .clk(axis_clk),
    .rst_n(axis_rst_n)
);
//******************************//
// FIR Dataflow                 //
//******************************//
/*
implementation of single fir output:
--------------------------------------------------------------------------------------------------------
clk        :__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__/▔\__
ss_tdata_  :|data |____________________________________________________________________|data |___
tdata_in   :__|data|_____________________________________________________________________________
mem_cnt    :|0|1    |2    |3    |4    |5    |6    |7    |8   |9    |10    |0
tap_in     :__|tap0 |tap1 |tap2 |tap3 |tap4 |tap5 |tap6 |tap7|tap8 |tap9  |tap10|__________________
data_in    :__|dat0 |dat1 |dat2 |dat3 |dat4 |dat5 |dat6 |dat7|dat8 |dat9  |dat10|__________________
WE         :/▔\_______________________________________________________________________________
outvalid   :______________________________________________________________/▔▔\___________________
---------------------------------------------------------------------------------------------------------
*/
//========== fir_dataflow_cfg_ctrl==========
always@*
    case(state)
        STAT_IDLE:begin
            fir_cfg_reg_in = 8'b0000_0100;
            fir_cfg_reg_mask = 8'b0000_0100;
            fir_cfg_reg_wen = 1'b0;
        end
        STAT_STORE_1_INPUT:begin
            fir_cfg_reg_in = 8'b0000_0000;
            fir_cfg_reg_mask = 8'b0000_0101;
            fir_cfg_reg_wen = 1'b1;
        end
        STAT_CAL:begin
            fir_cfg_reg_in = 8'b0000_0000;
            fir_cfg_reg_mask = 8'b0000_0000;
            fir_cfg_reg_wen = 1'b0;
        end
        STAT_FINISH:begin
            fir_cfg_reg_in = 8'b0000_0110;
            fir_cfg_reg_mask = 8'b0000_0110;
            fir_cfg_reg_wen = 1'b1;
        end
        default:begin
            fir_cfg_reg_in = 8'b0000_0000;
            fir_cfg_reg_mask = 8'b0000_0000;
            fir_cfg_reg_wen = 1'b0;
        end
    endcase
// ==========operation cnt ==========
assign equal_10_out = (op_cnt == 10)? 1'b1 : 1'b0; 
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        op_end <= 5'd0;
    else
        if( strm_valid && (state == STAT_STORE_1_INPUT || state == STAT_CAL) )
            if(op_end < 5'd10)
                op_end <= op_end + 5'd1;
            else
                op_end <= 5'd10;
        else
            op_end <= op_end;
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        op_cnt <= 5'd0;
    else
        op_cnt <= op_cnt_next;

always@*
    case(state)
        STAT_IDLE:
            op_cnt_next = 0;
        STAT_STORE_1_INPUT:
            op_cnt_next = op_cnt+5'd1;
        STAT_CAL:
            if(~equal_10_out)
                op_cnt_next = op_cnt+5'd1;
            else
                op_cnt_next = 5'd0;
        STAT_FINISH:
            op_cnt_next = 5'd0;
        default:
            op_cnt_next = 5'd0;
    endcase
//==========output  handshake==========
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        fir_valid <= 1'b0;
    else
        fir_valid <= equal_10_out;
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        fir_last <= 1'b0;
    else
        if(last_data_processing & one_input_finish)
            fir_last <= 1'b1;
        else
            fir_last <= 1'b0;
//==========input handshake==========
assign fir_ready = equal_10_out & ~strm_valid;
//==========tap ram==========
assign tap_ram_addr = (op_cnt<<2);
assign tap_ram_out = tap_Do;
//==========Data_ram_addr_ctrl==========
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        data_ram_addr_start <= 5'd0;
    else
        if(equal_10_out)
            if(data_ram_addr_start<5'd10)
                data_ram_addr_start <= data_ram_addr_start + 5'd1;
            else
                data_ram_addr_start <= 5'd0;
        else
            data_ram_addr_start <= data_ram_addr_start;
always@*
    case(state)
        STAT_STORE_1_INPUT:
            data_ram_we = 1'b1;
        default:
            data_ram_we = 1'b0;
    endcase

assign data_ram_addr_pre = data_ram_addr_start - op_cnt;
assign data_ram_addr_plus = (data_ram_addr_pre <0)? (data_ram_addr_pre+5'd11) : data_ram_addr_pre;
//==========Data_ram=============
assign data_ram_addr = {5'b0000,data_ram_addr_plus,2'b00}; // << 2
assign data_ram_out = data_Do;
assign data_A = (state == STAT_IDLE )? tap_A:data_ram_addr;
//assign data_A = data_ram_addr;
assign data_Di = (state == STAT_IDLE )? {pDATA_WIDTH{1'b0}} : data_ram_in;
//assign data_Di = data_ram_in;
assign data_WE = (state == STAT_IDLE )? tap_WE:{4{data_ram_we}};
//assign data_WE = {4{data_ram_we}};

assign data_EN = 1'b1;
//===========dataflow==============
//x[t] current input
assign data_ram_in = strm_data;
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        current_data_in <= {pDATA_WIDTH{1'b0}};
    else
        if(strm_valid)
            current_data_in <= strm_data;
        else
            current_data_in <= {pDATA_WIDTH{1'b0}};
assign data_in = (op_cnt == 5'd1)? current_data_in:
                 (op_cnt <= op_end)?data_ram_out:{pDATA_WIDTH{1'b0}};
assign mul_out = tap_ram_out * data_in;
assign adder_out = mul_out + psum_buffer_out;
//psum_buffer
assign psum_buffer_out = psum_buffer;
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        psum_buffer <= {pDATA_WIDTH{1'b0}};
    else
        psum_buffer <= psum_buffer_in;
always@*
    case(state)
        STAT_STORE_1_INPUT:
            psum_buffer_in = {pDATA_WIDTH{1'b0}};
        STAT_CAL:
            psum_buffer_in = adder_out;
        default:
            psum_buffer_in = {pDATA_WIDTH{1'b0}};
    endcase
assign fir_data = adder_out;
//fsm signal
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        last_data_processing <= 1'b0;
    else
        last_data_processing <= last_data_processing_next;
always@*
    if (axis_finish)
        last_data_processing_next = 1'b1;
    else if(one_input_finish)
        last_data_processing_next = 1'b0;
    else
        last_data_processing_next = last_data_processing;

//=====axis_out=====
axis_out axisout(
    .fir_data(fir_data),
    .fir_valid(fir_valid),
    .fir_last(fir_last),

    .tdata(sm_tdata),
    .tvalid(sm_tvalid),
    .tlast(sm_tlast),

    .tready(sm_tready),

    .clk(axis_clk),
    .rst_n(axis_rst_n)
);
endmodule