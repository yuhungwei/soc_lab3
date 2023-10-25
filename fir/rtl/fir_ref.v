`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
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
    output  reg [(pDATA_WIDTH-1):0] rdata,    
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  reg                     ss_tready, 
    input   wire                     sm_tready, 
    output  reg                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  reg                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  reg [(pADDR_WIDTH-1):0] tap_A,
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


    // write your code here!
    parameter IDLE = 2'b00, WAIT_FOR_DATA = 2'b01, COMPUTE = 2'b10,  RESET = 2'b11;
    reg [1:0] current_state, next_state;

    reg [(pDATA_WIDTH-1):0] configuration;
    reg [(pDATA_WIDTH-1):0] data_length;

    reg [(pADDR_WIDTH-1):0] axilite_write_addr;
    reg [(pDATA_WIDTH-1):0] axilite_write_data;
    reg axilite_wdata_received;
    reg axilite_waddr_received;
    reg axilite_write_en;

    //reg [(pADDR_WIDTH-1):0] axilite_reg_raddr;
    reg [(pDATA_WIDTH-1):0] axilite_read_reg;
    reg axilite_out_valid;
    reg wait_tapram_data;
    reg axilite_raddr_received;
    reg [(pADDR_WIDTH-1):0] axilite_read_addr;
    reg read_data_length;
    reg read_configuration;
    
    reg first_data;
    reg [5:0] shift_counter;
    wire data_write_en;
    reg [1:0] data_read_stage;
    reg computing, outputing;
    reg last_data;
    reg signed [(pDATA_WIDTH-1):0] data_received;
    reg [9:0] data_counter;

    reg signed [(pDATA_WIDTH-1):0] accumulated_result;
    wire signed [(pDATA_WIDTH-1):0] mult_result;
    reg signed [(pDATA_WIDTH-1):0] mult1;
    wire signed [(pDATA_WIDTH-1):0] mult2;
    reg signed [(pDATA_WIDTH-1):0] current_data;

    always@(*)begin
        case(current_state)
        IDLE:begin
            if(configuration[0] == 1) next_state = WAIT_FOR_DATA;
            else next_state = IDLE;
        end
        WAIT_FOR_DATA:begin
            if(ss_tvalid) next_state = COMPUTE;
            else next_state = WAIT_FOR_DATA;
        end
        COMPUTE:begin
            if(sm_tlast && sm_tready && sm_tvalid) next_state = IDLE;
            else next_state = COMPUTE;
        end
        default:begin
            next_state = IDLE;
        end
        endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            current_state <= IDLE;
        end
        else begin
            current_state <= next_state;
        end
    end

    //-------------------------------------tap sram----------------------------------------------------
    assign tap_EN = 1;

    // axi lite write
    assign awready = !axilite_waddr_received;
    assign wready = !axilite_wdata_received;
    assign tap_WE = {4{axilite_write_en && axilite_write_addr != 12'h00 && axilite_write_addr != 12'h10}};
    assign tap_Di = axilite_write_data;

    always@(*)begin
        axilite_write_en = (axilite_waddr_received && axilite_wdata_received);
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            axilite_write_addr <= 0;
        end
        else begin
            if(awvalid && awready) axilite_write_addr <= awaddr;
            else axilite_write_addr <= axilite_write_addr;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            axilite_write_data <= 0;
        end
        else begin
            if(wvalid && wready) axilite_write_data <= wdata;
            else axilite_write_data <= axilite_write_data;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            axilite_waddr_received <= 0;
        end
        else begin
            if(awvalid && awready) axilite_waddr_received <= 1;
            else if(axilite_write_en) axilite_waddr_received <= 0;
            else axilite_waddr_received <= axilite_waddr_received;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            axilite_wdata_received <= 0;
        end
        else begin
            if(wvalid && wready) axilite_wdata_received <= 1;
            else if(axilite_write_en) axilite_wdata_received <= 0;
            else axilite_wdata_received <= axilite_wdata_received;
        end
    end

    // axi lite read
    assign arready = !rvalid && !axilite_write_en;
    assign rvalid = axilite_out_valid;

    /*may have some problems*/
    always@(*)begin
        if(read_configuration) rdata = configuration;
        else if(read_data_length) rdata = data_length;
        else rdata = axilite_read_reg;
    end
    /**/

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            axilite_raddr_received <= 0;
        end
        else begin
            if(arvalid && arready) axilite_raddr_received <= 1;
            //else if(rvalid && rready) axilite_raddr_received <= 0;
            else axilite_raddr_received <= 0;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            axilite_read_addr <= 0;
        end
        else begin
            if(arvalid && arready) axilite_read_addr <= araddr;
            else axilite_read_addr <= axilite_read_addr;
        end
    end

    /*may have some problems*/
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            read_data_length <= 0;
        end
        else begin
            if(arvalid && arready && araddr == 12'h10) read_data_length <= 1;
            else if(rvalid && rready) read_data_length <= 0;
            else read_data_length <= read_data_length;
        end
    end
    /**/

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            read_configuration <= 0;
        end
        else begin 
            if(arvalid && arready && araddr == 12'h00) read_configuration <= 1;
            else if(rvalid && rready) read_configuration <= 0;
            else read_configuration <= read_configuration;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            wait_tapram_data <= 0;
        end
        else begin
            if(axilite_raddr_received) wait_tapram_data <= 1;
            else wait_tapram_data <= 0;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            axilite_read_reg <= 0;
        end
        else begin 
            if(wait_tapram_data) axilite_read_reg <= tap_Do;
            else axilite_read_reg <= axilite_read_reg;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            axilite_out_valid <= 0;
        end
        else begin
            if(rvalid && rready) axilite_out_valid <= 0;
            else if(wait_tapram_data) axilite_out_valid <= 1;
            else axilite_out_valid <= axilite_out_valid;
        end
    end

    // axi lite address control

    always@(*)begin
        if(axilite_raddr_received && (axilite_read_addr != 12'h00 && axilite_read_addr != 12'h10)) tap_A = axilite_read_addr-12'h20;
        else if(axilite_write_en) tap_A = axilite_write_addr - 12'h20;
        else tap_A = shift_counter;
    end

    // configuration and data length storage
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            configuration <= 32'h0000_0004;
        end
        else begin
            if(axilite_write_en && axilite_write_addr == 12'h00) configuration <= axilite_write_data;
            else if(ss_tvalid && current_state == WAIT_FOR_DATA) configuration <= configuration & 32'hFFFF_FFFE;
            else if(sm_tready && sm_tvalid && sm_tlast) configuration <= 32'h0000_0006;
            else if(rready && rvalid && read_configuration && configuration[1] == 1) configuration <= configuration & 32'hFFFF_FFFD;
            else configuration <= configuration;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            data_length <= 0;
        end
        else begin
            if(axilite_write_en && axilite_write_addr == 12'h10) data_length <= axilite_write_data;
            else data_length <= data_length;
        end
    end

    //------------------------------------COMPUTE state logic-----------------------------------------------------------
    // data sram control
    assign data_EN = (current_state == COMPUTE);
    assign data_write_en = (data_read_stage == 0);
    assign data_WE = {4{data_write_en}};
    assign data_A = (data_read_stage == 2) ? shift_counter - 4 : shift_counter;
    //assign mult1 = (shift_counter == 0) ? data_received : (first_data) ? 0 : data_Do;
    assign mult2 = tap_Do;
    assign data_Di = mult1;
    assign mult_result = mult1 * mult2;
    assign sm_tdata = accumulated_result;

    always@(*)begin
        if(shift_counter == 0)begin
            if(data_counter >= data_length) mult1 = 0;
            else mult1 = data_received;
        end
        else begin
            if(first_data) mult1 = 0;
            else mult1 = current_data;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            data_counter <= 0;
        end
        else begin
            if(sm_tready && sm_tvalid) data_counter <= data_counter + 1;
            else if(next_state == IDLE) data_counter <= 0;
            else data_counter <= data_counter;
        end
    end
    
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            data_received <= 0;
        end
        else begin
            if(ss_tready && ss_tvalid) data_received <= ss_tdata;
            else data_received <= data_received;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            last_data <= 0;
        end
        else begin
            if(next_state == IDLE) last_data <= 0;
            else if(ss_tready && ss_tvalid) last_data <= ss_tlast;
            else last_data <= last_data;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            computing <= 0;
        end
        else begin
            if(ss_tready && ss_tvalid) computing <= 1;
            else if(shift_counter == 0 && data_write_en) computing <= 0;
            else if(sm_tready && sm_tvalid && (data_counter >= data_length - 1) && !sm_tlast) computing <= 1;
            else computing <= computing;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            outputing <= 0;
        end
        else begin
            if(shift_counter == 0 && data_write_en) outputing <= 1;
            else if(sm_tready && sm_tvalid) outputing <= 0;
            else outputing <= outputing;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            data_read_stage <= 2;
        end
        else begin
            if(current_state ==COMPUTE)begin
                if(sm_tready && sm_tvalid) data_read_stage <= 2;
                else if(computing) begin
                    if(data_read_stage == 0) data_read_stage <= 2;
                    else data_read_stage <= data_read_stage - 1;
                end
                else data_read_stage <= data_read_stage;
            end
            else begin
                data_read_stage <= data_read_stage;
            end
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            current_data <= 0;
        end
        else begin
            if(data_read_stage == 1) current_data <= data_Do;
            else current_data <= current_data;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            first_data <= 0;
        end
        else begin
            if(current_state == WAIT_FOR_DATA && next_state == COMPUTE) first_data <= 1;
            else if(sm_tready && sm_tvalid) first_data <= 0;
            else first_data <= first_data;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            shift_counter <= 40;
        end
        else begin
            if(shift_counter == 0 && data_write_en) shift_counter <= 40;
            else if(data_write_en && shift_counter != 0) shift_counter <= shift_counter - 4;
            else shift_counter <= shift_counter;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            accumulated_result <= 0;
        end
        else begin
            if(sm_tready && sm_tvalid) accumulated_result <= 0;
            else if(data_read_stage == 0) accumulated_result <= accumulated_result + mult_result;
            else accumulated_result <= accumulated_result;
        end
    end


    // ss control
    always@(*)begin
        ss_tready = (!computing && !outputing && current_state == COMPUTE);
    end

    // sm control
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            sm_tvalid <= 0;
        end
        else begin
            if(sm_tready && sm_tvalid) sm_tvalid <= 0;
            else if(shift_counter == 0 && data_write_en) sm_tvalid <= 1;
            else sm_tvalid <= sm_tvalid;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            sm_tlast <= 0;
        end
        else begin
            if(sm_tvalid && sm_tready) sm_tlast <= 0;
            else if(shift_counter == 0 && data_write_en) sm_tlast <= (data_counter == data_length + 9);
            else sm_tlast <= sm_tlast;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin

        end
        else begin

        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin

        end
        else begin

        end
    end


endmodule