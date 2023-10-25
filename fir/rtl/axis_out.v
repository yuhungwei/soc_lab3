module axis_out
#(  
    parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)(
    input wire[(pDATA_WIDTH-1):0] fir_data,
    input wire fir_valid,
    input wire fir_last,

    output reg [(pDATA_WIDTH-1):0]tdata,
    output reg tvalid,
    output reg tlast,

    input wire tready,

    input wire clk,
    input wire rst_n
);

    localparam STRM_IDLE = 0;
    localparam STRM_OUTPUT = 1;

    reg [(pDATA_WIDTH-1):0] buff, buff_in;
    reg last_buff, last_buff_next;
    reg state, next_state;
    always@*
        case(state)
            STRM_IDLE:
                if(fir_valid)
                    next_state = STRM_OUTPUT;
                else
                    next_state = STRM_IDLE;
            STRM_OUTPUT:
                if(tready)
                    next_state = STRM_IDLE;
                else
                    next_state = STRM_OUTPUT;
            default:
                next_state = STRM_OUTPUT;
        endcase

    always@(posedge clk or negedge rst_n)
        if(~rst_n)
            state <= STRM_IDLE;
        else
            state <= next_state; 

    always@(posedge clk or negedge rst_n)
        if(~rst_n)
            buff <= {pDATA_WIDTH{1'b0}};
        else
            buff <= buff_in;
    always@*
        case(state)
            STRM_IDLE:
                if(fir_valid)
                    buff_in = fir_data;
                else
                    buff_in = {pDATA_WIDTH{1'b0}};
            STRM_OUTPUT:
                if(tready)
                    buff_in = {pDATA_WIDTH{1'b0}};
                else
                    buff_in = buff;
            default:
                buff_in = {pDATA_WIDTH{1'b0}};
        endcase


    always@(posedge clk or negedge rst_n)
        if(~rst_n)
            last_buff <= 1'b0;
        else
            last_buff <= last_buff_next;
    always@*
        case(state)
            STRM_IDLE:
                if(fir_valid)
                    last_buff_next = fir_last;
                else
                    last_buff_next =  1'b0;
            STRM_OUTPUT:
                if(tready)
                    last_buff_next =  1'b0;
                else
                    last_buff_next =  last_buff;
            default:
                last_buff_next =  1'b0;
        endcase

    always@*
        case(state)
            STRM_OUTPUT:begin
                tdata = buff;
                tlast = last_buff;
                tvalid = 1'b1;
            end
            default:begin
                tdata = {pDATA_WIDTH{1'b0}};
                tlast = 1'b0;
                tvalid = 1'b0;
            end
        endcase
endmodule