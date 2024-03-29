`timescale 1ns / 1ps
`define CYCLE_TIME 10


module fir_tb
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11,
    parameter Data_Num    = 600
)();
wire                                awready;
wire                                wready;
reg                                 awvalid;
reg         [(pADDR_WIDTH-1): 0]    awaddr;
reg                                 wvalid;
reg signed  [(pDATA_WIDTH-1) : 0]   wdata;
wire                                arready;
reg                                 rready;
reg                                 arvalid;
reg         [(pADDR_WIDTH-1): 0]    araddr;
wire                                rvalid;
wire signed [(pDATA_WIDTH-1): 0]    rdata;
reg                                 ss_tvalid;
reg signed  [(pDATA_WIDTH-1) : 0]   ss_tdata;
reg                                 ss_tlast;
wire                                ss_tready;
reg                                 sm_tready;
wire                                sm_tvalid;
wire signed [(pDATA_WIDTH-1) : 0]   sm_tdata;
wire                                sm_tlast;
reg                                 axis_clk;
reg                                 axis_rst_n;

// ram for tap
wire [3:0]               tap_WE;
wire                     tap_EN;
wire [(pDATA_WIDTH-1):0] tap_Di;
wire [(pADDR_WIDTH-1):0] tap_A;
wire [(pDATA_WIDTH-1):0] tap_Do;

// ram for data RAM
wire [3:0]               data_WE;
wire                     data_EN;
wire [(pDATA_WIDTH-1):0] data_Di;
wire [(pADDR_WIDTH-1):0] data_A;
wire [(pDATA_WIDTH-1):0] data_Do;



fir fir_DUT(
    .awready(awready),
    .wready(wready),
    .awvalid(awvalid),
    .awaddr(awaddr),
    .wvalid(wvalid),
    .wdata(wdata),
    .arready(arready),
    .rready(rready),
    .arvalid(arvalid),
    .araddr(araddr),
    .rvalid(rvalid),
    .rdata(rdata),
    .ss_tvalid(ss_tvalid),
    .ss_tdata(ss_tdata),
    .ss_tlast(ss_tlast),
    .ss_tready(ss_tready),
    .sm_tready(sm_tready),
    .sm_tvalid(sm_tvalid),
    .sm_tdata(sm_tdata),
    .sm_tlast(sm_tlast),

    // ram for tap
    .tap_WE(tap_WE),
    .tap_EN(tap_EN),
    .tap_Di(tap_Di),
    .tap_A(tap_A),
    .tap_Do(tap_Do),

    // ram for data
    .data_WE(data_WE),
    .data_EN(data_EN),
    .data_Di(data_Di),
    .data_A(data_A),
    .data_Do(data_Do),

    .axis_clk(axis_clk),
    .axis_rst_n(axis_rst_n)

    );

// RAM for tap
bram11 tap_RAM (
    .CLK(axis_clk),
    .WE(tap_WE),
    .EN(tap_EN),
    .Di(tap_Di),
    .A(tap_A),
    .Do(tap_Do)
);

// RAM for data
bram11 data_RAM(
    .CLK(axis_clk),
    .WE(data_WE),
    .EN(data_EN),
    .Di(data_Di),
    .A(data_A),
    .Do(data_Do)
);

reg signed [(pDATA_WIDTH-1):0] Din_list[0:(Data_Num-1)];
reg signed [(pDATA_WIDTH-1):0] golden_list[0:(Data_Num-1)];

reg     error_coef;
reg     error_Yn;
reg     streaming_data_end_input;
integer latency;
reg     round1;
reg     round2;
reg     round3;

reg [31:0]  data_length;
reg signed [31:0] coef[0:10]; 
integer Din, golden, input_data, golden_data, m,i,j,k;
////////////////////////////////////////
//              Dump file
////////////////////////////////////////
initial begin
    $dumpfile("fir.vcd");
    $dumpvars();
end

////////////////////////////////////////
//              Clock
////////////////////////////////////////
parameter CYCLE = `CYCLE_TIME;
always #(CYCLE/2.0) axis_clk = (~axis_clk);
////////////////////////////////////////
//              top work
////////////////////////////////////////

initial test_case_gen;
initial begin
    coef[0]  =  32'd0;
    coef[1]  = -32'd10;
    coef[2]  = -32'd9;
    coef[3]  =  32'd23;
    coef[4]  =  32'd56;
    coef[5]  =  32'd63;
    coef[6]  =  32'd56;
    coef[7]  =  32'd23;
    coef[8]  = -32'd9;
    coef[9]  = -32'd10;
    coef[10] =  32'd0;
end

initial axilite_task1;
initial stream_task1;
initial check_answer_task1;

initial axilite_task2;
initial stream_task2;
initial check_answer_task2;

initial axilite_task3;
initial stream_task3;
initial check_answer_task3;

////////////////////////////////////////
//              Task round1
////////////////////////////////////////
task axilite_task1; 
    begin
        reset_task;
        axilite_write_task;
        axilite_read_task;
    end 
endtask

task stream_task1; 
    begin
        reset_task;
        stream_write_task;
    end 
endtask

task check_answer_task1; 
    begin
        reset_task;
        streaming_read_task;
        round1 = 0;
        round2 = 1;
    end 
endtask

////////////////////////////////////////
//              Task round2
////////////////////////////////////////
task axilite_task2; 
    begin
        wait(round2);
        axilite_write_task;
        axilite_read_task;
    end 
endtask

task stream_task2; 
    begin
        wait(round2);
        stream_write_task;
    end 
endtask

task check_answer_task2; 
    begin
        wait(round2);
        streaming_read_task;
        round2 = 0;
        round3 = 1;
    end 
endtask

////////////////////////////////////////
//              Task round3
////////////////////////////////////////
task axilite_task3; 
    begin
        wait(round3);
        axilite_write_task;
        axilite_read_task;
    end 
endtask

task stream_task3; 
    begin
        wait(round3);
        stream_write_task;
    end 
endtask

task check_answer_task3; 
    begin
        wait(round3);
        streaming_read_task;
        if (error_Yn == 0 & error_coef == 0) begin
            $display("---------------------------------------------");
            $display("-----------Congratulations! Pass-------------");
        end
        else begin
            $display("--------Simulation Failed---------");
        end
        $finish;
    end 
endtask

////////////////////////////////////////
//              Time out
////////////////////////////////////////
integer timeout = (10000000);
initial begin
    while(timeout > 0) begin
        @(posedge axis_clk);
        timeout = timeout - 1;
    end
    $display($time, "Simualtion Hang ....");
    $finish;
end

////////////////////////////////////////
//              Reset task
////////////////////////////////////////
task reset_task; 
    begin
        round2 <= 0; round3 <= 0;
        axis_clk = 0;
        axis_rst_n = 1;
        #(CYCLE/2.0) axis_rst_n = 0;
        #(CYCLE/2.0) axis_rst_n = 1;
        
        round1 = 1;
        awvalid = 0;
        wvalid = 0;
        awaddr = 32'd0;
        error_coef = 0;
        streaming_data_end_input = 0;
    end 
endtask

////////////////////////////////////////
//    Generate the testcase(from .dat) 
////////////////////////////////////////
task test_case_gen; 
    begin
        data_length = 0;
        Din = $fopen("/home/ubuntu/soc/SOC_lab_fir/fir/samples_triangular_wave.dat","r");
        golden = $fopen("/home/ubuntu/soc/SOC_lab_fir/fir/out_gold.dat","r");
        $display("------Filling the data input(AXI-Stream)------");
            for(m=0;m<Data_Num;m=m+1) begin
                input_data = $fscanf(Din,"%d", Din_list[m]);
                golden_data = $fscanf(golden,"%d", golden_list[m]);
                data_length = data_length + 1;
                // $display("Filling Streaming Data %0d: %0d", m, Din_list[m]);
            end
        $display("------Finish Filling the data input(AXI-Stream)------\n");
    end
endtask

////////////////////////////////////////
//            AXILITE Write
////////////////////////////////////////
task axilite_write_task; 
    begin
        $display("==========================================================");
        $display("=================TASK: axilite_write_task==================");
        $display("==========================================================");
        $display("----Start the coefficient input(AXI-lite)----");
        config_write(12'h10, data_length);
        for(k=0; k< Tape_Num; k=k+1) begin
            config_write(12'h20+4*k, coef[k]);
        end
        awvalid <= 0; wvalid <= 0;
        $display("----End the coefficient input(AXI-lite)----\n");
    end
endtask

task config_write;
    input [11:0]    addr;
    input [31:0]    data;
    begin
        awvalid <= 0; wvalid <= 0;
        @(posedge axis_clk);
        awvalid <= 1; awaddr <= addr;
        wvalid  <= 1; wdata <= data;
        @(posedge axis_clk);
        while (!wready) @(posedge axis_clk);
    end
endtask

////////////////////////////////////////
//            AXILITE Read
////////////////////////////////////////
task axilite_read_task;
    begin
        $display("==========================================================");
        $display("=================TASK: axilite_read_task==================");
        $display("==========================================================");
        $display(" Check Coefficient ...");
        for(k=0; k < Tape_Num; k=k+1) begin
            config_read_check(12'h20+4*k, coef[k], 32'hffffffff);
        end
        arvalid <= 0;
        $display(" Tape programming done ...");
        $display(" Start FIR");
        @(posedge axis_clk) config_write(12'h00, 32'h0000_0001);    // ap_start = 1
        $display("----End the coefficient input(AXI-lite)----");
    end
endtask

task config_read_check;
    input [11:0]        addr;
    input signed [31:0] exp_data;
    input [31:0]        mask;
    begin
        error_coef <= 0;
        arvalid <= 0;
        @(posedge axis_clk);
        arvalid <= 1; araddr <= addr;
        rready <= 1;
        @(posedge axis_clk);
        while (!rvalid) @(posedge axis_clk);
        if( (rdata & mask) != (exp_data & mask)) begin
            $display("ERROR: exp = %d, rdata = %d", exp_data, rdata);
            error_coef = 1;
            $finish;
        end else begin
            $display("OK: exp = %d, rdata = %d", exp_data, rdata);
        end
    end
endtask

////////////////////////////////////////
//            STREAM Write
////////////////////////////////////////
task stream_write_task;
    begin
        $display("==========================================================");
        $display("================TASK: stream_write_task================");
        $display("==========================================================");
        latency = 0;
        $display("------------Start simulation-----------");
        ss_tvalid <= 0;
        $display("----Start the data input(AXI-Stream)----");
        for(j = 0; j < data_length; j = j + 1) begin
            ss_tlast <= 0;
            if(j > 0) latency <= latency + 1;
            ss(Din_list[j]);
        end
        $display("-------------------End the data input(AXI-Stream)-------------------    ||");
        streaming_data_end_input = 1;
        while(!sm_tlast) begin     
            latency <= latency + 1;
            @(posedge axis_clk);
        end
        
    end
endtask

task ss;
    input  signed [31:0] in1;
    begin
        ss_tvalid <= 1;
        ss_tdata  <= in1;
        @(posedge axis_clk);
        while (!ss_tready) begin
            if(awaddr == 0 && !sm_tlast) begin
                latency <= latency + 1;
            end
            @(posedge axis_clk);
        end
    end
endtask

////////////////////////////////////////
//            STREAM Read
////////////////////////////////////////
task streaming_read_task;
    begin
        $display("==========================================================");
        $display("================TASK: streaming_read_task================");
        $display("==========================================================");
        error_Yn = 0;
        sm_tready = 1;
        wait (sm_tvalid);
        for(k=0;k < data_length;k=k+1) begin
            sm(golden_list[k],k);
        end
        config_read_check(12'h00, 32'h02, 32'h0000_0002); // check ap_done = 1 (0x00 [bit 1])
        config_read_check(12'h00, 32'h04, 32'h0000_0004); // check ap_idle = 1 (0x00 [bit 2])
        $display("Latency: %d", latency);
    end
endtask

task sm;
    input  signed [31:0] in2; // golden data
    input         [31:0] pcnt; // pattern count
    begin
        sm_tready <= 1;
        @(posedge axis_clk) 
        wait(sm_tvalid);
        while(!sm_tvalid) @(posedge axis_clk);
        if (sm_tdata != in2) begin
            $display("[ERROR] [Pattern %5d]   Golden answer: %5d, Your answer: %5d    $Time: %5d", pcnt, in2, sm_tdata, $time);
            error_Yn <= 1;
        end
        else begin
            if(streaming_data_end_input == 0) $display("[PASS] [Pattern %5d]    Golden answer: %5d, Your answer: %5d      $Time: %5d", pcnt, in2, sm_tdata, $time);
            else if(streaming_data_end_input == 1) $display("[PASS] [Pattern %5d]    Golden answer: %5d, Your answer: %5d      $Time: %5d\n", pcnt, in2, sm_tdata, $time);
        end
        @(posedge axis_clk);
    end
endtask

endmodule

