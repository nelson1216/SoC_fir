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
    // write your code here!

//FSM state
reg [3:0]               state, next_state;
`define IDLE            3'd0
`define COEF            3'd1
`define CAL             3'd2 
`define DONE            3'd3

//Configuration address map
reg [(pDATA_WIDTH-1):0] config_addr;
reg                     config_en;
//AXILITE
reg [31:0]              data_length;
reg                     awready_reg;
reg                     wready_reg;
reg                     arready_reg;
reg                     rvalid_reg;
reg [(pDATA_WIDTH-1):0] rdata_reg;
reg [3:0]               read_addr_cnt;

reg [3:0]               tap_WE_reg;
reg [(pDATA_WIDTH-1):0] tap_Di_reg;
reg [(pADDR_WIDTH-1):0] tap_A_reg;

//AXISTREAM
reg                     ss_tready_reg;
reg                     sm_tvalid_reg; 
reg                     sm_tvalid_delay;
reg                     sm_tvalid_delay1;
reg                     sm_tvalid_delay2;
//reg [(pDATA_WIDTH-1):0] sm_tdata_reg;
reg                     sm_tlast_reg;

reg [3:0]               data_WE_reg;
reg [(pDATA_WIDTH-1):0] data_Di_reg;
reg [(pADDR_WIDTH-1):0] data_A_reg;
reg [(pADDR_WIDTH-1):0] data_write_cnt;

//FIR Calculate
reg [(pDATA_WIDTH-1):0]     mul_tap;
reg [(pDATA_WIDTH-1):0]     mul_data;
reg [(pDATA_WIDTH-1):0]     mul_result;
reg [(pDATA_WIDTH-1):0]     sum;
reg [(pADDR_WIDTH-1):0]     sum_cnt;
reg [4:0]                   tap_cnt;


assign awready = awready_reg;
assign wready = wready_reg;
assign arready = arready_reg;
assign rvalid = rvalid_reg;
assign rdata = rdata_reg;

assign tap_EN = 1'b1;
assign tap_WE = tap_WE_reg;
assign tap_Di = tap_Di_reg;
assign tap_A = tap_A_reg;

assign ss_tready = ss_tready_reg;
assign sm_tvalid = sm_tvalid_reg;
//assign sm_tdata = sm_tdata_reg;
assign sm_tdata = sum;
assign sm_tlast = sm_tlast_reg;

assign data_EN = 1'b1;
assign data_WE = data_WE_reg;
assign data_Di = data_Di_reg;
assign data_A = data_A_reg;



//FSM
always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)
        state <= `IDLE;
    else
        state <= next_state;
end

always @* begin
   next_state = `IDLE;
   case(state)
        `IDLE: begin
            if(wdata == 32'd600) next_state = `COEF;
            else next_state = `IDLE;
        end
        `COEF: begin
            if(rvalid & rready && araddr == 12'h048) next_state = `CAL;
            else next_state = `COEF;
        end
        `CAL: begin
            if(sm_tlast) next_state = `DONE;
            else next_state = `CAL;
        end    
        `DONE: begin
            if(rvalid & rready) next_state = `IDLE;
            else next_state = `DONE;
        end      
   endcase 
end

//Configuration address map
always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         config_addr <= 32'h0000_0004;   //ap_idle
    else if(rvalid & rready && state == `DONE)              config_addr <= 32'h0000_0004;
    else if(sum_cnt == 12'd601 && state == `CAL)            config_addr <= 32'h0000_0002;   //ap_done
    //else if(awaddr==12'h00 && wvalid && sm_tlast)         config_addr <= 32'h0000_0002;   //ap_done
    else if(wvalid && state == `CAL)begin      
        if(config_addr[2] == 1)                             config_addr <= wdata;   //ap_start = 1
        else                                                config_addr <= 32'h0000_0000;
    end
    else                                                    config_addr <= config_addr;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)     config_en <= 1'b0;
    else if(awaddr==12'h00 && wvalid)                       config_en <= 1'b1;
    else if(sm_tlast)                                       config_en <= 1'b1;
    else if(config_addr == 32'h0000_0002)                   config_en <= 1'b1;
    else                                                    config_en <= 1'b0;
end

//Get data length
always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)             data_length <= 32'd0;
    else if(awaddr == 12'h10)   data_length <= wdata;
    else                        data_length <= data_length;
end

//axi-stream handshake
//master to slave
always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         ss_tready_reg <= 1'b0;
    else if(state == `CAL && tap_cnt == 5'd10)              ss_tready_reg <= 1'b1;
    else if(state == `COEF && next_state == `CAL)           ss_tready_reg <= 1'b1;
    else                                                    ss_tready_reg <= 1'b0;
end

//Data RAM
always @(posedge axis_clk or negedge axis_rst_n)begin  //use for writing new data into BRAM
    if(~axis_rst_n)                                         data_write_cnt <= 12'h0;
    else if(state == `CAL && data_WE == 4'b1111)begin     
        if(data_write_cnt == 12'h028)                       data_write_cnt <= 12'h0;
        else                                                data_write_cnt <= data_write_cnt + 12'h004;
    end
    else                                                    data_write_cnt <= data_write_cnt;
end                                

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                 data_WE_reg <= 4'b0;
    else if(wvalid & wready)        data_WE_reg <= 4'b1111;
    //else if(sum_cnt == 12'd600)     data_WE_reg <= 4'b0;
    else if(ss_tvalid & ss_tready)  data_WE_reg <= 4'b1111;
    else                            data_WE_reg <= 4'b0;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         data_A_reg <= 12'h0;
    else if(state == `COEF && awaddr != 12'h010)            data_A_reg <= awaddr - 12'h20;
    else if(state == `CAL)begin
        if(ss_tvalid && ss_tready)                          data_A_reg <= data_write_cnt; //write data into BRAM
        else if(data_A_reg == 12'h000 && awaddr == 12'h000) data_A_reg <= 12'h028;
        else                                                data_A_reg <= data_A_reg - 12'h004; //point to the output data we need
    end                                
    else                                                    data_A_reg <= data_A_reg;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         data_Di_reg <= 32'd0;
    else if(ss_tdata == 32'd1 && ss_tready)                 data_Di_reg <= ss_tdata;
    else if(state == `CAL && data_WE == 4'b1111)            data_Di_reg <= ss_tdata;
    else if(state == `COEF)                                 data_Di_reg <= 32'd0;
    else                                                    data_Di_reg <= data_Di_reg;
end

//slave to master
always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         sm_tvalid_delay <= 1'd0;
    else if(state == `CAL && data_WE && sum_cnt != 12'd1)   sm_tvalid_delay <= 1'd1;
    else if(sm_tvalid_delay1)                               sm_tvalid_delay <= 1'd1;
    else                                                    sm_tvalid_delay <= 1'd0;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         sm_tvalid_reg <= 1'd0;
    else if(state == `CAL)                                  sm_tvalid_reg <= sm_tvalid_delay;
    else                                                    sm_tvalid_reg <= sm_tvalid_reg;
end
 
/*
always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         sm_tvalid_delay1 <= 1'd0;
    else if(sm_tvalid_delay2)                               sm_tvalid_delay1 <= 1'd1;
    else                                                    sm_tvalid_delay1 <= 1'd0;
end
*/

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         sm_tvalid_delay1 <= 1'd0;
    else if(sum_cnt == 12'd600 && ss_tready)                sm_tvalid_delay1 <= 1'd1;
    else                                                    sm_tvalid_delay1 <= 1'd0;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         sm_tlast_reg <= 1'd0;
    else if(sum_cnt == 12'd601 && sm_tvalid)                sm_tlast_reg <= 1'd1;
    else                                                    sm_tlast_reg <= 1'd0;
end

//axi-lite handshake
//write
always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         awready_reg <= 1'b0;
    else if(state == `CAL || state == `DONE)                awready_reg <= 1'b0;
    else if(awvalid & awready)                              awready_reg <= 1'b0;
    else if(awvalid && awready == 1'b0)                     awready_reg <= 1'b1;
    else                                                    awready_reg <= 1'b0;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         wready_reg <= 1'b0;
    else if(state == `CAL)                                  wready_reg <= 1'b0;
    else if(awvalid && wvalid == 1'b1)                      wready_reg <= 1'b0;
    else if(awvalid && wvalid == 1'b0)                      wready_reg <= 1'b1;
    else                                                    wready_reg <= 1'b0;
end

//read
always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         read_addr_cnt <= 4'd0;
    else if(arvalid && rready && read_addr_cnt == 4'd0)     read_addr_cnt <=4'd1;
    else if(arvalid && rready && read_addr_cnt == 4'd1)     read_addr_cnt <=4'd2;
    else                                                    read_addr_cnt <= 4'd0;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         arready_reg <= 1'b0;
    else if(state == `DONE && arvalid & rready)             arready_reg <= 1'b1;
    else if(arvalid & !rready)                              arready_reg <= 1'b1;
    else                                                    arready_reg <= 1'b0;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                 rvalid_reg <= 1'b0;
    else if(arready & rready)       rvalid_reg <= 1'b1;
    else                            rvalid_reg <= 1'b0; 
end

always @* begin
    if(~axis_rst_n)             rdata_reg = 32'h0;
    else if(config_en)          rdata_reg = config_addr;
    else if(rvalid & rready)    rdata_reg = tap_Do;
    else                        rdata_reg = 32'h0;
end

//tap RAM
always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)             tap_WE_reg <= 4'b0;
    else if(state == `COEF)begin
        if(awvalid & awready)   tap_WE_reg <= 4'b1111;
        else                    tap_WE_reg <= 4'b0;
    end
    else                        tap_WE_reg <= 4'b0;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         tap_A_reg <= 12'b0;
    else if(state == `CAL)begin
        //if(arready)                                         tap_A_reg <= 12'h000;
        if(!data_WE && tap_A == 12'h000)                    tap_A_reg <= 12'h000;
        else if(tap_A == 12'h028)                           tap_A_reg <= 12'h000;
        else                                                tap_A_reg <= tap_A_reg + 12'h004;
    end
    else if(state == `COEF)begin
        if(awvalid & awready)begin
            if(awaddr == 12'h010)   tap_A_reg <= awaddr;
            else                    tap_A_reg <= awaddr - 12'h20;
        end
    else if(arvalid & !rready)      tap_A_reg <= araddr - 12'h20;
    else                            tap_A_reg <= tap_A_reg;
    end
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)             tap_Di_reg <= 32'b0;
    else if(awvalid & awready)  tap_Di_reg <= wdata;
    else                        tap_Di_reg <= tap_Di_reg;
end

//Calculate
//assign mul_tap = tap_Do;
reg [(pDATA_WIDTH-1):0]         mul_tap_delay;
reg [(pDATA_WIDTH-1):0]         mul_data_delay;

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)             mul_tap_delay <= 32'd0;
    else if(state == `CAL)      mul_tap_delay <= tap_Do;
    else                        mul_tap_delay <= mul_tap_delay;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)             mul_tap <= 32'd0;
    else if(state == `CAL)      mul_tap <= mul_tap_delay;
    else                        mul_tap <= mul_tap;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)             mul_data_delay <= 32'd0;
    else if(state == `CAL)      mul_data_delay <= data_Do;        
    else                        mul_data_delay <= mul_data;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)             mul_data <= 32'd0;
    else if(state == `CAL)      mul_data <= mul_data_delay;
    else                        mul_data <= mul_data;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)             mul_result <= 32'd0;
    else if(state == `CAL)      mul_result <= mul_data * mul_tap;
    else                        mul_result <= mul_result;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)             sum <= 32'd0;
    else if(state == `CAL)begin
        if(sm_tvalid)           sum <= 32'd0;      
        else                    sum <= sum + mul_result;
    end
    else                        sum <= sum;
end 


always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)                                         sum_cnt <= 12'd0;
    else if(ss_tvalid & ss_tready)                          sum_cnt <= sum_cnt + 12'd1;
    else if(ss_tready && sum_cnt == 12'd600)                sum_cnt <= sum_cnt + 12'd1;
    else if(ss_tready && sum_cnt == 12'd601)                sum_cnt <= 12'd0;                  
    else                                                    sum_cnt <= sum_cnt;
end

always @(posedge axis_clk or negedge axis_rst_n)begin
    if(~axis_rst_n)             tap_cnt <= 5'd0;
    else if(state == `CAL)begin
        if(tap_cnt == 5'd10)    tap_cnt <= 5'd0;
        else if(ss_tready)      tap_cnt <= 5'd0;
        else                    tap_cnt <= tap_cnt + 5'd1;
    end
    else                        tap_cnt <= tap_cnt;
end

endmodule