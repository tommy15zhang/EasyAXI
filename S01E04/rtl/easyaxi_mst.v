// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_mst.v
// Author        : Rongye
// Created On    : 2025-02-06 06:45
// Last Modified : 2025-04-14 08:13
// ---------------------------------------------------------------------------------
// Description   : AXI Master with burst support up to length 8
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_MST (
// Global
    input  wire                      clk,
    input  wire                      rst_n, 
    input  wire                      enable,

// AXI AR Channel
    output wire                      axi_mst_arvalid,
    input  wire                      axi_mst_arready,
    output wire  [`AXI_ID_W-1:0]     axi_mst_arid,
    output wire  [`AXI_ADDR_W-1:0]   axi_mst_araddr,
    output wire  [`AXI_LEN_W-1:0]    axi_mst_arlen,
    output wire  [`AXI_SIZE_W-1:0]   axi_mst_arsize,
    output wire  [`AXI_BURST_W-1:0]  axi_mst_arburst,

    input  wire                      axi_mst_rvalid,
    output wire                      axi_mst_rready,
    input  wire  [`AXI_DATA_W-1:0]   axi_mst_rdata,
    input  wire  [`AXI_RESP_W-1:0]   axi_mst_rresp,
    input  wire                      axi_mst_rlast
);

localparam DLY = 0.1;
localparam MAX_BURST_LEN = 8;  // Maximum burst length support
localparam BURST_CNT_W   = $clog2(MAX_BURST_LEN);  // Maximum burst length cnt width

//--------------------------------------------------------------------------------
// Inner Signal
//--------------------------------------------------------------------------------
wire                     rd_buff_set;
wire                     rd_buff_clr;
wire                     rd_buff_full;

reg                      rd_valid_buff_r;
reg                      rd_req_buff_r;
reg                      rd_comp_buff_r;

reg  [`AXI_ID_W-1:0]     rd_id_buff_r;
reg  [`AXI_ADDR_W-1:0]   rd_addr_buff_r;
reg  [`AXI_LEN_W-1:0]    rd_len_buff_r;
reg  [`AXI_SIZE_W-1:0]   rd_size_buff_r;
reg  [`AXI_BURST_W-1:0]  rd_burst_buff_r;
    
// Data buffer now supports up to 8 beats
reg  [`AXI_DATA_W-1:0]   rd_data_buff_r [MAX_BURST_LEN-1:0];
reg  [BURST_CNT_W-1:0]   rd_data_cnt_r;  // Counter for burst data
reg  [`AXI_RESP_W-1:0]   rd_resp_buff_r;
wire                     rd_resp_err;

wire                     rd_req_en;
wire                     rd_result_en;
wire                     rd_result_last;

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign rd_buff_set = ~rd_buff_full & enable;
assign rd_buff_clr = rd_valid_buff_r & ~rd_req_buff_r & ~rd_comp_buff_r;

assign rd_req_en      = axi_mst_arvalid & axi_mst_arready;
assign rd_result_en   = axi_mst_rvalid & axi_mst_rready;
assign rd_result_last = axi_mst_rlast;

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_valid_buff_r <= #DLY 1'b0;
    end
    else if (rd_buff_set) begin
        rd_valid_buff_r <= #DLY 1'b1;
    end
    else if (rd_buff_clr) begin
        rd_valid_buff_r <= #DLY 1'b0;
    end
end
assign rd_buff_full = &rd_valid_buff_r;

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_req_buff_r <= #DLY 1'b0;
    end
    else if (rd_buff_set) begin
        rd_req_buff_r <= #DLY 1'b1;
    end
    else if (rd_req_en) begin
        rd_req_buff_r <= #DLY 1'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_comp_buff_r <= #DLY 1'b0;
    end
    else if (rd_buff_set) begin
        rd_comp_buff_r <= #DLY 1'b1;
    end
    else if (rd_result_en & rd_result_last) begin
        rd_comp_buff_r <= #DLY 1'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_data_cnt_r  <= #DLY {BURST_CNT_W{1'b0}};
    end
    else if (rd_buff_set) begin
        rd_data_cnt_r  <= #DLY {BURST_CNT_W{1'b0}};
    end
    else if (rd_result_en) begin
        rd_data_cnt_r <= #DLY rd_data_cnt_r + 1;
    end
end
//--------------------------------------------------------------------------------
// AXI AR Payload Buffer
//--------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_id_buff_r    <= #DLY {`AXI_ID_W{1'b0}};
        rd_addr_buff_r  <= #DLY {`AXI_ADDR_W{1'b0}};
        rd_len_buff_r   <= #DLY {`AXI_LEN_W{1'b0}};
        rd_size_buff_r  <= #DLY `AXI_SIZE_1B;
        rd_burst_buff_r <= #DLY `AXI_BURST_INCR;
    end
    else if (rd_req_en) begin
        rd_id_buff_r   <= #DLY rd_id_buff_r + `AXI_ID_W'h1;
        
        // Burst configuration
        case (rd_id_buff_r[2:0])  // Use lower 3 bits for case selection
            3'b000: begin  // INCR burst, len=1
                rd_addr_buff_r  <= #DLY `AXI_ADDR_W'h0;
                rd_burst_buff_r <= #DLY `AXI_BURST_INCR;
                rd_len_buff_r   <= #DLY `AXI_LEN_W'h0;  // 1 transfer
                rd_size_buff_r  <= #DLY `AXI_SIZE_4B;
            end
            3'b001: begin  // INCR burst, len=4
                rd_addr_buff_r  <= #DLY `AXI_ADDR_W'h10;
                rd_burst_buff_r <= #DLY `AXI_BURST_INCR;
                rd_len_buff_r   <= #DLY `AXI_LEN_W'h3;  // 4 transfers
                rd_size_buff_r  <= #DLY `AXI_SIZE_4B;
            end
            3'b010: begin  // INCR burst, len=8
                rd_addr_buff_r  <= #DLY `AXI_ADDR_W'h20;
                rd_burst_buff_r <= #DLY `AXI_BURST_INCR;
                rd_len_buff_r   <= #DLY `AXI_LEN_W'h7;  // 8 transfers
                rd_size_buff_r  <= #DLY `AXI_SIZE_4B;
            end
            3'b011: begin  // FIXED burst, len=4
                rd_addr_buff_r  <= #DLY `AXI_ADDR_W'h30;
                rd_burst_buff_r <= #DLY `AXI_BURST_FIXED;
                rd_len_buff_r   <= #DLY `AXI_LEN_W'h3;
                rd_size_buff_r  <= #DLY `AXI_SIZE_4B;
            end
            3'b100: begin  // WRAP burst, len=4
                rd_addr_buff_r  <= #DLY `AXI_ADDR_W'h34;  // Must be aligned to 16B for 4x4B
                rd_burst_buff_r <= #DLY `AXI_BURST_WRAP;
                rd_len_buff_r   <= #DLY `AXI_LEN_W'h3;
                rd_size_buff_r  <= #DLY `AXI_SIZE_4B;
            end
            3'b101: begin  // WRAP burst, len=8
                rd_addr_buff_r  <= #DLY `AXI_ADDR_W'h38;  // Must be aligned to 32B for 8x4B
                rd_burst_buff_r <= #DLY `AXI_BURST_WRAP;
                rd_len_buff_r   <= #DLY `AXI_LEN_W'h7;
                rd_size_buff_r  <= #DLY `AXI_SIZE_4B;
            end
            default: begin  // Default INCR burst
                rd_addr_buff_r  <= #DLY rd_addr_buff_r + `AXI_ADDR_W'h40;
                rd_burst_buff_r <= #DLY `AXI_BURST_INCR;
                rd_len_buff_r   <= #DLY `AXI_LEN_W'h3;
                rd_size_buff_r  <= #DLY `AXI_SIZE_4B;
            end
        endcase
    end
end

//--------------------------------------------------------------------------------
// AXI R Payload Buffer - Now supports up to 8-beat burst
//--------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_resp_buff_r <= #DLY {`AXI_RESP_W{1'b0}};
    end
    else if (rd_result_en) begin
        rd_resp_buff_r <= #DLY (axi_mst_rresp > rd_resp_buff_r) ? axi_mst_rresp : rd_resp_buff_r; // merge is the worst resp
    end
end
assign rd_resp_err = (rd_resp_buff_r == `AXI_RESP_SLVERR) | (rd_resp_buff_r == `AXI_RESP_DECERR);
always @(posedge clk or negedge rst_n) begin
    integer i;
    if (~rst_n) begin
        for (i=0; i<MAX_BURST_LEN; i=i+1) begin
            rd_data_buff_r[i] <= #DLY {`AXI_DATA_W{1'b0}};
        end
    end
    else if (rd_result_en) begin
        rd_data_buff_r[rd_data_cnt_r] <= #DLY axi_mst_rdata;
    end
end
//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_mst_arvalid = rd_req_buff_r;
assign axi_mst_arid    = rd_id_buff_r;
assign axi_mst_araddr  = rd_addr_buff_r;
assign axi_mst_arlen   = rd_len_buff_r;
assign axi_mst_arsize  = rd_size_buff_r;
assign axi_mst_arburst = rd_burst_buff_r;

assign axi_mst_rready  = 1'b1;  // Always ready to accept read data

endmodule
