// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_slv.v
// Author        : Rongye
// Created On    : 2025-02-06 06:52
// Last Modified : 2025-03-15 20:30
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_SLV (
// Global
    input  wire                      clk,
    input  wire                      rst_n, 
    input  wire                      enable,

// AXI AR Channel
    input  wire                      axi_slv_arvalid,
    output wire                      axi_slv_arready,
    input  wire  [`AXI_ID_W    -1:0] axi_slv_arid,
    input  wire  [`AXI_ADDR_W  -1:0] axi_slv_araddr,
    input  wire  [`AXI_LEN_W   -1:0] axi_slv_arlen,
    input  wire  [`AXI_SIZE_W  -1:0] axi_slv_arsize,
    input  wire  [`AXI_BURST_W -1:0] axi_slv_arburst,

    output wire                      axi_slv_rvalid,
    input  wire                      axi_slv_rready,
    output wire  [`AXI_DATA_W  -1:0] axi_slv_rdata,
    output wire  [`AXI_RESP_W  -1:0] axi_slv_rresp,
    output wire                      axi_slv_rlast
);
localparam DLY       = 0.1;
localparam CLR_CNT_W = 4; 
localparam REG_ADDR  = 16'h0000;  // Default register address

//--------------------------------------------------------------------------------
// Inner Signal
//--------------------------------------------------------------------------------
wire                     rd_buff_set;         // Buffer set condition (read request handshake)
wire                     rd_buff_clr;         // Buffer clear condition (last read result handshake)
wire                     rd_buff_full;        // Buffer full flag

reg                      rd_valid_buff_r;     // Valid buffer register
reg                      rd_result_buff_r;    // Result buffer register
reg  [`AXI_LEN_W   -1:0] rd_result_index_r;   // Current read index

reg  [`AXI_ID_W    -1:0] rd_id_buff_r;        // AXI ID buffer
reg  [`AXI_ADDR_W  -1:0] rd_addr_buff_r;      // AXI Address buffer
reg  [`AXI_LEN_W   -1:0] rd_len_buff_r;       // AXI Length buffer
reg  [`AXI_SIZE_W  -1:0] rd_size_buff_r;      // AXI Size buffer
reg  [`AXI_BURST_W -1:0] rd_burst_buff_r;     // AXI Burst type buffer

reg  [`AXI_DATA_W  -1:0] rd_data_buff_r;      // Read data buffer
reg  [`AXI_RESP_W  -1:0] rd_resp_buff_r;      // Read response buffer

wire                     rd_req_en;           // Read request handshake (valid & ready)
wire                     rd_dec_miss;         // Address decode miss flag
wire                     rd_result_en;        // Read result handshake (valid & ready)
wire                     rd_result_last;      // Last read result flag

wire                     rd_data_get;         // Data fetch condition (counter max)
wire                     rd_data_err;         // Data error flag (simulated)

reg  [CLR_CNT_W    -1:0] clr_cnt_r;          // Clear counter for data fetch simulation

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign rd_buff_set = rd_req_en;  // Set buffer on read request handshake
assign rd_buff_clr = rd_result_en & rd_result_last;  // Clear buffer on last read result

assign rd_req_en      = axi_slv_arvalid & axi_slv_arready;  // Read request handshake
assign rd_dec_miss    = (axi_slv_araddr != REG_ADDR);       // Address decode miss
assign rd_result_en   = axi_slv_rvalid & axi_slv_rready;    // Read result handshake
assign rd_result_last = axi_slv_rlast;                      // Last read result

// Valid state control
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_valid_buff_r <= #DLY 1'b0;
    end
    else if (rd_buff_set) begin
        rd_valid_buff_r <= #DLY 1'b1;  // Set valid buffer
    end
    else if (rd_buff_clr) begin
        rd_valid_buff_r <= #DLY 1'b0;  // Clear valid buffer
    end
end
assign rd_buff_full = &rd_valid_buff_r;  // Buffer full when all bits set

// Result state control
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_result_buff_r  <= #DLY 1'b0;    
        rd_result_index_r <= #DLY {`AXI_LEN_W{1'b0}};
    end
    else if (rd_buff_set) begin
        rd_result_buff_r  <= #DLY rd_dec_miss ? 1'b1 : 1'b0;  // Set result buffer on decode miss
        rd_result_index_r <= #DLY rd_dec_miss ? axi_slv_arlen : {`AXI_LEN_W{1'b0}};  // Set index based on decode miss
    end
    else if (rd_result_en) begin
        rd_result_buff_r  <= #DLY rd_data_get;  // Update result buffer on data fetch
        rd_result_index_r <= #DLY rd_result_index_r + `AXI_LEN_W'h1;  // Increment read index
    end
    else if (rd_data_get) begin
        rd_result_buff_r  <= #DLY 1'b1;  // Set result buffer on data fetch
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
        rd_size_buff_r  <= #DLY {`AXI_SIZE_W{1'b0}};
        rd_burst_buff_r <= #DLY {`AXI_BURST_W{1'b0}};
    end
    else if (axi_slv_arvalid & axi_slv_arready) begin
        rd_id_buff_r    <= #DLY axi_slv_arid;    // Capture AXI ID
        rd_addr_buff_r  <= #DLY axi_slv_araddr;  // Capture AXI Address
        rd_len_buff_r   <= #DLY axi_slv_arlen;   // Capture AXI Length
        rd_size_buff_r  <= #DLY axi_slv_arsize;  // Capture AXI Size
        rd_burst_buff_r <= #DLY axi_slv_arburst; // Capture AXI Burst type
    end
end

//--------------------------------------------------------------------------------
// AXI R Payload Buffer
//--------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_data_buff_r  <= #DLY {`AXI_DATA_W{1'b0}};    
        rd_resp_buff_r  <= #DLY {`AXI_RESP_W{1'b0}};    
    end
    else if (rd_buff_set) begin
        rd_resp_buff_r  <= #DLY rd_dec_miss ? `AXI_RESP_DECERR : `AXI_RESP_OK;  // Set response on decode miss
    end
    else if (rd_data_get) begin
        rd_data_buff_r  <= #DLY rd_data_buff_r + `AXI_DATA_W'h1;  // Simulate data increment
        rd_resp_buff_r  <= #DLY rd_data_err ? `AXI_RESP_SLVERR : `AXI_RESP_OK;  // Set response on error
    end
end

//--------------------------------------------------------------------------------
// Simulate the data reading process
//--------------------------------------------------------------------------------
assign rd_data_get = (clr_cnt_r == ({CLR_CNT_W{1'b1}} - rd_result_index_r));  // Data fetch when counter max
assign rd_data_err = (rd_id_buff_r == `AXI_ID_W'h9) ;  // Simulated data error (when id is 9)
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        clr_cnt_r <= #DLY {CLR_CNT_W{1'b0}};
    end
    else if (rd_req_en & ~rd_dec_miss) begin
        clr_cnt_r <= #DLY clr_cnt_r + 1;  // Increment counter on valid request
    end
    else if (rd_result_en) begin
        clr_cnt_r <= #DLY (rd_result_index_r == rd_len_buff_r) ? {CLR_CNT_W{1'b0}} : `AXI_LEN_W'h1;  // Reset or increment counter
    end
    else if (rd_data_get) begin
        clr_cnt_r <= #DLY {CLR_CNT_W{1'b0}};  // Reset counter on data fetch
    end
    else if (clr_cnt_r != {CLR_CNT_W{1'b0}}) begin
        clr_cnt_r <= #DLY clr_cnt_r + 1;  // Auto-increment if non-zero
    end
end

//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_slv_arready = (~rd_buff_full);  // Ready when buffer not full

assign axi_slv_rvalid  = rd_result_buff_r;  // Read valid signal

assign axi_slv_rdata   = rd_data_buff_r;    // Read data output
assign axi_slv_rresp   = rd_resp_buff_r;    // Read response output
assign axi_slv_rlast   = (rd_len_buff_r == rd_result_index_r);  // Last read flag

endmodule
