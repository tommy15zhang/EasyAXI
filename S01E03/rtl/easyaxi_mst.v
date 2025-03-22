// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_mst.v
// Author        : Rongye
// Created On    : 2025-02-06 06:45
// Last Modified : 2025-03-20 00:24
// ---------------------------------------------------------------------------------
// Description   : When the enable signal is valid, it triggers the valid signal to be set.  
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
    output wire  [`AXI_ID_W    -1:0] axi_mst_arid,
    output wire  [`AXI_ADDR_W  -1:0] axi_mst_araddr,
    output wire  [`AXI_LEN_W   -1:0] axi_mst_arlen,
    output wire  [`AXI_SIZE_W  -1:0] axi_mst_arsize,
    output wire  [`AXI_BURST_W -1:0] axi_mst_arburst,

    input  wire                      axi_mst_rvalid,
    output wire                      axi_mst_rready,
    input  wire  [`AXI_DATA_W  -1:0] axi_mst_rdata,
    input  wire  [`AXI_RESP_W  -1:0] axi_mst_rresp,
    input  wire                      axi_mst_rlast
);
localparam DLY = 0.1;

//--------------------------------------------------------------------------------
// Inner Signal
//--------------------------------------------------------------------------------
wire                     rd_buff_set;         // Buffer set condition (enable & not full)
wire                     rd_buff_clr;         // Buffer clear condition (valid & no pending request/complete)
wire                     rd_buff_full;        // Buffer full flag

reg                      rd_valid_buff_r;     // Valid buffer register
reg                      rd_req_buff_r;       // Request buffer register
reg                      rd_comp_buff_r;      // Completion buffer register

reg  [`AXI_ID_W    -1:0] rd_id_buff_r;        // AXI ID buffer
reg  [`AXI_ADDR_W  -1:0] rd_addr_buff_r;      // AXI Address buffer
reg  [`AXI_LEN_W   -1:0] rd_len_buff_r;       // AXI Length buffer
reg  [`AXI_SIZE_W  -1:0] rd_size_buff_r;      // AXI Size buffer
reg  [`AXI_BURST_W -1:0] rd_burst_buff_r;     // AXI Burst type buffer
    
reg  [`AXI_DATA_W  -1:0] rd_data_buff_r;      // Read data buffer
reg  [`AXI_RESP_W  -1:0] rd_resp_buff_r;      // Read response buffer

wire                     rd_req_en;           // Read request handshake (valid & ready)
wire                     rd_result_en;        // Read result handshake (valid & ready)
wire                     rd_result_last;      // Last read result flag
wire                     rd_result_err;       // Read response error flag

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign rd_buff_set = ~rd_buff_full & enable;  // Set buffer if not full and enabled
assign rd_buff_clr = rd_valid_buff_r & ~rd_req_buff_r & ~rd_comp_buff_r;  // Clear buffer if valid and no pending operations

assign rd_req_en      = axi_mst_arvalid & axi_mst_arready;  // Read request handshake
assign rd_result_en   = axi_mst_rvalid & axi_mst_rready;    // Read result handshake
assign rd_result_last = axi_mst_rlast;                      // Last read result

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

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_req_buff_r <= #DLY 1'b0;
    end
    else if (rd_buff_set) begin
        rd_req_buff_r <= #DLY 1'b1;  // Set request buffer
    end
    else if (rd_req_en) begin
        rd_req_buff_r <= #DLY 1'b0;  // Clear request buffer on handshake
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_comp_buff_r <= #DLY 1'b0;
    end
    else if (rd_buff_set) begin
        rd_comp_buff_r <= #DLY 1'b1;  // Set completion buffer
    end
    else if (rd_result_en & rd_result_last) begin
        rd_comp_buff_r <= #DLY 1'b0;  // Clear completion buffer on last result
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
        rd_size_buff_r  <= #DLY `AXI_SIZE_4B;
        rd_burst_buff_r <= #DLY `AXI_BURST_INCR;
    end
    else if (rd_req_en) begin
        rd_id_buff_r   <= #DLY rd_id_buff_r + `AXI_ID_W'h1;  // Increment ID
        rd_addr_buff_r <= #DLY ((rd_id_buff_r + `AXI_ID_W'h1) < `AXI_ID_W'hA) ? `AXI_ADDR_W'h0 : `AXI_ADDR_W'h1;  // Address toggle logic
        rd_len_buff_r  <= #DLY rd_len_buff_r + `AXI_LEN_W'h1;  // Increment length
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
    else if (rd_result_en) begin
        rd_data_buff_r  <= #DLY axi_mst_rdata;  // Capture read data
        rd_resp_buff_r  <= #DLY axi_mst_rresp;  // Capture read response
    end
end
assign rd_result_err = (axi_mst_rresp != `AXI_RESP_OK);  // Error flag for non-OK response

//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_mst_arvalid = rd_req_buff_r;  // AXI ARVALID signal
assign axi_mst_arid    = rd_id_buff_r;   // AXI ARID signal
assign axi_mst_araddr  = rd_addr_buff_r; // AXI ARADDR signal
assign axi_mst_arlen   = rd_len_buff_r;  // AXI ARLEN signal
assign axi_mst_arsize  = rd_size_buff_r; // AXI ARSIZE signal
assign axi_mst_arburst = rd_burst_buff_r; // AXI ARBURST signal

assign axi_mst_rready  = 1'b1;  // Always ready to accept read data
endmodule
