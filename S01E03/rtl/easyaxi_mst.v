// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_mst.v
// Author        : Rongye
// Created On    : 2025-02-06 06:45
// Last Modified : 2025-03-15 09:49
// ---------------------------------------------------------------------------------
// Description   : When the enable signal is valid, it triggers the valid signal to be set.  
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_MST (
// Global
    input  wire                            clk,
    input  wire                            rst_n, 
    input  wire                            enable,

// AXI AR Channel
    output wire                            axi_mst_arvalid,
    input  wire                            axi_mst_arready,
    output wire  [`AXI_ID_W    -1:0]       axi_mst_arid,
    output wire  [`AXI_ADDR_W  -1:0]       axi_mst_araddr,
    output wire  [`AXI_LEN_W   -1:0]       axi_mst_arlen,
    output wire  [`AXI_SIZE_W  -1:0]       axi_mst_arsize,
    output wire  [`AXI_BURST_W -1:0]       axi_mst_arburst,

    input  wire                            axi_mst_rvalid,
    output wire                            axi_mst_rready,
    input  wire  [`AXI_DATA_W    -1:0]     axi_mst_rdata,
    input  wire  [`AXI_RESP_W    -1:0]     axi_mst_rresp,
    input  wire                            axi_mst_rlast
);
localparam DLY = 0.1;
//--------------------------------------------------------------------------------
// Inner Signal
//--------------------------------------------------------------------------------
wire                     rd_buff_set;         
wire                     rd_buff_clr;     
wire                     rd_buff_full;     

reg                      rd_valid_buff_r;
reg                      rd_req_buff_r;
reg                      rd_comp_buff_r;

reg  [`AXI_ID_W    -1:0] rd_id_buff_r;   
reg  [`AXI_ADDR_W  -1:0] rd_addr_buff_r; 
reg  [`AXI_LEN_W   -1:0] rd_len_buff_r; 
reg  [`AXI_SIZE_W  -1:0] rd_size_buff_r; 
reg  [`AXI_BURST_W -1:0] rd_burst_buff_r; 
    
reg  [`AXI_DATA_W  -1:0] rd_data_buff_r; 
reg  [`AXI_RESP_W  -1:0] rd_resp_buff_r; 

wire                     rd_result_err;         
//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign rd_buff_set = ~rd_buff_full & enable; 
assign rd_buff_clr = rd_valid_buff_r & rd_req_buff_r & rd_comp_buff_r; 
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_valid_buff_r <= #DLY 1'b0;
    end
    else if (rd_buff_set) begin
        rd_valid_buff_r <= #DLY 1'b1;        // Latch valid on handshake
    end
    else if (rd_buff_clr) begin
        rd_valid_buff_r <= #DLY 1'b0;        // Clear buffer
    end
end
assign rd_buff_full = &rd_valid_buff_r;          // All bits set = full

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_req_buff_r <= #DLY 1'b0;
    end
    else if (rd_buff_set) begin
        rd_req_buff_r <= #DLY 1'b0;        // Latch valid on handshake
    end
    else if (axi_mst_arvalid & axi_mst_arready) begin
        rd_req_buff_r <= #DLY 1'b1;        // Latch valid on handshake
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_comp_buff_r <= #DLY 1'b0;
    end
    else if (rd_buff_set) begin
        rd_comp_buff_r <= #DLY 1'b0;        // Latch valid on handshake
    end
    else if (axi_mst_rvalid & axi_mst_rready & axi_mst_rlast) begin
        rd_comp_buff_r <= #DLY 1'b1;        // Latch valid on handshake
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_id_buff_r   <= #DLY {`AXI_ID_W{1'b0}};
        rd_addr_buff_r <= #DLY {`AXI_ADDR_W{1'b0}};
    end
    else if (axi_mst_arvalid & axi_mst_arready) begin
        rd_id_buff_r   <= #DLY rd_id_buff_r + `AXI_ID_W'h1;  // Increment ID per transfer
        rd_addr_buff_r <= #DLY ((rd_id_buff_r + `AXI_ID_W'h1) < `AXI_ID_W'hA) ? `AXI_ADDR_W'h0 : `AXI_ADDR_W'h1;  // Switch base address
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_data_buff_r  <= #DLY {`AXI_DATA_W{1'b0}};    
        rd_resp_buff_r  <= #DLY {`AXI_RESP_W{1'b0}};    
    end
    else if (axi_mst_rvalid & axi_mst_rready) begin
        rd_data_buff_r  <= #DLY axi_mst_rdata;    
        rd_resp_buff_r  <= #DLY axi_mst_rresp;
    end
end
assign rd_result_err = (axi_mst_rresp != `AXI_RESP_OK); 
//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_mst_arvalid = rd_valid_buff_r & ~rd_req_buff_r;
assign axi_mst_arid    = rd_id_buff_r;
assign axi_mst_araddr  = rd_addr_buff_r;
assign axi_mst_arlen   = `AXI_LEN_W'h0;
assign axi_mst_arsize  = `AXI_SIZE_4B;
assign axi_mst_arburst = `AXI_BURST_INCR;

assign axi_mst_rready  = 1'b1;
endmodule
