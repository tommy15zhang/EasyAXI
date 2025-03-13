// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_slv.v
// Author        : Rongye
// Created On    : 2025-02-06 06:52
// Last Modified : 2025-03-13 10:27
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_SLV (
// Global
    input  wire                            clk,
    input  wire                            rst_n, 
    input  wire                            enable,

// AXI AR Channel
    input  wire                            axi_slv_arvalid,
    output wire                            axi_slv_arready,
    input  wire  [`AXI_ID_W    -1:0]       axi_slv_arid,
    input  wire  [`AXI_ADDR_W  -1:0]       axi_slv_araddr,
    input  wire  [`AXI_LEN_W   -1:0]       axi_slv_arlen,
    input  wire  [`AXI_SIZE_W  -1:0]       axi_slv_arsize,
    input  wire  [`AXI_BURST_W -1:0]       axi_slv_arburst,

    output wire                            axi_slv_rvalid,
    input  wire                            axi_slv_rready,
    output wire  [`AXI_DATA_W    -1:0]     axi_slv_rdata,
    output wire  [`AXI_RESP_W    -1:0]     axi_slv_rresp,
    output wire                            axi_slv_rlast
);
localparam DLY       = 0.1;
localparam CLR_CNT_W = 4;     // Clear counter W
localparam REG_ADDR  = 16'h0000;  // Default register address
//--------------------------------------------------------------------------------
// Inner Signal
//--------------------------------------------------------------------------------
reg                       rd_valid_buff_r;
reg  [`AXI_ID_W    -1:0]  rd_id_buff_r;   
reg  [`AXI_ADDR_W  -1:0]  rd_addr_buff_r; 
reg  [`AXI_LEN_W   -1:0]  rd_len_buff_r; 
reg  [`AXI_SIZE_W  -1:0]  rd_size_buff_r; 
reg  [`AXI_BURST_W -1:0]  rd_burst_buff_r; 
reg  [`AXI_DATA_W  -1:0]  rd_data_buff_r; 
reg  [`AXI_RESP_W  -1:0]  rd_resp_buff_r; 
reg                       rd_last_buff_r; 

reg                       rd_result_en_r;
reg  [`AXI_LEN_W   -1:0]  rd_index_r; 

wire                      rd_buff_full;   

reg  [CLR_CNT_W    -1:0]  clr_cnt_r;   
wire                      rd_result_en;         
//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_valid_buff_r <= #DLY 1'b0;
    end
    else if (axi_slv_rvalid & axi_slv_rready & axi_slv_rlast) begin
        rd_valid_buff_r <= #DLY 1'b0;        // Clear buffer
    end
    else if (axi_slv_arvalid & axi_slv_arready) begin
        rd_valid_buff_r <= #DLY 1'b1;        // Latch valid on handshake
    end
end
assign rd_buff_full = &rd_valid_buff_r;          // All bits set = full

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_id_buff_r    <= #DLY {`AXI_ID_W{1'b0}};
        rd_addr_buff_r  <= #DLY {`AXI_ADDR_W{1'b0}};
        rd_len_buff_r   <= #DLY {`AXI_LEN_W{1'b0}};
        rd_size_buff_r  <= #DLY {`AXI_SIZE_W{1'b0}};
        rd_burst_buff_r <= #DLY {`AXI_BURST_W{1'b0}};
    end
    else if (axi_slv_arvalid & axi_slv_arready) begin
        rd_id_buff_r    <= #DLY axi_slv_arid;    // Capture incoming
        rd_addr_buff_r  <= #DLY axi_slv_araddr;
        rd_len_buff_r   <= #DLY axi_slv_arlen;
        rd_size_buff_r  <= #DLY axi_slv_arsize;
        rd_burst_buff_r <= #DLY axi_slv_arburst;
    end
end
assign rd_dec_miss = (axi_slv_araddr != REG_ADDR);  // Address decode miss

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_result_en_r  <= #DLY 1'b0;    
        rd_index_r      <= #DLY {`AXI_LEN_W{1'b0}};
    end
    else if (axi_slv_arvalid & axi_slv_arready) begin
        rd_result_en_r  <= #DLY rd_dec_miss ? 1'b1 : 1'b0;    
        rd_index_r      <= #DLY rd_dec_miss ? axi_slv_arlen : {`AXI_LEN_W{1'b0}};
    end
    else if (axi_slv_rvalid & axi_slv_rready) begin
        rd_result_en_r  <= #DLY 1'b0;    
        rd_index_r      <= #DLY rd_index_r + 1;
    end
    else if (rd_result_en) begin
        rd_result_en_r  <= #DLY 1'b1;    
        rd_index_r      <= #DLY rd_index_r;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_data_buff_r  <= #DLY {`AXI_DATA_W{1'b0}};    
        rd_resp_buff_r  <= #DLY {`AXI_RESP_W{1'b0}};    
    end
    else if (axi_slv_arvalid & axi_slv_arready) begin
        rd_data_buff_r  <= #DLY rd_data_buff_r;    
        rd_resp_buff_r  <= #DLY rd_dec_miss ? `AXI_RESP_DECERR : `AXI_RESP_OK;    
    end
    else if (rd_result_en) begin
        rd_data_buff_r  <= #DLY rd_data_buff_r + 1;    
        rd_resp_buff_r  <= #DLY rd_resp_buff_r;    
    end
end

assign rd_result_en = clr_cnt_r == {CLR_CNT_W{1'b1}} - rd_index_r;  // Clear when counter max
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        clr_cnt_r <= #DLY {CLR_CNT_W{1'b0}};
    end
    else if (axi_slv_arvalid & axi_slv_arready & ~rd_dec_miss) begin
        clr_cnt_r <= #DLY clr_cnt_r + 1;      // Increment on handshake
    end
    else if (rd_result_en) begin
        clr_cnt_r <= #DLY (rd_index_r == rd_len_buff_r) ? {CLR_CNT_W{1'b0}} : 4'h1;  // Reset counter
    end
    else if (clr_cnt_r != {CLR_CNT_W{1'b0}}) begin
        clr_cnt_r <= #DLY clr_cnt_r + 1;      // Auto-increment if non-zero
    end
end

//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_slv_arready = (~rd_buff_full);  // Ready when buffer available & address match

assign axi_slv_rvalid  = rd_result_en_r;
assign axi_slv_rdata   = rd_data_buff_r;
assign axi_slv_rresp   = rd_resp_buff_r;
assign axi_slv_rlast   = (rd_len_buff_r == rd_index_r);


endmodule
