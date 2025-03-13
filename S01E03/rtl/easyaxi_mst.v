// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_mst.v
// Author        : Rongye
// Created On    : 2025-02-06 06:45
// Last Modified : 2025-03-13 10:22
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

    output wire                            axi_mst_rready
);
localparam DLY = 0.1;
//--------------------------------------------------------------------------------
// Inner Signal
//--------------------------------------------------------------------------------
reg                    axi_mst_arvalid_r;
reg [`AXI_ADDR_W -1:0] axi_mst_araddr_r;
reg [`AXI_ID_W   -1:0] axi_mst_arid_r;

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        axi_mst_arvalid_r <= #DLY 1'b0;
    end
    else if (~axi_mst_arvalid_r & enable) begin  // Trigger on enable
        axi_mst_arvalid_r <= #DLY 1'b1;          // Assert valid
    end
    else if (axi_mst_arvalid & axi_mst_arready) begin
        axi_mst_arvalid_r <= #DLY 1'b0;          // Deassert after handshake
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        axi_mst_arid_r   <= #DLY {`AXI_ID_W{1'b0}};
        axi_mst_araddr_r <= #DLY {`AXI_ADDR_W{1'b0}};
    end
    else if (axi_mst_arvalid & axi_mst_arready) begin
        axi_mst_arid_r   <= #DLY axi_mst_arid_r + 1;  // Increment ID per transfer
        axi_mst_araddr_r <= #DLY (axi_mst_arid_r < 4'hA) ? 16'h0000 : 16'h0001;  // Switch base address
    end
end

//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_mst_arvalid = axi_mst_arvalid_r;
assign axi_mst_arid    = axi_mst_arid_r;
assign axi_mst_araddr  = axi_mst_araddr_r;
assign axi_mst_arlen   = `AXI_LEN_W'h3;
assign axi_mst_arsize  = `AXI_SIZE_4B;
assign axi_mst_arburst = `AXI_BURST_INCR;

assign axi_mst_rready  = 1'b1;
endmodule
