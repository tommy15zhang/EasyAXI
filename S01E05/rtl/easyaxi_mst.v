// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_mst.v
// Author        : Rongye
// Created On    : 2025-02-06 06:45
// Last Modified : 2025-05-17 08:47
// ---------------------------------------------------------------------------------
// Description   : AXI Master with burst support up to length 8 and outstanding capability
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_MST #(
    parameter OST_DEPTH = 16  // Outstanding depth, must be power of 2
)(
// Global
    input  wire                      clk,
    input  wire                      rst_n, 
    input  wire                      enable,
    output wire                      error,

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
    input  wire  [`AXI_ID_W    -1:0] axi_mst_rid,
    input  wire  [`AXI_DATA_W  -1:0] axi_mst_rdata,
    input  wire  [`AXI_RESP_W  -1:0] axi_mst_rresp,
    input  wire                      axi_mst_rlast
);

localparam DLY = 0.1;
localparam MAX_BURST_LEN = 8;  // Maximum burst length support
localparam BURST_CNT_W   = $clog2(MAX_BURST_LEN);  // Maximum burst length cnt width
localparam OST_CNT_W     = $clog2(OST_DEPTH);      // Outstanding counter width

//--------------------------------------------------------------------------------
// Inner Signal
//--------------------------------------------------------------------------------
wire                                  rd_buff_set;
wire                                  rd_buff_clr;
wire                                  rd_buff_full;

// Outstanding request tracking
reg                                   rd_valid_buff_r[OST_DEPTH-1:0];
reg                                   rd_req_buff_r  [OST_DEPTH-1:0];
reg                                   rd_comp_buff_r [OST_DEPTH-1:0];

reg  [OST_DEPTH-1:0]                  rd_valid_bits;        
// Outstanding payload buffers
reg  [`AXI_ID_W                 -1:0] rd_id_buff_r   [OST_DEPTH-1:0];
reg  [`AXI_ADDR_W               -1:0] rd_addr_buff_r [OST_DEPTH-1:0];
reg  [`AXI_LEN_W                -1:0] rd_len_buff_r  [OST_DEPTH-1:0];
reg  [`AXI_SIZE_W               -1:0] rd_size_buff_r [OST_DEPTH-1:0];
reg  [`AXI_BURST_W              -1:0] rd_burst_buff_r[OST_DEPTH-1:0];
    
// Data buffer now supports up to 8 beats per outstanding request
reg  [`AXI_DATA_W*MAX_BURST_LEN -1:0] rd_data_buff_r [OST_DEPTH-1:0];
reg  [BURST_CNT_W               -1:0] rd_data_cnt_r  [OST_DEPTH-1:0];  // Counter for burst data
reg  [`AXI_RESP_W               -1:0] rd_resp_buff_r [OST_DEPTH-1:0];
wire [OST_DEPTH                 -1:0] rd_resp_err;

wire                                  rd_req_en;
wire                                  rd_result_en;
wire [`AXI_ID_W                 -1:0] rd_result_id;
wire                                  rd_result_last;

// Pointer and status signals
reg  [OST_CNT_W                 -1:0] rd_set_ptr_r; 
reg  [OST_CNT_W                 -1:0] rd_clr_ptr_r; 
reg  [OST_CNT_W                 -1:0] rd_req_ptr_r; 

//--------------------------------------------------------------------------------
// Pointer Management
//--------------------------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_set_ptr_r  <= #DLY {OST_CNT_W{1'b0}};
    end
    else if (rd_buff_set) begin
        rd_set_ptr_r  <= #DLY rd_set_ptr_r + 1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_clr_ptr_r <= #DLY {OST_CNT_W{1'b0}};
    end
    else if (rd_buff_clr) begin
        rd_clr_ptr_r <= #DLY rd_clr_ptr_r + 1;
    end
end
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_req_ptr_r  <= #DLY {OST_CNT_W{1'b0}};
    end
    else if (rd_req_en) begin
        rd_req_ptr_r  <= #DLY rd_req_ptr_r + 1;
    end
end
//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign rd_buff_set = ~rd_buff_full & enable;
assign rd_buff_clr = rd_valid_buff_r[rd_clr_ptr_r] & ~rd_req_buff_r[rd_clr_ptr_r] & ~rd_comp_buff_r[rd_clr_ptr_r];
always @(*) begin
    integer i;
    rd_valid_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin: CHK_FULL
        rd_valid_bits[i] = rd_valid_buff_r[i];
    end
end
assign rd_buff_full = &rd_valid_bits;

// Generate outstanding buffers
genvar i;
generate
for (i=0; i<OST_DEPTH; i=i+1) begin: OST_BUFFERS
    // Valid buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_valid_buff_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_valid_buff_r[i] <= #DLY 1'b1;
        end
        else if (rd_buff_clr && (rd_clr_ptr_r == i)) begin
            rd_valid_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Request buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_req_buff_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_req_buff_r[i] <= #DLY 1'b1;
        end
        else if (rd_req_en && (rd_req_ptr_r == i)) begin
            rd_req_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Completion buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_comp_buff_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_comp_buff_r[i] <= #DLY 1'b1;
        end
        else if (rd_result_en && rd_result_last && (rd_result_id == rd_id_buff_r[i])) begin
            rd_comp_buff_r[i] <= #DLY 1'b0;
        end
    end

end
endgenerate

//--------------------------------------------------------------------------------
// AXI AR Payload Buffer
//--------------------------------------------------------------------------------
generate
for (i=0; i<OST_DEPTH; i=i+1) begin: AR_PAYLOAD
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_id_buff_r    [i] <= #DLY{`AXI_ID_W{1'b0}};
            rd_addr_buff_r  [i] <= #DLY{`AXI_ADDR_W{1'b0}};
            rd_len_buff_r   [i] <= #DLY{`AXI_LEN_W{1'b0}};
            rd_size_buff_r  [i] <= #DLY `AXI_SIZE_1B;
            rd_burst_buff_r [i] <= #DLY `AXI_BURST_INCR;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_id_buff_r    [i] <= #DLY i;
            // Burst configuration
            case (i)  // Use lower 3 bits for case selection
                3'b000: begin  // INCR burst, len=1
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h0;
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h0;  // 1 transfer
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b001: begin  // INCR burst, len=4
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h10;
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;  // 4 transfers
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b010: begin  // INCR burst, len=8
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h20;
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h7;  // 8 transfers
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b011: begin  // FIXED burst, len=4
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h30;
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_FIXED;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b100: begin  // WRAP burst, len=4
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h34;  // Must be aligned to 16B for 4x4B
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_WRAP;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b101: begin  // WRAP burst, len=8
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h38;  // Must be aligned to 32B for 8x4B
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_WRAP;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h7;
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b110: begin  // FIXED burst, len=8
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h40;  // Must be aligned to 32B for 8x4B
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_FIXED;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h7;
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                default: begin  // Default INCR burst
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h80;
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
            endcase
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// AXI R Payload Buffer
//--------------------------------------------------------------------------------
generate
for (i=0; i<OST_DEPTH; i=i+1) begin: R_PAYLOAD
    // Response buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_resp_buff_r[i] <= #DLY {`AXI_RESP_W{1'b0}};
        end
        else if (rd_result_en && (rd_result_id == rd_id_buff_r[i])) begin
            rd_resp_buff_r[i] <= #DLY (axi_mst_rresp > rd_resp_buff_r[i]) ? axi_mst_rresp 
                                                                          : rd_resp_buff_r[i];
        end
    end
    assign rd_resp_err[i] = (rd_resp_buff_r[i] == `AXI_RESP_SLVERR) | 
                            (rd_resp_buff_r[i] == `AXI_RESP_DECERR);

    // Data buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end
        else if (rd_result_en && (rd_result_id == rd_id_buff_r[i])) begin
            rd_data_cnt_r[i] <= #DLY rd_data_cnt_r[i] + 1;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_data_buff_r[i] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
        end
        else if (rd_result_en && (rd_result_id == rd_id_buff_r[i])) begin
            rd_data_buff_r[i][(rd_data_cnt_r[i]*`AXI_DATA_W) +: `AXI_DATA_W] <= #DLY axi_mst_rdata;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_data_buff_r[i] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign error           = |rd_resp_err;

assign axi_mst_arvalid = |rd_valid_bits;
assign axi_mst_arid    = rd_id_buff_r    [rd_req_ptr_r];
assign axi_mst_araddr  = rd_addr_buff_r  [rd_req_ptr_r];
assign axi_mst_arlen   = rd_len_buff_r   [rd_req_ptr_r];
assign axi_mst_arsize  = rd_size_buff_r  [rd_req_ptr_r];
assign axi_mst_arburst = rd_burst_buff_r [rd_req_ptr_r];

assign axi_mst_rready  = 1'b1;  // Always ready to accept read data

// Handshake signals
assign rd_req_en      = axi_mst_arvalid & axi_mst_arready;
assign rd_result_en   = axi_mst_rvalid & axi_mst_rready;
assign rd_result_id   = axi_mst_rid;
assign rd_result_last = axi_mst_rlast;

endmodule
