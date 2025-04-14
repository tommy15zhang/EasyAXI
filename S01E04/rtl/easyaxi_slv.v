// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_slv.v
// Author        : Rongye
// Created On    : 2025-02-06 06:52
// Last Modified : 2025-04-14 08:18
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
localparam DLY            = 0.1;
localparam CLR_CNT_W      = 4;
localparam REG_ADDR       = 16'h0000;  // Default register address

//--------------------------------------------------------------------------------
// Inner Signal
//--------------------------------------------------------------------------------
wire                     rd_buff_set;         // Buffer set condition (read request handshake)
wire                     rd_buff_clr;         // Buffer clear condition (last read result handshake)
wire                     rd_buff_full;        // Buffer full flag

reg                      rd_valid_buff_r;     // Valid buffer register
reg                      rd_result_buff_r;    // Result buffer register
reg  [`AXI_LEN_W   -1:0] rd_curr_index_r;     // Current read index

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

reg  [CLR_CNT_W    -1:0] clr_cnt_r;           // Clear counter for data fetch simulation

// Burst address calculation
wire [`AXI_ADDR_W-1:0]   rd_start_addr;       // Start address based on axi_addr
wire [`AXI_LEN_W -1:0]   rd_burst_lenth;      // Burst_length
reg  [2**`AXI_SIZE_W-1:0]rd_number_bytes;     // Max Size is 128Byte -> 8'b1000_0000
wire [`AXI_ADDR_W-1:0]   rd_wrap_boundary;    // Wrap boundary address
wire [`AXI_ADDR_W-1:0]   rd_aligned_addr;     // Aligned address 
wire                     rd_wrap_en;          // Wrap happen
reg                      rd_wrap_en_r;        // Wrap happen Tag

reg  [`AXI_ADDR_W-1:0]   rd_curr_addr_r;      // Current address

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign rd_buff_set = rd_req_en;  // Set buffer on read request handshake
assign rd_buff_clr = rd_result_en & rd_result_last;  // Clear buffer on last read result

assign rd_req_en      = axi_slv_arvalid & axi_slv_arready;  // Read request handshake
assign rd_dec_miss    = 1'b0/*  (axi_slv_araddr != REG_ADDR) */;       // Address decode miss
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
    end
    else if (rd_buff_set) begin
        rd_result_buff_r  <= #DLY rd_dec_miss ? 1'b1 : 1'b0;  // Set result buffer on decode miss
    end
    else if (rd_result_en) begin
        rd_result_buff_r  <= #DLY rd_data_get;  // Update result buffer on data fetch
    end
    else if (rd_data_get) begin
        rd_result_buff_r  <= #DLY 1'b1;  // Set result buffer on data fetch
    end
end

// Burst Addr Ctrl
assign rd_start_addr    = rd_buff_set ? axi_slv_araddr      : rd_addr_buff_r;  // Start_Address raw addr
assign rd_number_bytes  = rd_buff_set ? 1 << axi_slv_arsize : 1 << rd_size_buff_r;  
assign rd_burst_lenth   = rd_buff_set ? axi_slv_arlen + 1   : rd_len_buff_r + 1;  
assign rd_aligned_addr  = rd_start_addr / rd_number_bytes * rd_number_bytes;  // Aligned_Address = (INT(Start_Address / Number_Bytes)) × Number_Bytes
assign rd_wrap_boundary = rd_start_addr / (rd_burst_lenth * rd_number_bytes) * (rd_burst_lenth * rd_number_bytes);  // Wrap_Boundary = (INT(Start_Address / (Number_Bytes × Burst_Length)))× (Number_Bytes × Burst_Length)
assign rd_wrap_en     = (rd_curr_addr_r + rd_number_bytes) == (rd_wrap_boundary + (rd_burst_lenth * rd_number_bytes));  // Wrap active

// Read index control
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_curr_index_r <= #DLY {`AXI_LEN_W{1'b0}};
    end
    else if (rd_buff_set) begin
        rd_curr_index_r <= #DLY rd_dec_miss ? rd_burst_lenth : `AXI_LEN_W'h1;  // Set index based on decode miss
    end
    else if (rd_result_en) begin
        rd_curr_index_r <= #DLY rd_curr_index_r + `AXI_LEN_W'h1;  // Increment read index
    end
end
// Read wrap control
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_wrap_en_r <= #DLY 1'b0;
    end
    else if (rd_buff_set) begin
        rd_wrap_en_r <= #DLY 1'b0;  // reset 0 when get new request
    end
    else if (rd_data_get) begin
        rd_wrap_en_r <= #DLY rd_wrap_en;  // Increment read wrap_en
    end
end
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_curr_addr_r  <= #DLY {`AXI_ADDR_W{1'b0}};
    end
    else if (rd_buff_set) begin
        rd_curr_addr_r  <= #DLY rd_start_addr;  // for any Burst type first transfer: Address_1 = Start_address
    end
    else if (rd_data_get) begin
        case (rd_burst_buff_r)
            `AXI_BURST_FIXED: begin 
                rd_curr_addr_r  <= #DLY rd_start_addr; // For FIXED, the address does not change: Address_N = Start_address
            end
            `AXI_BURST_INCR: begin 
                rd_curr_addr_r  <= #DLY rd_aligned_addr + (rd_curr_index_r * rd_number_bytes); // For INCR: Address_N = Aligned_address + (N-1) * Number_Bytes
            end
            `AXI_BURST_WRAP: begin
                if (rd_wrap_en)
                    rd_curr_addr_r  <= #DLY rd_wrap_boundary;  // For WRAP and wrap en: Address_N = Wrap_Boundary
                else if (rd_wrap_en_r)
                    rd_curr_addr_r  <= #DLY rd_start_addr + (rd_curr_index_r * rd_number_bytes) - (rd_number_bytes * rd_burst_lenth); // For WRAP and wrapped: Address_N = Start_Address + ((N – 1) × Number_Bytes) – (Number_Bytes × Burst_Length)
                else
                    rd_curr_addr_r  <= #DLY rd_aligned_addr + (rd_curr_index_r * rd_number_bytes); // For WRAP which not wrapped: Address_N = Aligned_address + (N-1) * Number_Bytes
            end
            default: // AXI_BURST_RSV is err Burst type
                rd_curr_addr_r <= #DLY {`AXI_ADDR_W{1'b0}};
        endcase
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
    else if (rd_buff_set) begin
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
        rd_data_buff_r  <= #DLY {{`AXI_DATA_W-`AXI_ID_W-`AXI_ADDR_W{1'b0}},rd_id_buff_r,rd_curr_addr_r};  // Use id and address as data for demonstration
        rd_resp_buff_r  <= #DLY rd_data_err ? `AXI_RESP_SLVERR : `AXI_RESP_OK;  // Set response on error
    end
end

//--------------------------------------------------------------------------------
// Simulate the data reading process
//--------------------------------------------------------------------------------
assign rd_data_get = (clr_cnt_r == ({CLR_CNT_W{1'b1}} - rd_curr_index_r));  // Data fetch when counter max
assign rd_data_err = (rd_id_buff_r == `AXI_ID_W'hF) & (rd_curr_index_r == rd_burst_lenth);  // Simulated data error (when id is 9)
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        clr_cnt_r <= #DLY {CLR_CNT_W{1'b0}};
    end
    else if (rd_req_en & ~rd_dec_miss) begin
        clr_cnt_r <= #DLY `AXI_LEN_W'h1;  // Increment counter on valid request
    end
    else if (rd_result_en) begin
        clr_cnt_r <= #DLY (rd_curr_index_r == rd_burst_lenth) ? {CLR_CNT_W{1'b0}} : `AXI_LEN_W'h1;  // Reset or increment counter
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
assign axi_slv_rlast   = axi_slv_rvalid & (rd_curr_index_r == rd_burst_lenth);  // Last read flag

endmodule
