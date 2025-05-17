// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_slv.v
// Author        : Rongye
// Created On    : 2025-02-06 06:52
// Last Modified : 2025-05-17 01:29
// ---------------------------------------------------------------------------------
// Description   : AXI Slave with burst support up to length 8 and outstanding capability
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_SLV #(
    parameter OST_DEPTH = 8  // Outstanding depth, must be power of 2
)(
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
    output wire  [`AXI_ID_W    -1:0] axi_slv_rid,
    output wire  [`AXI_DATA_W  -1:0] axi_slv_rdata,
    output wire  [`AXI_RESP_W  -1:0] axi_slv_rresp,
    output wire                      axi_slv_rlast
);

localparam DLY            = 0.1;
localparam CLR_CNT_W      = 4;
localparam REG_ADDR       = 16'h0000;  // Default register address
localparam OST_CNT_W      = $clog2(OST_DEPTH);  // Outstanding counter width

//--------------------------------------------------------------------------------
// Inner Signal 
//--------------------------------------------------------------------------------
wire                     rd_buff_set;         
wire                     rd_buff_clr;         
wire                     rd_buff_full;        

reg  [OST_DEPTH-1:0]     rd_valid_buff_r;     // Valid buffer register
reg  [OST_DEPTH-1:0]     rd_result_buff_r;    // Result buffer register

reg  [OST_CNT_W-1:0]     rd_req_ptr_r;        
reg  [OST_CNT_W-1:0]     rd_comp_ptr_r;      
reg  [OST_CNT_W-1:0]     rd_result_ptr_r;      

reg  [`AXI_LEN_W   -1:0] rd_curr_index_r [OST_DEPTH-1:0];     // Current read index
reg  [`AXI_ID_W    -1:0] rd_id_buff_r    [OST_DEPTH-1:0];     // AXI ID buffer
reg  [`AXI_ADDR_W  -1:0] rd_addr_buff_r  [OST_DEPTH-1:0];     // AXI Address buffer
reg  [`AXI_LEN_W   -1:0] rd_len_buff_r   [OST_DEPTH-1:0];     // AXI Length buffer
reg  [`AXI_SIZE_W  -1:0] rd_size_buff_r  [OST_DEPTH-1:0];     // AXI Size buffer
reg  [`AXI_BURST_W -1:0] rd_burst_buff_r [OST_DEPTH-1:0];     // AXI Burst type buffer
reg  [`AXI_DATA_W  -1:0] rd_data_buff_r  [OST_DEPTH-1:0];     // Read data buffer
reg  [`AXI_RESP_W  -1:0] rd_resp_buff_r  [OST_DEPTH-1:0];     // Read response buffer
reg  [`AXI_ADDR_W  -1:0] rd_curr_addr_r  [OST_DEPTH-1:0];     // Current address
reg                      rd_wrap_en_r    [OST_DEPTH-1:0];     // Wrap happen Tag
reg  [CLR_CNT_W    -1:0] clr_cnt_r       [OST_DEPTH-1:0];     // Clear counter

wire                     rd_req_en;           
wire                     rd_dec_miss;         
wire                     rd_result_en;        
wire                     rd_result_last;      
wire                     rd_data_get;         
wire                     rd_data_err;         

// Burst address calculation
wire [`AXI_ADDR_W-1:0]   rd_start_addr;       // Start address based on axi_addr
wire [`AXI_LEN_W -1:0]   rd_burst_lenth;      // Burst_length
wire [2**`AXI_SIZE_W-1:0]rd_number_bytes;     // Number of bytes
wire [`AXI_ADDR_W-1:0]   rd_wrap_boundary;    // Wrap boundary address
wire [`AXI_ADDR_W-1:0]   rd_aligned_addr;     // Aligned address 
wire                     rd_wrap_en;          // Wrap happen

// Pointer management
wire [OST_CNT_W-1:0]     rd_req_slot;         // Next available slot for request
wire [OST_CNT_W-1:0]     rd_comp_slot;        // Next completion slot
wire [OST_CNT_W-1:0]     rd_result_slot;      // Next result data slot

//--------------------------------------------------------------------------------
// Pointer Management
//--------------------------------------------------------------------------------
assign rd_req_slot    = rd_req_ptr_r;
assign rd_comp_slot   = rd_comp_ptr_r;
assign rd_result_slot = rd_result_ptr_r;

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_req_ptr_r  <= #DLY {OST_CNT_W{1'b0}};
    end
    else if (rd_buff_set) begin
        rd_req_ptr_r  <= #DLY rd_req_ptr_r + 1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_comp_ptr_r <= #DLY {OST_CNT_W{1'b0}};
    end
    else if (rd_buff_clr) begin
        rd_comp_ptr_r <= #DLY rd_comp_ptr_r + 1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_result_ptr_r <= #DLY {OST_CNT_W{1'b0}};
    end
    else if (rd_data_get) begin
        rd_result_ptr_r <= #DLY rd_result_ptr_r + 1;
    end
end
//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign rd_buff_set = axi_slv_arvalid & axi_slv_arready;
assign rd_buff_clr = rd_result_en & rd_result_last;
assign rd_buff_full = &rd_valid_buff_r;

assign axi_slv_arready = ~rd_buff_full;

assign rd_req_en      = rd_buff_set;
assign rd_dec_miss    = (axi_slv_araddr != REG_ADDR);
assign rd_result_en   = axi_slv_rvalid & axi_slv_rready;
assign rd_result_last = axi_slv_rlast;

genvar i;
generate 
for (i=0; i<OST_DEPTH; i=i+1) begin: OST_BUFFERS
    // Valid buffer control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_valid_buff_r <= #DLY 1'b0;
        end
        else begin
            if (rd_buff_set && (rd_req_slot == i)) begin
                rd_valid_buff_r[i] <= #DLY 1'b1;
            end
            if (rd_buff_clr && (rd_comp_slot == i)) begin
                rd_valid_buff_r[i] <= #DLY 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_result_buff_r[i] <= #DLY 1'b0;
        end
        else begin
            if (rd_buff_set && (rd_req_slot == i)) begin
                rd_result_buff_r[i] <= #DLY rd_dec_miss ? 1'b1 : 1'b0;
            end
            else if (rd_data_get && (rd_result_slot == i)) begin
                rd_result_buff_r[i] <= #DLY 1'b1;
            end
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// AXI AR Payload Buffer
//--------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    integer i;
    if (~rst_n) begin
        for (i=0; i<OST_DEPTH; i=i+1) begin
            rd_id_buff_r[i]    <= #DLY {`AXI_ID_W{1'b0}};
            rd_addr_buff_r[i]  <= #DLY {`AXI_ADDR_W{1'b0}};
            rd_len_buff_r[i]   <= #DLY {`AXI_LEN_W{1'b0}};
            rd_size_buff_r[i]  <= #DLY {`AXI_SIZE_W{1'b0}};
            rd_burst_buff_r[i] <= #DLY {`AXI_BURST_W{1'b0}};
        end
    end
    else if (rd_buff_set) begin
        rd_id_buff_r[rd_req_slot]    <= #DLY axi_slv_arid;
        rd_addr_buff_r[rd_req_slot]  <= #DLY axi_slv_araddr;
        rd_len_buff_r[rd_req_slot]   <= #DLY axi_slv_arlen;
        rd_size_buff_r[rd_req_slot]  <= #DLY axi_slv_arsize;
        rd_burst_buff_r[rd_req_slot] <= #DLY axi_slv_arburst;
    end
end

//--------------------------------------------------------------------------------
// Burst Address Control
//--------------------------------------------------------------------------------
assign rd_start_addr    = rd_addr_buff_r[rd_comp_slot];
assign rd_number_bytes  = 1 << rd_size_buff_r[rd_comp_slot];
assign rd_burst_lenth   = rd_len_buff_r[rd_comp_slot] + 1;
assign rd_aligned_addr  = rd_start_addr / rd_number_bytes * rd_number_bytes;
assign rd_wrap_boundary = rd_start_addr / (rd_burst_lenth * rd_number_bytes) * (rd_burst_lenth * rd_number_bytes);
assign rd_wrap_en       = (rd_curr_addr_r[rd_comp_slot] + rd_number_bytes) == (rd_wrap_boundary + (rd_burst_lenth * rd_number_bytes));

// Read index control
always @(posedge clk or negedge rst_n) begin
    integer i;
    if (~rst_n) begin
        for (i=0; i<OST_DEPTH; i=i+1) begin
            rd_curr_index_r[i] <= #DLY {`AXI_LEN_W{1'b0}};
        end
    end
    else if (rd_buff_set) begin
        rd_curr_index_r[rd_req_slot] <= #DLY rd_dec_miss ? rd_burst_lenth : `AXI_LEN_W'h1;
    end
    else if (rd_result_en) begin
        rd_curr_index_r[rd_comp_slot] <= #DLY rd_curr_index_r[rd_comp_slot] + `AXI_LEN_W'h1;
    end
end

// Read wrap control
always @(posedge clk or negedge rst_n) begin
    integer i;
    if (~rst_n) begin
        for (i=0; i<OST_DEPTH; i=i+1) begin
            rd_wrap_en_r[i] <= #DLY 1'b0;
        end
    end
    else if (rd_buff_set) begin
        rd_wrap_en_r[rd_req_slot] <= #DLY 1'b0;
    end
    else if (rd_data_get) begin
        rd_wrap_en_r[rd_comp_slot] <= #DLY rd_wrap_en_r[rd_comp_slot] | rd_wrap_en;
    end
end

// Current address control
always @(posedge clk or negedge rst_n) begin
    integer i;
    if (~rst_n) begin
        for (i=0; i<OST_DEPTH; i=i+1) begin
            rd_curr_addr_r[i] <= #DLY {`AXI_ADDR_W{1'b0}};
        end
    end
    else if (rd_buff_set) begin
        rd_curr_addr_r[rd_req_slot] <= #DLY axi_slv_araddr;
    end
    else if (rd_data_get) begin
        case (rd_burst_buff_r[rd_comp_slot])
            `AXI_BURST_FIXED: rd_curr_addr_r[rd_comp_slot] <= #DLY axi_slv_araddr;
            `AXI_BURST_INCR:  rd_curr_addr_r[rd_comp_slot] <= #DLY rd_aligned_addr + (rd_curr_index_r[rd_comp_slot] * rd_number_bytes);
            `AXI_BURST_WRAP: begin
                if (rd_wrap_en)
                    rd_curr_addr_r[rd_comp_slot] <= #DLY rd_wrap_boundary;
                else if (rd_wrap_en_r[rd_comp_slot])
                    rd_curr_addr_r[rd_comp_slot] <= #DLY axi_slv_araddr + (rd_curr_index_r[rd_comp_slot] * rd_number_bytes) - (rd_number_bytes * rd_burst_lenth);
                else
                    rd_curr_addr_r[rd_comp_slot] <= #DLY rd_aligned_addr + (rd_curr_index_r[rd_comp_slot] * rd_number_bytes);
            end
            default: rd_curr_addr_r[rd_comp_slot] <= #DLY {`AXI_ADDR_W{1'b0}};
        endcase
    end
end

//--------------------------------------------------------------------------------
// AXI R Payload Buffer
//--------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    integer i;
    if (~rst_n) begin
        for (i=0; i<OST_DEPTH; i=i+1) begin
            rd_data_buff_r[i] <= #DLY {`AXI_DATA_W{1'b0}};
            rd_resp_buff_r[i] <= #DLY {`AXI_RESP_W{1'b0}};
        end
    end
    else if (rd_buff_set) begin
        rd_resp_buff_r[rd_req_slot] <= #DLY rd_dec_miss ? `AXI_RESP_DECERR : `AXI_RESP_OKAY;
    end
    else if (rd_data_get) begin
        rd_data_buff_r[rd_comp_slot] <= #DLY {{`AXI_DATA_W-`AXI_ID_W-`AXI_ADDR_W{1'b0}}, rd_id_buff_r[rd_comp_slot], rd_curr_addr_r[rd_comp_slot]};
        rd_resp_buff_r[rd_comp_slot] <= #DLY rd_data_err ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
    end
end

//--------------------------------------------------------------------------------
// Simulate the data reading process
//--------------------------------------------------------------------------------
assign rd_data_get = (clr_cnt_r[rd_comp_slot] == ({CLR_CNT_W{1'b1}} - rd_curr_index_r[rd_comp_slot]));
assign rd_data_err = (rd_id_buff_r[rd_comp_slot] == `AXI_ID_W'hF) & (rd_curr_index_r[rd_comp_slot] == rd_burst_lenth);

always @(posedge clk or negedge rst_n) begin
    integer i;
    if (~rst_n) begin
        for (i=0; i<OST_DEPTH; i=i+1) begin
            clr_cnt_r[i] <= #DLY {CLR_CNT_W{1'b0}};
        end
    end
    else if (rd_buff_set && ~rd_dec_miss) begin
        clr_cnt_r[rd_req_slot] <= #DLY `AXI_LEN_W'h1;
    end
    else if (rd_result_en) begin
        clr_cnt_r[rd_comp_slot] <= #DLY (rd_curr_index_r[rd_comp_slot] == rd_burst_lenth) ? {CLR_CNT_W{1'b0}} : `AXI_LEN_W'h1;
    end
    else if (rd_data_get) begin
        clr_cnt_r[rd_comp_slot] <= #DLY {CLR_CNT_W{1'b0}};
    end
    else begin
        for (i=0; i<OST_DEPTH; i=i+1) begin
            if (clr_cnt_r[i] != {CLR_CNT_W{1'b0}}) begin
                clr_cnt_r[i] <= #DLY clr_cnt_r[i] + 1;
            end
        end
    end
end

//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_slv_rvalid  = rd_valid_buff_r[rd_comp_slot] & rd_result_buff_r[rd_comp_slot];
assign axi_slv_rid     = rd_id_buff_r[rd_comp_slot];
assign axi_slv_rdata   = rd_data_buff_r[rd_comp_slot];
assign axi_slv_rresp   = rd_resp_buff_r[rd_comp_slot];
assign axi_slv_rlast   = axi_slv_rvalid & (rd_curr_index_r[rd_comp_slot] == rd_burst_lenth);

endmodule
