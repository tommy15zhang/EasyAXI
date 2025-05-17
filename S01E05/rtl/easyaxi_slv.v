// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_slv.v
// Author        : Rongye
// Created On    : 2025-02-06 06:52
// Last Modified : 2025-05-17 07:39
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
reg  [OST_DEPTH-1:0]     rd_comp_buff_r;

reg  [OST_CNT_W-1:0]     rd_set_ptr_r;        
reg  [OST_CNT_W-1:0]     rd_clr_ptr_r;        
reg  [OST_CNT_W-1:0]     rd_result_ptr_r;      
reg  [OST_CNT_W-1:0]     rd_data_ptr_r;      

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

wire                     clr_cnt_en_r                     ;    
reg  [CLR_CNT_W    -1:0] clr_cnt_r                      ;     // Clear counter

wire                     rd_dec_miss;         
wire                     rd_result_en;        
wire [`AXI_ID_W    -1:0] rd_result_id;        
wire                     rd_result_last;      
wire                     clr_cnt_en;         
wire                     rd_data_get;         
wire                     rd_data_err;         

// Burst address calculation
wire [`AXI_ADDR_W-1:0]   rd_start_addr    [OST_DEPTH-1:0]; // Start address based on axi_addr
wire [`AXI_LEN_W -1:0]   rd_burst_lenth   [OST_DEPTH-1:0]; // Burst_length
wire [2**`AXI_SIZE_W-1:0]rd_number_bytes  [OST_DEPTH-1:0]; // Number of bytes
wire [`AXI_ADDR_W-1:0]   rd_wrap_boundary [OST_DEPTH-1:0]; // Wrap boundary address
wire [`AXI_ADDR_W-1:0]   rd_aligned_addr  [OST_DEPTH-1:0]; // Aligned address
wire                     rd_wrap_en       [OST_DEPTH-1:0]; // Wrap happen

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
        rd_result_ptr_r <= #DLY {OST_CNT_W{1'b0}};
    end
    else if (rd_result_en & rd_result_last) begin
        rd_result_ptr_r <= #DLY rd_result_ptr_r + 1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_data_ptr_r <= #DLY {OST_CNT_W{1'b0}};
    end
    else if (rd_result_en & rd_result_last) begin
        rd_data_ptr_r <= #DLY rd_data_ptr_r + 1;
    end
end
//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign rd_buff_set = axi_slv_arvalid & axi_slv_arready;
assign rd_buff_clr = rd_valid_buff_r[rd_clr_ptr_r] & ~rd_result_buff_r[rd_clr_ptr_r] & 
                     ~rd_comp_buff_r[rd_clr_ptr_r] & (|rd_valid_buff_r);
assign rd_buff_full = &rd_valid_buff_r;

assign axi_slv_arready = ~rd_buff_full;

assign rd_dec_miss    = 1'b0/*  (axi_slv_araddr != REG_ADDR) */;
assign rd_result_en   = axi_slv_rvalid & axi_slv_rready;
assign rd_result_id   = axi_slv_rid;
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
            if (rd_buff_set && (rd_set_ptr_r == i)) begin
                rd_valid_buff_r[i] <= #DLY 1'b1;
            end
            if (rd_buff_clr && (rd_clr_ptr_r == i)) begin
                rd_valid_buff_r[i] <= #DLY 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_result_buff_r[i] <= #DLY 1'b0;
        end
        else begin
            if (rd_buff_set && (rd_set_ptr_r == i)) begin
                rd_result_buff_r[i] <= #DLY rd_dec_miss ? 1'b1 : 1'b0;
            end
            else if (rd_data_get && (rd_data_ptr_r == i)) begin
                rd_result_buff_r[i] <= #DLY 1'b1;
            end
        else if (rd_result_en) begin
                rd_result_buff_r[i] <= #DLY 1'b0;
            end
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
            rd_id_buff_r[i]    <= #DLY {`AXI_ID_W{1'b0}};
            rd_addr_buff_r[i]  <= #DLY {`AXI_ADDR_W{1'b0}};
            rd_len_buff_r[i]   <= #DLY {`AXI_LEN_W{1'b0}};
            rd_size_buff_r[i]  <= #DLY {`AXI_SIZE_W{1'b0}};
            rd_burst_buff_r[i] <= #DLY {`AXI_BURST_W{1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_id_buff_r[i]    <= #DLY axi_slv_arid;
            rd_addr_buff_r[i]  <= #DLY axi_slv_araddr;
            rd_len_buff_r[i]   <= #DLY axi_slv_arlen;
            rd_size_buff_r[i]  <= #DLY axi_slv_arsize;
            rd_burst_buff_r[i] <= #DLY axi_slv_arburst;
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// Burst Address Control
//--------------------------------------------------------------------------------
generate 
for (i=0; i<OST_DEPTH; i=i+1) begin: BURST_CTRL
    assign rd_start_addr   [i] = rd_addr_buff_r[i];
    assign rd_number_bytes [i] = 1 << rd_size_buff_r[i];
    assign rd_burst_lenth  [i] = rd_len_buff_r[i] + 1;
    assign rd_aligned_addr [i] = rd_start_addr[i] / rd_number_bytes[i] * rd_number_bytes[i];
    assign rd_wrap_boundary[i] = rd_start_addr[i] / (rd_burst_lenth[i] * rd_number_bytes[i]) * (rd_burst_lenth[i] * rd_number_bytes[i]);
    assign rd_wrap_en      [i] = (rd_curr_addr_r[i] + rd_number_bytes[i]) == (rd_wrap_boundary[i] + (rd_burst_lenth[i] * rd_number_bytes[i]));

    // Read index control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_curr_index_r[i] <= #DLY {`AXI_LEN_W{1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_curr_index_r[i] <= #DLY rd_dec_miss ? rd_burst_lenth[i] : `AXI_LEN_W'h1;
        end
        else if (rd_result_en && (rd_result_ptr_r == i)) begin
            rd_curr_index_r[i] <= #DLY rd_curr_index_r[i] + `AXI_LEN_W'h1;
        end
    end

    // Read wrap control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_wrap_en_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_wrap_en_r[i] <= #DLY 1'b0;
        end
        else if (rd_data_get && (rd_data_ptr_r == i)) begin
            rd_wrap_en_r[i] <= #DLY rd_wrap_en_r[i] | rd_wrap_en[i];
        end
    end

    // Current address control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_curr_addr_r[i] <= #DLY {`AXI_ADDR_W{1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_curr_addr_r[i] <= #DLY axi_slv_araddr;
        end
        else if (rd_data_get && (rd_data_ptr_r == i)) begin
            case (rd_burst_buff_r[i])
                `AXI_BURST_FIXED: rd_curr_addr_r[i] <= #DLY axi_slv_araddr;
                `AXI_BURST_INCR : rd_curr_addr_r[i] <= #DLY rd_aligned_addr[i] + (rd_curr_index_r[i] * rd_number_bytes[i]);
                `AXI_BURST_WRAP : begin
                    if (rd_wrap_en[i])
                        rd_curr_addr_r[i] <= #DLY rd_wrap_boundary[i];
                    else if (rd_wrap_en_r[i])
                        rd_curr_addr_r[i] <= #DLY axi_slv_araddr + (rd_curr_index_r[i] * rd_number_bytes[i]) - (rd_number_bytes[i] * rd_burst_lenth[i]);
                    else
                        rd_curr_addr_r[i] <= #DLY rd_aligned_addr[i] + (rd_curr_index_r[i] * rd_number_bytes[i]);
                end
                default: rd_curr_addr_r[i] <= #DLY {`AXI_ADDR_W{1'b0}};
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
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_data_buff_r[i] <= #DLY {`AXI_DATA_W{1'b0}};
            rd_resp_buff_r[i] <= #DLY {`AXI_RESP_W{1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_resp_buff_r[i] <= #DLY rd_dec_miss ? `AXI_RESP_DECERR : `AXI_RESP_OKAY;
        end
        else if (rd_data_get && (rd_data_ptr_r == i)) begin
            rd_data_buff_r[i] <= #DLY {{`AXI_DATA_W-`AXI_ID_W-`AXI_ADDR_W{1'b0}}, rd_id_buff_r[i], rd_curr_addr_r[i]};
            rd_resp_buff_r[i] <= #DLY rd_data_err ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// Simulate the data reading process
//--------------------------------------------------------------------------------
assign clr_cnt_en  = rd_valid_buff_r[rd_result_ptr_r] & ~rd_result_buff_r[rd_result_ptr_r] & ~clr_cnt_en_r;
assign rd_data_get = (clr_cnt_r == rd_id_buff_r[rd_data_ptr_r]);
assign rd_data_err = (rd_id_buff_r[rd_data_ptr_r] == `AXI_ID_W'hF) & (rd_curr_index_r[rd_data_ptr_r] == rd_burst_lenth[rd_data_ptr_r]);

assign clr_cnt_en_r = (clr_cnt_r != {CLR_CNT_W{1'b0}});
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        clr_cnt_r <= #DLY {CLR_CNT_W{1'b0}};
    end
    else if (clr_cnt_en) begin
        clr_cnt_r <= #DLY `AXI_LEN_W'h1;
    end
    else if (rd_data_get) begin
        clr_cnt_r <= #DLY {CLR_CNT_W{1'b0}};
    end
    else begin
        if (clr_cnt_r != {CLR_CNT_W{1'b0}}) begin
            clr_cnt_r <= #DLY clr_cnt_r + 1;
        end
    end
end
//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_slv_rvalid  = rd_valid_buff_r[rd_result_ptr_r] & rd_result_buff_r[rd_result_ptr_r];
assign axi_slv_rid     = rd_id_buff_r[rd_result_ptr_r];
assign axi_slv_rdata   = rd_data_buff_r[rd_result_ptr_r];
assign axi_slv_rresp   = rd_resp_buff_r[rd_result_ptr_r];
assign axi_slv_rlast   = axi_slv_rvalid & (rd_curr_index_r[rd_result_ptr_r] == rd_burst_lenth[rd_result_ptr_r]);

endmodule
