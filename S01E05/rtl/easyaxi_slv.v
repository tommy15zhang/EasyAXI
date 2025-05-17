// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_slv.v
// Author        : Rongye
// Created On    : 2025-02-06 06:52
// Last Modified : 2025-05-16 23:26
// ---------------------------------------------------------------------------------
// Description   : AXI Slave with outstanding support (array-style declaration)
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

localparam DLY = 0.1;
localparam MAX_BURST_LEN = 8;
localparam BURST_CNT_W   = $clog2(MAX_BURST_LEN);
localparam OST_CNT_W     = $clog2(OST_DEPTH);
localparam REG_ADDR      = 16'h0000;

//--------------------------------------------------------------------------------
// Inner Signal (Array-style declaration)
//--------------------------------------------------------------------------------
// Control signals
reg                     rd_valid_buff_r[OST_DEPTH-1:0];
reg                     rd_req_buff_r[OST_DEPTH-1:0]; 
reg                     rd_comp_buff_r[OST_DEPTH-1:0];

// Payload buffers
reg  [`AXI_ID_W-1:0]    rd_id_buff_r[OST_DEPTH-1:0];
reg  [`AXI_ADDR_W-1:0]  rd_addr_buff_r[OST_DEPTH-1:0];
reg  [`AXI_LEN_W-1:0]   rd_len_buff_r[OST_DEPTH-1:0]; 
reg  [`AXI_SIZE_W-1:0]  rd_size_buff_r[OST_DEPTH-1:0];
reg  [`AXI_BURST_W-1:0] rd_burst_buff_r[OST_DEPTH-1:0];

// Data buffers  
reg  [`AXI_DATA_W-1:0]  rd_data_buff_r[OST_DEPTH-1:0][MAX_BURST_LEN-1:0];
reg  [BURST_CNT_W-1:0]  rd_data_cnt_r[OST_DEPTH-1:0];
reg  [`AXI_RESP_W-1:0]  rd_resp_buff_r[OST_DEPTH-1:0];

// Address calculation
reg  [`AXI_ADDR_W-1:0]  rd_curr_addr_r[OST_DEPTH-1:0];
reg                     rd_wrap_en_r[OST_DEPTH-1:0];

// Status signals
wire                    rd_buff_set;
wire                    rd_buff_clr; 
wire                    rd_buff_full;
wire                    rd_req_en;
wire                    rd_result_en;
wire [OST_CNT_W-1:0]    rd_req_slot;
wire [OST_CNT_W-1:0]    rd_comp_slot;

//--------------------------------------------------------------------------------
// Pointer Management 
//--------------------------------------------------------------------------------
reg [OST_CNT_W-1:0] rd_req_ptr_r;  
reg [OST_CNT_W-1:0] rd_comp_ptr_r;

assign rd_req_slot  = rd_req_ptr_r;
assign rd_comp_slot = rd_comp_ptr_r;

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_req_ptr_r  <= #DLY 0;
        rd_comp_ptr_r <= #DLY 0;
    end else begin
        rd_req_ptr_r  <= #DLY rd_buff_set ? rd_req_ptr_r + 1 : rd_req_ptr_r;
        rd_comp_ptr_r <= #DLY rd_buff_clr ? rd_comp_ptr_r + 1 : rd_comp_ptr_r;
    end
end

//--------------------------------------------------------------------------------
// Main Control
//--------------------------------------------------------------------------------
assign rd_buff_set = ~rd_buff_full & axi_slv_arvalid & enable;
assign rd_buff_clr = rd_valid_buff_r[rd_comp_slot] & 
                    ~rd_req_buff_r[rd_comp_slot] & 
                    ~rd_comp_buff_r[rd_comp_slot];

reg [OST_DEPTH-1:0] rd_valid_bits;
always @(*) begin
integer i;
    for (i=0; i<OST_DEPTH; i=i+1) 
        rd_valid_bits[i] = rd_valid_buff_r[i];
end
assign rd_buff_full = &rd_valid_bits;

assign rd_req_en    = axi_slv_arvalid & axi_slv_arready;
assign rd_result_en = axi_slv_rvalid & axi_slv_rready;

// Generate for all OST slots
generate
genvar i;
for (i=0; i<OST_DEPTH; i++) begin: OST_SLOTS
    
    // Address calculation
    wire [`AXI_ADDR_W-1:0] rd_start_addr    = rd_addr_buff_r[i];
    wire [`AXI_ADDR_W-1:0] rd_aligned_addr  = (rd_start_addr >> rd_size_buff_r[i]) << rd_size_buff_r[i];
    wire [`AXI_ADDR_W-1:0] rd_wrap_boundary = (rd_start_addr / ((rd_len_buff_r[i]+1) << rd_size_buff_r[i])) * 
                                             ((rd_len_buff_r[i]+1) << rd_size_buff_r[i]);
    wire rd_wrap_en = (rd_curr_addr_r[i] + (1 << rd_size_buff_r[i])) == 
                     (rd_wrap_boundary + ((rd_len_buff_r[i]+1) << rd_size_buff_r[i]));

    // Control FFs
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_valid_buff_r[i] <= #DLY 1'b0;
            rd_req_buff_r[i]   <= #DLY 1'b0;
            rd_comp_buff_r[i]  <= #DLY 1'b0;
        end else begin
            // Valid buffer
            if (rd_buff_set && (rd_req_slot == i))
                rd_valid_buff_r[i] <= #DLY 1'b1;
            else if (rd_buff_clr && (rd_comp_slot == i))
                rd_valid_buff_r[i] <= #DLY 1'b0;
                
            // Request buffer  
            if (rd_buff_set && (rd_req_slot == i))
                rd_req_buff_r[i] <= #DLY 1'b1;
            else if (rd_req_en && (rd_req_slot == i))
                rd_req_buff_r[i] <= #DLY 1'b0;
                
            // Completion buffer
            if (rd_buff_set && (rd_req_slot == i)) 
                rd_comp_buff_r[i] <= #DLY 1'b1;
            else if (rd_result_en && axi_slv_rid == rd_id_buff_r[i] && axi_slv_rlast)
                rd_comp_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Address control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_curr_addr_r[i] <= #DLY 0;
            rd_wrap_en_r[i]   <= #DLY 0;
        end else if (rd_buff_set && (rd_req_slot == i)) begin
            rd_curr_addr_r[i] <= #DLY axi_slv_araddr;
            rd_wrap_en_r[i]   <= #DLY 0;
        end else if (rd_result_en && (axi_slv_rid == rd_id_buff_r[i])) begin
            case (rd_burst_buff_r[i])
                `AXI_BURST_FIXED: rd_curr_addr_r[i] <= #DLY rd_addr_buff_r[i];
                `AXI_BURST_INCR:  rd_curr_addr_r[i] <= #DLY rd_aligned_addr + 
                                                     ((rd_data_cnt_r[i]+1) << rd_size_buff_r[i]);
                `AXI_BURST_WRAP: begin
                    if (rd_wrap_en)
                        rd_curr_addr_r[i] <= #DLY rd_wrap_boundary;
                    else if (rd_wrap_en_r[i])
                        rd_curr_addr_r[i] <= #DLY rd_addr_buff_r[i] + 
                                           ((rd_data_cnt_r[i]+1) << rd_size_buff_r[i]) - 
                                           ((rd_len_buff_r[i]+1) << rd_size_buff_r[i]);
                    else
                        rd_curr_addr_r[i] <= #DLY rd_aligned_addr + 
                                           ((rd_data_cnt_r[i]+1) << rd_size_buff_r[i]);
                end
            endcase
            rd_wrap_en_r[i] <= #DLY rd_wrap_en_r[i] | rd_wrap_en;
        end
    end

    // Payload buffers
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_id_buff_r[i]    <= #DLY 0;
            rd_addr_buff_r[i]  <= #DLY 0;
            rd_len_buff_r[i]   <= #DLY 0;
            rd_size_buff_r[i]  <= #DLY `AXI_SIZE_1B;
            rd_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
        end else if (rd_buff_set && (rd_req_slot == i)) begin
            rd_id_buff_r[i]    <= #DLY axi_slv_arid;
            rd_addr_buff_r[i]  <= #DLY axi_slv_araddr;
            rd_len_buff_r[i]   <= #DLY axi_slv_arlen;
            rd_size_buff_r[i]  <= #DLY axi_slv_arsize;
            rd_burst_buff_r[i] <= #DLY axi_slv_arburst;
        end
    end

    // Data handling
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_data_cnt_r[i] <= #DLY 0;
            rd_resp_buff_r[i] <= #DLY `AXI_RESP_OKAY;
            for (int j=0; j<MAX_BURST_LEN; j++)
                rd_data_buff_r[i][j] <= #DLY 0;
        end else if (rd_buff_set && (rd_req_slot == i)) begin
            rd_data_cnt_r[i] <= #DLY 0;
            rd_resp_buff_r[i] <= #DLY (axi_slv_araddr != REG_ADDR) ? `AXI_RESP_DECERR : `AXI_RESP_OKAY;
        end else if (rd_result_en && (axi_slv_rid == rd_id_buff_r[i])) begin
            rd_data_cnt_r[i] <= #DLY rd_data_cnt_r[i] + 1;
            rd_data_buff_r[i][rd_data_cnt_r[i]] <= #DLY 
                {{`AXI_DATA_W-`AXI_ID_W-`AXI_ADDR_W{1'b0}}, axi_slv_rid, rd_curr_addr_r[i]};
                
            // Error injection
            if ((axi_slv_rid == 'hF) && (rd_data_cnt_r[i] == rd_len_buff_r[i]))
                rd_resp_buff_r[i] <= #DLY `AXI_RESP_SLVERR;
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// Output Logic
//--------------------------------------------------------------------------------
assign axi_slv_arready = ~rd_buff_full;

reg [OST_DEPTH-1:0] rd_comp_bits;
always @(*) begin
integer i;
    for (i=0; i<OST_DEPTH; i=i+1) 
        rd_comp_bits[i] = rd_comp_buff_r[i];
end
assign axi_slv_rvalid  = |(rd_comp_bits);
assign axi_slv_rid     = rd_id_buff_r[rd_comp_slot];
assign axi_slv_rdata   = rd_data_buff_r[rd_comp_slot][rd_data_cnt_r[rd_comp_slot]]; 
assign axi_slv_rresp   = rd_resp_buff_r[rd_comp_slot];
assign axi_slv_rlast   = (rd_data_cnt_r[rd_comp_slot] == rd_len_buff_r[rd_comp_slot]);

endmodule
