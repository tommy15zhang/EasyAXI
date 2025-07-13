// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2022 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : tb_rvseed.v
// Author        : Rongye
// Created On    : 2022-03-25 04:18
// Last Modified : 2025-02-08 06:48
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------
`timescale 1ns / 1ps

module TESTBENCH ();

reg                  clk;
reg                  rst_n;
reg                  enable;

localparam SIM_PERIOD = 20; // 20ns -> 50MHz

integer k;
initial begin
    #(SIM_PERIOD/2);
    clk = 1'b0;
    reset;
    // $finish;
end

initial begin
    #(SIM_PERIOD * 1000);
    $display("Time Out");
    $finish;
end

always #(SIM_PERIOD/2) clk = ~clk;

task reset;                // reset 1 clock
    begin
        enable = 0; 
        rst_n = 0; 
        #(SIM_PERIOD * 1);
        rst_n = 1;
        #(SIM_PERIOD * 5 + 1);
        enable = 1; 
    end
endtask

EASYAXI_TOP U_EASYAXI_TOP (
    .clk                            ( clk                           ),
    .rst_n                          ( rst_n                         ),
    .enable                         ( enable                        )
);

// vcs 
initial begin
   // $fsdbDumpfile("sim_out.fsdb");
   // $fsdbDumpvars("+all");
    $dumpfile("dump.fst");      // 任意名字，后缀可写为 .fst/.vcd
    $dumpvars(0, TESTBENCH);    // 从顶层 dump 所有信号
end

endmodule
