`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 25.11.2024 23:50:00
// Design Name: 
// Module Name: Timestamp_Generator_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Testbench for Timestamp_Generator module
// 
// Dependencies: Timestamp_Generator
// 
//////////////////////////////////////////////////////////////////////////////////

module Timestamp_Generator_tb;

    // Testbench parameters
    parameter CLK_DIVIDER = 4;
    parameter TIME_OFFSET = 10;
    
    // Clock period for simulation (50 MHz -> 20 ns clock period)
    parameter CLK_PERIOD = 20;
    
    // Testbench signals
    reg tb_clk;
    reg tb_reset;
    wire [39:0] tb_timestamp;
    
    // Instantiate the DUT (Device Under Test)
    Timestamp_Generator #(
        .CLK_DIVIDER(CLK_DIVIDER),
        .TIME_OFFSET(TIME_OFFSET)
    ) dut (
        .i_clk(tb_clk),
        .i_reset(tb_reset),
        .o_timestamp(tb_timestamp)
    );
    
    // Clock generation
    initial begin
        tb_clk = 0;
        forever #(CLK_PERIOD / 2) tb_clk = ~tb_clk; // Toggle clock every half-period
    end

    // Test sequence
    initial begin
        // Initialize signals
        tb_reset = 0;

        // Apply reset
        #(5 * CLK_PERIOD); // Wait for 5 clock cycles
        tb_reset = 1;

        // Simulation runs for a specific time
        #(500 * CLK_PERIOD); // Simulate for 500 clock cycles
        
        // Finish simulation
        $stop;
    end
    
    // Monitor timestamp output
    initial begin
        $monitor("Time = %t ns, Reset = %b, Timestamp = %d",
                 $time, tb_reset, tb_timestamp);
    end

endmodule
