`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 26.11.2024 02:00:00
// Design Name: 
// Module Name: UART_Controller_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Enhanced Testbench for UART_Controller module
// 
// Dependencies: UART_Controller
// 
//////////////////////////////////////////////////////////////////////////////////

module UART_Controller_tb;

    // Parameters
    parameter CLK_PERIOD = 10;  // Clock period in ns (100 MHz)
    
    // Testbench signals
    reg tb_clk;
    reg tb_reset;
    wire tb_TX_out;
    wire tb_TX_Active;
    wire tb_TX_Done;
    wire [7:0] tb_TX_Byte;
    wire tb_TX_DV;
    wire [3:0] tb_current_state;

    // Instantiate the DUT (Device Under Test)
    UART_Controller dut (
        .i_clk(tb_clk),
        .i_reset(tb_reset),
        .o_TX_out(tb_TX_out)
    );

    // Internal signal connections (for monitoring)
    assign tb_TX_Active = dut.w_TX_Active;
    assign tb_TX_Done = dut.w_TX_Done;
    assign tb_TX_Byte = dut.r_TX_Byte;
    assign tb_TX_DV = dut.r_TX_DV;
    assign tb_current_state = dut.current_state;
    
    // Clock generation
    initial begin
        tb_clk = 0;
        forever #(CLK_PERIOD / 2) tb_clk = ~tb_clk;  // Toggle clock every half-period
    end

    // Test sequence
    initial begin
        // Initialize signals
        tb_reset = 0;

        // Apply reset
        #(5 * CLK_PERIOD);  // Wait for 5 clock cycles
        tb_reset = 1;

        // Wait for simulation to run
        #(200000 * CLK_PERIOD);  // Run for a sufficient time to observe UART transmission
        
        // Finish simulation
        $stop;
    end

    // Monitor Outputs and Internal States
    initial begin
        $monitor(
            "Time: %t | Reset: %b | TX_out: %b | TX_Active: %b | TX_Done: %b | TX_Byte: 0x%h | TX_DV: %b",
            $time, tb_reset, tb_TX_out, tb_TX_Active, tb_TX_Done, tb_TX_Byte, tb_TX_DV
        );
    end

endmodule
