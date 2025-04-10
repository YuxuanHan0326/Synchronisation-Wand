`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 26.11.2024 21:26:14
// Design Name: 
// Module Name: testing_top_fifo_to_TX
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module testing_top_fifo_to_TX(
        input wire clk,
        input wire reset,
        input wire [199:0] i_package,
        input wire i_sample_valid,
        output wire o_TX_out        
    );
    
    wire w_rd_en_IMU1;
    wire [7:0] w_dout_IMU1;
    wire [9:0] w_data_count_IMU1;
    
    UART_Controller u_uart_controller (
        .i_clk(clk),                       // Clock signal
        .i_reset(reset),                   // Reset signal
        .o_TX_out(o_TX_out),                 // UART TX output
        .o_IMU1_fifo_rd_en(w_rd_en_IMU1), // IMU1 FIFO read enable
        .i_IMU1_fifo_dout(w_dout_IMU1), // IMU1 FIFO data output
        .i_IMU1_fifo_data_count(w_data_count_IMU1) // IMU1 FIFO data count
    );
    
    IMU_Fifo_Controller u_imu_fifo_controller (
        .i_clk(clk),                        // Clock signal
        .i_reset(reset),                    // Reset signal
        .i_imu_sample_package(i_package), // 200-bit IMU sample package
        .i_imu_sample_valid(i_sample_valid),     // IMU sample valid signal
        .i_fifo_rd_en(w_rd_en_IMU1),          // FIFO read enable signal
        .o_fifo_dout(w_dout_IMU1),            // FIFO data output
        .o_fifo_full(),            // FIFO full flag
        .o_fifo_empty(),          // FIFO empty flag
        .o_fifo_data_count(w_data_count_IMU1) // FIFO data count
    );
endmodule
