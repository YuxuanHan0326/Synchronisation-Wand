`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 26.11.2024
// Design Name: 
// Module Name: testing_top_fifo_to_TX_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Testbench for testing_top_fifo_to_TX with IMU sample loading
// 
//////////////////////////////////////////////////////////////////////////////////

module testing_top_fifo_to_TX_tb;

    // Testbench parameters
    parameter CLK_PERIOD = 0.3; // 100 MHz clock (10 ns period)
    
    // Testbench signals
    reg tb_clk;
    reg tb_reset;
    reg [199:0] tb_i_package;
    reg tb_i_sample_valid;
    wire tb_o_TX_out;

    // Internal signal monitoring for `testing_top_fifo_to_TX`
    wire tb_w_rd_en_IMU1;
    wire [7:0] tb_w_dout_IMU1;
    wire [9:0] tb_w_data_count_IMU1;

    wire [7:0] tb_r_TX_Byte;
    wire tb_r_TX_DV;
    wire tb_w_TX_Active;
    wire tb_w_TX_Done;

    wire [199:0] tb_r_imu_sample_package;
    wire [4:0] tb_r_byte_counter;
    wire [7:0] tb_w_fifo_din;
    wire tb_r_fifo_wr_en;
    wire tb_r_current_state;

    // Instantiate the DUT (Device Under Test)
    testing_top_fifo_to_TX dut (
        .clk(tb_clk),
        .reset(tb_reset),
        .i_package(tb_i_package),
        .i_sample_valid(tb_i_sample_valid),
        .o_TX_out(tb_o_TX_out)
    );

    // Internal signal connections for monitoring
    assign tb_w_rd_en_IMU1 = dut.w_rd_en_IMU1;
    assign tb_w_dout_IMU1 = dut.w_dout_IMU1;
    assign tb_w_data_count_IMU1 = dut.w_data_count_IMU1;

    // Signals from `UART_Controller`
    assign tb_r_TX_Byte = dut.u_uart_controller.r_TX_Byte;
    assign tb_r_TX_DV = dut.u_uart_controller.r_TX_DV;
    assign tb_w_TX_Active = dut.u_uart_controller.w_TX_Active;
    assign tb_w_TX_Done = dut.u_uart_controller.w_TX_Done;

    // Signals from `IMU_Fifo_Controller`
    assign tb_r_imu_sample_package = dut.u_imu_fifo_controller.r_imu_sample_package;
    assign tb_r_byte_counter = dut.u_imu_fifo_controller.r_byte_counter;
    assign tb_w_fifo_din = dut.u_imu_fifo_controller.w_fifo_din;
    assign tb_r_fifo_wr_en = dut.u_imu_fifo_controller.r_fifo_wr_en;
    assign tb_r_current_state = dut.u_imu_fifo_controller.r_current_state;

    // Clock generation
    initial begin
        tb_clk = 0;
        forever #(CLK_PERIOD / 2) tb_clk = ~tb_clk; // Toggle clock every half-period
    end

    // Test sequence
    initial begin
        // Initialize signals
        tb_reset = 0;
        tb_i_package = 200'b0;
        tb_i_sample_valid = 0;

        // Apply reset
        #(5 * CLK_PERIOD);
        tb_reset = 1;

        // Send a valid IMU sample
        #(10 * CLK_PERIOD);
        tb_i_package = 200'h2c0100004ac9be0680fff2049afffeffe5fffffff3fe900096;
        tb_i_sample_valid = 1;
        #(CLK_PERIOD);
        tb_i_sample_valid = 0;

        // Wait for the sample to be processed and written to FIFO
        #(700 * CLK_PERIOD);

        // Send another IMU sample
        tb_i_package = 200'h11223344556677889900AABBCCDDEEFFAABBCCDDEE;
        tb_i_sample_valid = 1;
        #(CLK_PERIOD);
        tb_i_sample_valid = 0;

        // Wait for the second sample to be processed
        #(500 * CLK_PERIOD);

        // Finish simulation
        $stop;
    end

    // Monitor Outputs and Internal States
    initial begin
        $monitor(
            "Time: %t | TX Byte: 0x%h | TX DV: %b | TX Active: %b | TX Done: %b | IMU Package: 0x%h | Byte Counter: %d | FIFO Data: 0x%h | FIFO WR_EN: %b | State: %d | Data Count: %d",
            $time, tb_r_TX_Byte, tb_r_TX_DV, tb_w_TX_Active, tb_w_TX_Done, tb_r_imu_sample_package,
            tb_r_byte_counter, tb_w_fifo_din, tb_r_fifo_wr_en, tb_r_current_state, tb_w_data_count_IMU1
        );
    end

endmodule
