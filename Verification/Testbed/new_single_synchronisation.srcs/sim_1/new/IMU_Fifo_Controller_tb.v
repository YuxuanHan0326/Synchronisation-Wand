`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 26.11.2024
// Design Name: 
// Module Name: IMU_Fifo_Controller_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Testbench for IMU_Fifo_Controller with FIFO write and read verification
// 
//////////////////////////////////////////////////////////////////////////////////

module IMU_Fifo_Controller_tb;

    // Testbench parameters
    parameter CLK_PERIOD = 2; // 100 MHz clock (10 ns period)
    
    // Testbench signals
    reg tb_clk;
    reg tb_reset;
    reg [199:0] tb_imu_sample_package;
    reg tb_imu_sample_valid;
    reg tb_fifo_rd_en;
    wire [7:0] tb_fifo_dout;
    wire tb_fifo_full;
    wire tb_fifo_empty;
    wire [9:0] tb_fifo_data_count;

    // Internal signal monitoring
    wire [199:0] tb_r_imu_sample_package;
    wire [4:0] tb_r_byte_counter;
    wire [7:0] tb_w_fifo_din;
    wire tb_r_fifo_wr_en;
    wire tb_r_current_state;

    // Instantiate the DUT (Device Under Test)
    IMU_Fifo_Controller dut (
        .i_clk(tb_clk),
        .i_reset(tb_reset),
        .i_imu_sample_package(tb_imu_sample_package),
        .i_imu_sample_valid(tb_imu_sample_valid),
        .i_fifo_rd_en(tb_fifo_rd_en),
        .o_fifo_dout(tb_fifo_dout),
        .o_fifo_full(tb_fifo_full),
        .o_fifo_empty(tb_fifo_empty),
        .o_fifo_data_count(tb_fifo_data_count)
    );

    // Internal signal connections
    assign tb_r_imu_sample_package = dut.r_imu_sample_package;
    assign tb_r_byte_counter = dut.r_byte_counter;
    assign tb_w_fifo_din = dut.w_fifo_din;
    assign tb_r_fifo_wr_en = dut.r_fifo_wr_en;
    assign tb_r_current_state = dut.r_current_state;

    // Clock generation
    initial begin
        tb_clk = 0;
        forever #(CLK_PERIOD / 2) tb_clk = ~tb_clk; // Toggle clock every half-period
    end

    // Test sequence
    initial begin
        // Initialize signals
        tb_reset = 0;
        tb_imu_sample_package = 200'b0;
        tb_imu_sample_valid = 0;
        tb_fifo_rd_en = 0;

        // Apply reset
        #(5 * CLK_PERIOD);
        tb_reset = 1;

        // Send a valid IMU sample
        #(10 * CLK_PERIOD);
        tb_imu_sample_package = 200'h2c0100004ac9be0680fff2049afffeffe5fffffff3fe900096;
        tb_imu_sample_valid = 1;
        #(CLK_PERIOD);
        tb_imu_sample_valid = 0;

        // Wait for the sample to be processed and written to FIFO
        #(100 * CLK_PERIOD - CLK_PERIOD / 2);

        // Read the FIFO to verify data
        repeat (25) begin
            if (!tb_fifo_empty) begin
                tb_fifo_rd_en = 1; // Enable FIFO read
                #(CLK_PERIOD);
                tb_fifo_rd_en = 0; // Deassert read enable
                #(CLK_PERIOD);
            end
        end

        // Send another sample
        tb_imu_sample_package = 200'h11223344556677889900AABBCCDDEEFFAABBCCDDEE;
        tb_imu_sample_valid = 1;
        #(CLK_PERIOD);
        tb_imu_sample_valid = 0;

        // Wait for the second sample to be processed and written to FIFO
        #(100 * CLK_PERIOD);

        // Read the FIFO to verify the second data sample
        repeat (25) begin
            if (!tb_fifo_empty) begin
                tb_fifo_rd_en = 1; // Enable FIFO read
                #(CLK_PERIOD);
                tb_fifo_rd_en = 0; // Deassert read enable
                #(CLK_PERIOD);
            end
        end

        // Finish simulation
        $stop;
    end

    // Monitor Outputs and Internal States
    initial begin
        $monitor(
            "Time: %t | State: %d | Byte Counter: %d | FIFO Data: 0x%h | FIFO WR_EN: %b | IMU Sample: 0x%h | FIFO Count: %d | FIFO Full: %b | FIFO Empty: %b",
            $time, tb_r_current_state, tb_r_byte_counter, tb_fifo_dout, tb_r_fifo_wr_en, tb_r_imu_sample_package,
            tb_fifo_data_count, tb_fifo_full, tb_fifo_empty
        );
    end

endmodule
