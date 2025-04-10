module top(
    input wire      sysclk,
    input wire      reset,
    output wire     uart_rx_out,
    
    output wire     IMU1_MOSI,
    output wire     IMU1_SCLK,
    input wire      IMU1_MISO,
    output wire     IMU1_CS,
    input wire      IMU1_interrupt,
    output wire     IMU1_trig,
    output wire     IMU1_fifo_full,
    
    output wire     IMU2_MOSI,
    output wire     IMU2_SCLK,
    input wire      IMU2_MISO,
    output wire     IMU2_CS,
    input wire      IMU2_interrupt,
    output wire     IMU2_trig,
    output wire     IMU2_fifo_full,
    
    output wire     EM_control,
    output wire     EM_status,
    output wire     EM_fifo_full
    );
    
    wire            clk;
    wire [39:0]     w_timestamp;
    
    // IMU 1
    wire [199:0]    w_IMU1_output_sample;
    wire            w_IMU1_sample_valid;
    wire            w_IMU1_fifo_rd_en;
    wire [7:0]      w_IMU1_fifo_dout;
    wire [9:0]      w_IMU1_fifo_data_count;
    wire            w_IMU1_interrupt_synchronised;
    
    // IMU 2
    wire [199:0]    w_IMU2_output_sample;
    wire            w_IMU2_sample_valid;
    wire            w_IMU2_fifo_rd_en;
    wire [7:0]      w_IMU2_fifo_dout;
    wire [9:0]      w_IMU2_fifo_data_count;
    wire            w_IMU2_interrupt_synchronised;
    
    // EM
    wire [199:0]    w_EM_output_datapack;
    wire            w_EM_package_valid;
    wire            w_EM_fifo_rd_en;
    wire [7:0]      w_EM_fifo_dout;
    wire [9:0]      w_EM_fifo_data_count;
    
    // Debug
    wire [7:0]      w_IMU1_TX_Byte;
    wire [7:0]      w_IMU1_RX_Byte;
    wire            w_IMU1_TX_Valid;
    wire            w_IMU1_RX_Valid;
    wire            w_IMU1_TX_Ready;
    wire [5:0]      w_IMU1_current_state;
    wire [7:0]      w_IMU1_bytes_transmitted;
    wire            w_IMU1_timer_IR_flg;
    wire            w_IMU1_TX_enable;
    wire            w_IMU1_custom_trigger;
    wire [7:0]      w_UART_TX_Byte;
    wire            w_TX_DV;
    
    
    // ----------------------------------------------------------- Shared ----------------------------------------------------------------
    // MMCM
    clk_wiz_0 CLK_28MHz (
        // Clock out ports
        .clk_out1(clk),     // output clk_out1
        // Status and control signals
        .resetn(1'b1),      // input resetn
        .locked(),          // output locked
        // Clock in ports
        .clk_in1(sysclk)      // input clk_in1
    );
    
    
    // UART Controller
    UART_Controller
    (
        .i_clk(clk),
        .i_reset(reset),
        .o_TX_out(uart_rx_out),
        
        // IMU 1
        .o_IMU1_fifo_rd_en(w_IMU1_fifo_rd_en),
        .i_IMU1_fifo_dout(w_IMU1_fifo_dout),
        .i_IMU1_fifo_data_count(w_IMU1_fifo_data_count),
        
        // IMU 2
        .o_IMU2_fifo_rd_en(w_IMU2_fifo_rd_en),
        .i_IMU2_fifo_dout(w_IMU2_fifo_dout),
        .i_IMU2_fifo_data_count(w_IMU2_fifo_data_count),
        
        // EM
        .o_EM_fifo_rd_en(w_EM_fifo_rd_en),
        .i_EM_fifo_dout(w_EM_fifo_dout),
        .i_EM_fifo_data_count(w_EM_fifo_data_count),
        
        // Debug
        .o_TX_Byte(w_UART_TX_Byte),
        .o_TX_DV(w_TX_DV)
    );
    
    
    // Timestamp Generator
    Timestamp_Generator #
    (.CLK_DIVIDER(1))  // Default 28
    Timestamp_Generator
    (
        .i_clk(clk),
        .i_reset(reset),
        .o_timestamp(w_timestamp)
    );
     
   
    // -------------------------------------------------------- IMU 1 ---------------------------------------------------------
    // SPI Driver
    IMU_SPI_Driver #(.IMU_ID(8'd1),
                     .TIME_OFFSET(32'd0))  // TIME_OFFSET in main clock domain
    IMU1_Driver
    (
        .i_Clk(clk),
        .i_Rst_L(reset),
        .o_SPI_Clk(IMU1_SCLK),
        .o_SPI_MOSI(IMU1_MOSI),
        .i_SPI_MISO(IMU1_MISO),
        .o_SPI_CS_n(IMU1_CS),
        .o_SM_trigger(IMU1_trig),
        .i_DRDY_synced(w_IMU1_interrupt_synchronised),
        .o_output_sample(w_IMU1_output_sample),
        .o_sample_valid(w_IMU1_sample_valid),
        .i_timestamp(w_timestamp),
        
        // Debug
        .o_TX_Byte(w_IMU1_TX_Byte),
        .o_RX_Byte(w_IMU1_RX_Byte),
        .o_TX_Valid(w_IMU1_TX_Valid),
        .o_RX_Valid(w_IMU1_RX_Valid),
        .o_TX_Ready(w_IMU1_TX_Ready),
        .o_current_state(w_IMU1_current_state),
        .o_bytes_transmitted(w_IMU1_bytes_transmitted),
        .o_timer_IR_flg(w_IMU1_timer_IR_flg),
        .o_TX_enable(w_IMU1_TX_enable),
        .o_custom_trigger(w_IMU1_custom_trigger)
    );
    
    // Fifo Controller
    IMU_Fifo_Controller IMU1_Fifo_Controller
    (
        .i_clk(clk),
        .i_reset(reset),
        .i_imu_sample_package(w_IMU1_output_sample),
        .i_imu_sample_valid(w_IMU1_sample_valid),
        .i_fifo_rd_en(w_IMU1_fifo_rd_en),
        .o_fifo_dout(w_IMU1_fifo_dout),
        .o_fifo_full(IMU1_fifo_full),
        .o_fifo_empty(),
        .o_fifo_data_count(w_IMU1_fifo_data_count)
    );

    // 2FF Synchroniser
    Dual_FF_Synchroniser IMU1_dual_ff_synchroniser (
        .clk        (clk), // Clock input
        .rst_n      (reset), // Active-low asynchronous reset
        .async_in   (IMU1_interrupt), // Asynchronous input signal
        .sync_out   (w_IMU1_interrupt_synchronised)  // Synchronized output signal
    );


    
    // -------------------------------------------------------- IMU 2 ---------------------------------------------------------
    // SPI Driver
    IMU_SPI_Driver #(.IMU_ID(8'd2),
                     .TIME_OFFSET(32'd77000))  // TIME_OFFSET in main clock domain
    IMU2_Driver
    (
        .i_Clk(clk),
        .i_Rst_L(reset),
        .o_SPI_Clk(IMU2_SCLK),
        .o_SPI_MOSI(IMU2_MOSI),
        .i_SPI_MISO(IMU2_MISO),
        .o_SPI_CS_n(IMU2_CS),
        .o_SM_trigger(IMU2_trig),
        .i_DRDY_synced(w_IMU2_interrupt_synchronised),
        .o_output_sample(w_IMU2_output_sample),
        .o_sample_valid(w_IMU2_sample_valid),
        .i_timestamp(w_timestamp)
    );
    
    // Fifo Controller
    IMU_Fifo_Controller IMU2_Fifo_Controller
    (
        .i_clk(clk),
        .i_reset(reset),
        .i_imu_sample_package(w_IMU2_output_sample),
        .i_imu_sample_valid(w_IMU2_sample_valid),
        .i_fifo_rd_en(w_IMU2_fifo_rd_en),
        .o_fifo_dout(w_IMU2_fifo_dout),
        .o_fifo_full(IMU2_fifo_full),
        .o_fifo_empty(),
        .o_fifo_data_count(w_IMU2_fifo_data_count)
    );

    // 2FF Synchroniser
    Dual_FF_Synchroniser IMU2_dual_ff_synchroniser (
        .clk        (clk), // Clock input
        .rst_n      (reset), // Active-low asynchronous reset
        .async_in   (IMU2_interrupt), // Asynchronous input signal
        .sync_out   (w_IMU2_interrupt_synchronised)  // Synchronized output signal
    );
    
    // ------------------------------------------------------------ EM ----------------------------------------------------------------------
    // Instantiation of EM_Controller                  
    EM_Controller #(
        .INITIAL_TIME_OFFSET(32'd0),         // 0 ms
        .TIME_OFFSET_INCREMENT_STEP(32'd2800),      // 0.1 ms
        .PULSE_WIDTH(32'd8960000),   // 320 ms
        .MAX_SYNC_ERROR(14000), // main clock
        .NUMBER_OF_PEAKS(16'd83),      // 7 for 10 ms, 11 for 5 ms, 23 for 2 ms, 43 for 1 ms, 83 for 0.5 ms (must be odd)
        .IDLE_TIME(32'd56000000)     // 1 second
    ) EM_Controller (
        .i_Clk(clk),                     // Clock signal
        .i_Rst_L(reset),                 // Active-low reset signal
        .i_sample_finished(w_IMU1_sample_valid), // Sample finished flag
        .i_timestamp(w_timestamp),
        .o_EM_control(EM_control),        // EM control output signal
        .o_package_valid(w_EM_package_valid),
        .o_output_datapack(w_EM_output_datapack)
    );
    
    // Fifo Controller
    IMU_Fifo_Controller EM_Fifo_Controller
    (
        .i_clk(clk),
        .i_reset(reset),
        .i_imu_sample_package(w_EM_output_datapack),
        .i_imu_sample_valid(w_EM_package_valid),
        .i_fifo_rd_en(w_EM_fifo_rd_en),
        .o_fifo_dout(w_EM_fifo_dout),
        .o_fifo_full(EM_fifo_full),
        .o_fifo_empty(),
        .o_fifo_data_count(w_EM_fifo_data_count)
    );
    
    assign EM_status = EM_control;

    // ------------------------------------------------------------ Debug --------------------------------------------------------------
    // ILA
    ila_0 ILA (
        .clk(clk), // input wire clk
        .probe0(w_IMU1_TX_Byte),   // Transmitted byte
        .probe1(w_IMU1_RX_Byte),   // Received byte
        .probe2(reset),       // Reset signal
        .probe3(IMU1_MOSI),        // SPI MOSI
        .probe4(IMU1_MISO),        // SPI MISO
        .probe5(IMU1_SCLK),     // SPI Clock
        .probe6(w_IMU1_TX_Ready),  // Transmit ready
        .probe7(w_IMU1_RX_Valid),  // Receive valid
        .probe8(w_IMU1_TX_Valid),   // Transmit valid
        .probe9(w_IMU1_current_state), // input wire [5:0]  probe9 
	    .probe10(IMU1_trig), // wire probe10 
	    .probe11(w_IMU1_bytes_transmitted), // input wire [7:0]  probe11 
	    .probe12(w_IMU1_timer_IR_flg), // input wire [0:0]  probe12
	    .probe13(w_IMU1_TX_enable),
	    .probe14(w_IMU1_custom_trigger),
	    .probe15(IMU1_CS),
	    .probe16(w_EM_output_datapack),
	    .probe17(w_EM_package_valid),
	    .probe18(uart_rx_out),
	    .probe19(w_IMU1_fifo_data_count),
	    .probe20(w_IMU1_fifo_dout),  // fifo_out
	    .probe21(w_IMU1_fifo_rd_en),  // fifo_rd_en
	    .probe22(w_UART_TX_Byte),  // TX_byte
	    .probe23(w_TX_DV),  // TX_DV
	    .probe24(w_IMU1_interrupt_synchronised),
	    .probe25(w_IMU1_output_sample[47:0])
    );
    
endmodule
