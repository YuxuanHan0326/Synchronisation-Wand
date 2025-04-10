`timescale 1ns / 1ps

module tb_IMU_SPI_Driver;

  // Clock and Reset
  reg clk;
  reg rst_n;

  // SPI Signals
  wire spi_clk;
  wire spi_mosi;
  wire spi_miso;  // Connected to MOSI for loopback testing
  wire spi_cs_n;

  // Control Signals
  reg i_DRDY_synced;
  wire o_SM_trigger;

  // Timestamp
  reg [39:0] i_timestamp;

  // Outputs from the DUT
  wire [199:0] o_output_sample;
  wire o_sample_valid;

  // Debug Outputs
  wire [7:0] o_TX_Byte, o_RX_Byte;
  wire o_TX_Valid, o_RX_Valid;
  wire o_TX_Ready;
  wire [5:0] o_current_state;
  wire [7:0] o_bytes_transmitted;
  wire o_timer_IR_flg, o_TX_enable, o_custom_trigger;

  // Instantiate the IMU_SPI_Driver
  IMU_SPI_Driver #(
      .IMU_ID(8'd1),
      .TIME_OFFSET(32'd100)
  ) uut (
      .i_Clk(clk),
      .i_Rst_L(rst_n),
      .o_SPI_Clk(spi_clk),
      .o_SPI_MOSI(spi_mosi),
      .i_SPI_MISO(spi_miso),  // Loopback connection
      .o_SPI_CS_n(spi_cs_n),
      .o_SM_trigger(o_SM_trigger),
      .i_DRDY_synced(i_DRDY_synced),
      .o_output_sample(o_output_sample),
      .o_sample_valid(o_sample_valid),
      .i_timestamp(i_timestamp),
      .o_TX_Byte(o_TX_Byte),
      .o_RX_Byte(o_RX_Byte),
      .o_TX_Valid(o_TX_Valid),
      .o_RX_Valid(o_RX_Valid),
      .o_TX_Ready(o_TX_Ready),
      .o_current_state(o_current_state),
      .o_bytes_transmitted(o_bytes_transmitted),
      .o_timer_IR_flg(o_timer_IR_flg),
      .o_TX_enable(o_TX_enable),
      .o_custom_trigger(o_custom_trigger)
  );

  // Connect MOSI to MISO for loopback testing
  assign spi_miso = spi_mosi;

  // Clock Generation - 28 MHz
  initial begin
    clk = 0;
    i_timestamp = 40'd0;
    forever #0.1 clk = ~clk; // Half-period for 28 MHz clock (35.71 ns period)
  end

  // Reset and Stimulus Generation
  initial begin
    // Initial values
    rst_n = 0;
    i_DRDY_synced = 0;
    i_timestamp = 0;

    // Apply reset
    #1;
    rst_n = 1;

    // Wait for a few clock cycles
    #200;
    i_DRDY_synced = 1;
    #1;
    i_DRDY_synced = 0;

    #200;
    i_DRDY_synced = 1;
    #1;
    i_DRDY_synced = 0;

    #10000000;

    $finish;
  end

  always @(posedge clk) begin
    i_timestamp <= i_timestamp + 1;
  end

  // Monitoring Outputs
  initial begin
    $display("Time\tState\tSample Valid\tSPI CS\tSPI CLK\tTX Byte\tRX Byte\tBytes Sent");
    forever begin
      @(posedge clk);
      $display("%0t\t%0d\t%b\t%b\t%b\t%h\t%h\t%d",
               $time, o_current_state, o_sample_valid, spi_cs_n, spi_clk, o_TX_Byte, o_RX_Byte, o_bytes_transmitted);
    end
  end

endmodule
