`timescale 1ns / 1ps

module EM_Controller_tb;

    // Testbench signals
    reg i_Clk;
    reg i_Rst_L;
    reg i_sample_finished;
    wire o_EM_control;
    
    // Clock generation parameters
    localparam CLK_PERIOD = 2;
    
    // Instantiate DUT (Device Under Test)
    EM_Controller #(
        .TIME_OFFSET(32'd1119990),
        .PULSE_WIDTH(32'd10),
        .MAX_SYNC_ERROR(32'd10),
        .NUMBER_OF_PEAKS(8'd7),
        .IDLE_TIME(32'd5)
    ) DUT (
        .i_Clk(i_Clk),
        .i_Rst_L(i_Rst_L),
        .i_sample_finished(i_sample_finished),
        .o_EM_control(o_EM_control)
    );
    
    // Clock generation
    always begin
        #(CLK_PERIOD / 2) i_Clk = ~i_Clk;
    end
    
    // Test sequence
    initial begin
        // Initialize inputs
        i_Clk = 0;
        i_Rst_L = 0;
        i_sample_finished = 0;
        
        // Apply reset
        #1;
        i_Rst_L = 1;
        
        // Wait for some time then trigger sample finished signal
        #30;  // Simulating delay for FSM transitions
        i_sample_finished = 1;
        #4;
        i_sample_finished = 0;
        
        // Wait and observe FSM behavior
        #500;
        
        i_sample_finished = 1;
        #1;
        i_sample_finished = 0;
        
        // Run simulation for more time
        #5000;
        
        // End simulation
        $stop;
    end
    
endmodule
