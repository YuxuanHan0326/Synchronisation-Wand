`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/02/03 12:10:02
// Design Name: 
// Module Name: EM_Controller
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


module EM_Controller
#(
    parameter           INITIAL_TIME_OFFSET = 32'd0,
    parameter           TIME_OFFSET_INCREMENT_STEP = 32'd14000,      // Default 0.5 ms
    parameter           PULSE_WIDTH = 32'd8960000,
    parameter           MAX_SYNC_ERROR = 32'd280000,    // 10 ms
    parameter           NUMBER_OF_PEAKS = 16'd7,
    parameter           IDLE_TIME = 32'd56000000        // 1 second in the front and behind
)
(
    input wire          i_Clk,
    input wire          i_Rst_L,
    input wire          i_sample_finished,
    input wire [39:0]   i_timestamp,
    output wire         o_EM_control,
    output wire         o_package_valid,
    output wire [199:0] o_output_datapack
);

    reg         r_EM_control = 1'b0;
    reg         r_timer_IR_flg = 1'b1;
    reg [31:0]  r_delay_counter = 32'd0;
    reg [31:0]  r_timer_autoreload_value = IDLE_TIME;
    reg [15:0]  r_pulse_count = 16'd0;
    reg         r_package_valid;
    reg [199:0] r_output_datapack;
    reg [39:0]  r_timestamp_syncevent;
    reg [31:0]  r_time_offset = INITIAL_TIME_OFFSET;
    
    localparam IMU_SAMPLING_PERIOD = 32'd1120000;       // 40 ms
    
    // FSM state encoding
    localparam  DELAY_INITIAL = 4'd0,
                READY = 4'd1,
                SET_PHASE = 4'd2,
                INITIAL_PULSE = 4'd3,
                SUCCESSIVE_PULSES = 4'd4,
                DELAY_POST = 4'd5;
    
    reg [3:0]   r_current_state = DELAY_INITIAL;
    
    // Main FSM
    always @ (posedge i_Clk or negedge i_Rst_L) begin
        if (~i_Rst_L) begin
            r_current_state <= DELAY_INITIAL;
            r_EM_control <= 1'b0;
            r_timer_IR_flg <= 1'b0;
            r_delay_counter <= 32'd0;
            r_timer_autoreload_value <= IDLE_TIME;
            r_pulse_count <= 16'd0;
            r_package_valid <= 1'b0;
            r_time_offset <= INITIAL_TIME_OFFSET; 
        end else begin
        
            // Timer logic
            if (~r_timer_IR_flg) begin
                if (r_delay_counter < r_timer_autoreload_value) begin
                    r_delay_counter <= r_delay_counter + 1;
                end else begin
                    r_timer_IR_flg <= 1;       // Set timer flag
                    r_delay_counter <= 32'd0; // Reset delay counter
                end
            end
            
            // FSM states
            case (r_current_state)
            
                DELAY_INITIAL : begin
                    r_package_valid <= 1'b0;
                    if (r_timer_IR_flg) begin
                        r_current_state <= READY;
                    end else begin
                        r_current_state <= DELAY_INITIAL;
                    end
                end
                
                READY : begin
                    if (i_sample_finished) begin
                        r_current_state <= SET_PHASE;
                        r_timer_autoreload_value <= IMU_SAMPLING_PERIOD - r_time_offset - 2;  // Minus 2 extra clock due to fsm
                        r_timer_IR_flg <= 0;
                    end else begin
                        r_current_state <= READY;
                    end
                end
                
                SET_PHASE : begin
                    if (r_timer_IR_flg) begin
                        r_current_state <= INITIAL_PULSE;
                        r_timer_autoreload_value <= PULSE_WIDTH - 2;
                        r_timer_IR_flg <= 0;
                        r_EM_control <= 1;
                        r_timestamp_syncevent <= i_timestamp;   // Record timestamp for syncevent
                    end else begin
                        r_current_state <= SET_PHASE;
                    end
                end
                
                INITIAL_PULSE : begin
                    if (r_timer_IR_flg) begin
                        r_current_state <= SUCCESSIVE_PULSES;
                        r_EM_control <= ~r_EM_control;
                        r_timer_autoreload_value <= PULSE_WIDTH + MAX_SYNC_ERROR - 2;
                        r_timer_IR_flg <= 0;
                    end else begin
                        r_current_state <= INITIAL_PULSE;
                    end
                end
                
                SUCCESSIVE_PULSES : begin
                    if (r_timer_IR_flg) begin
                        if (r_pulse_count < NUMBER_OF_PEAKS - 1) begin
                            r_EM_control <= ~r_EM_control;
                            r_current_state <= SUCCESSIVE_PULSES;
                            r_timer_IR_flg <= 0;
                            r_pulse_count <= r_pulse_count + 1;
                        end else begin
                            r_pulse_count <= 16'd0;
                            r_current_state <= DELAY_POST;
                            r_timer_autoreload_value <= IDLE_TIME;
                            r_timer_IR_flg <= 0;
                        end
                    end else begin
                        r_current_state <= SUCCESSIVE_PULSES;
                    end
                end
                
                DELAY_POST : begin
                    if (r_timer_IR_flg) begin
                        r_current_state <= DELAY_INITIAL;
                        r_timer_IR_flg <= 0;
                        r_output_datapack[199:72] <= {8'h2C, 8'hFF, i_timestamp, r_timestamp_syncevent, r_time_offset};
                        r_output_datapack[31:0] <= 32'd0;
                        r_package_valid <= 1'b1;

                        // Avoid overflow
                        if (r_time_offset >= (IMU_SAMPLING_PERIOD - 2 - TIME_OFFSET_INCREMENT_STEP)) begin
                            r_time_offset <= r_time_offset + TIME_OFFSET_INCREMENT_STEP - IMU_SAMPLING_PERIOD;
                        end else begin
                            r_time_offset <= r_time_offset + TIME_OFFSET_INCREMENT_STEP;    // Increment time offset
                        end

                    end else begin
                        r_current_state <= DELAY_POST;
                    end
                end
                
            endcase
        end
    end
    
    // Assign outputs
    assign o_EM_control = r_EM_control;
    assign o_output_datapack = r_output_datapack;
    assign o_package_valid = r_package_valid;
    
endmodule
