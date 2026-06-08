`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 26.11.2024 15:04:21
// Design Name: 
// Module Name: IMU_Fifo_Controller
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


module IMU_Fifo_Controller(
    input wire          i_clk,
    input wire          i_reset,
    
    input wire  [199:0] i_imu_sample_package,
    input wire          i_imu_sample_valid,
    
    input wire          i_fifo_rd_en,
    output wire [7:0]   o_fifo_dout,
    
    output wire         o_fifo_full,
    output wire         o_fifo_empty,
    output wire [9:0]   o_fifo_data_count
    );
    
    localparam  AWAITING_SAMPLE         = 0,
                ACTIVE                  = 1;
    
    
    reg  [199:0] r_imu_sample_package    = 200'b0;
    reg  [4:0]   r_byte_counter          = 5'd0;
    wire  [7:0]  w_fifo_din;
    reg          r_fifo_wr_en            = 0;
    
    reg          r_current_state         = AWAITING_SAMPLE;
    
    always @(posedge i_clk or negedge i_reset) begin
        if (~i_reset) begin
            r_current_state <= AWAITING_SAMPLE;
            r_byte_counter <= 5'd0;
        end
        else begin
            case (r_current_state)
            
                AWAITING_SAMPLE: begin
                    r_byte_counter <= 5'd0;
                    if (i_imu_sample_valid) begin
                        r_imu_sample_package <= i_imu_sample_package;
                        r_current_state <= ACTIVE;
                        r_fifo_wr_en <= 1;
                    end
                end
                
                ACTIVE: begin
                    if (r_byte_counter != 5'd24) begin
                        r_byte_counter <= r_byte_counter + 1;
                        r_imu_sample_package <= r_imu_sample_package << 8;
                    end
                    else begin
                        r_current_state <= AWAITING_SAMPLE;
                        r_fifo_wr_en <= 0;
                    end
                end
                
            endcase
        end
    end
    
    assign w_fifo_din = r_imu_sample_package [199:192];
    
    IMU_fifo fifo_1024 (
        .clk(i_clk),                // input wire clk
        .srst(~i_reset),              // input wire srst
        .din(w_fifo_din),                // input wire [7 : 0] din
        .wr_en(r_fifo_wr_en),            // input wire wr_en
        .rd_en(i_fifo_rd_en),            // input wire rd_en
        .dout(o_fifo_dout),              // output wire [7 : 0] dout
        .full(o_fifo_full),              // output wire full
        .empty(o_fifo_empty),            // output wire empty
        .data_count(o_fifo_data_count)  // output wire [9 : 0] data_count
    );
endmodule
