`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 25.11.2024 23:43:58
// Design Name: 
// Module Name: Timestamp_Generator
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


module Timestamp_Generator
    #(parameter CLK_DIVIDER = 28)
    (
        input wire i_clk,
        input wire i_reset,
        output wire [39:0] o_timestamp
    );
    
    reg [39:0] r_timestamp = 40'd0;
    
    reg [$clog2(CLK_DIVIDER + 1) - 1:0] clk_divider_counter;
    
    always @(posedge i_clk or negedge i_reset) begin
        if (~i_reset) begin
            r_timestamp <= 40'd0;
            clk_divider_counter <= 0;
        end
        else begin
            if (clk_divider_counter == CLK_DIVIDER - 1) begin
                clk_divider_counter <= 0;
                r_timestamp <= r_timestamp + 1;
            end
            else begin
                clk_divider_counter <= clk_divider_counter + 1;
            end
        end
        
    end
    
    assign o_timestamp = r_timestamp;
endmodule
