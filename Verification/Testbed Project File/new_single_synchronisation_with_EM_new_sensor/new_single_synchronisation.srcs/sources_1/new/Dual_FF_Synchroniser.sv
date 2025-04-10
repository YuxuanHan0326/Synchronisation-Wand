`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/02/23 02:16:06
// Design Name: 
// Module Name: Dual_FF_Synchroniser
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


module Dual_FF_Synchroniser (
    input logic clk,          // Clock input
    input logic rst_n,        // Active-low asynchronous reset
    input logic async_in,     // Asynchronous input signal
    output logic sync_out     // Synchronized output signal
);

    // Two flip-flops for synchronization
    (* ASYNC_REG = "true" *) logic ff1, ff2;

    // Synchronization logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ff1 <= 1'b0;  // Reset both flip-flops to zero
            ff2 <= 1'b0;
        end else begin
            ff1 <= async_in;  // First flip-flop captures the async input
            ff2 <= ff1;       // Second flip-flop stabilizes the signal
        end
    end

    // Output from the second flip-flop
    assign sync_out = ff2;

endmodule

