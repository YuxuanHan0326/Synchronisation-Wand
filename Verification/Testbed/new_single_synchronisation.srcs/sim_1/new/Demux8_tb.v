`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 26.11.2024
// Design Name: 
// Module Name: AntiMux8_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Testbench for AntiMux8
// 
//////////////////////////////////////////////////////////////////////////////////

module Demux8_tb;

    // Parameters
    parameter DATA_WIDTH = 32;
    parameter SELECT_SIZE = 3;

    // Testbench Signals
    reg [DATA_WIDTH-1:0] tb_data_i;
    reg [SELECT_SIZE-1:0] tb_select_i;
    wire [DATA_WIDTH-1:0] tb_data0_o;
    wire [DATA_WIDTH-1:0] tb_data1_o;
    wire [DATA_WIDTH-1:0] tb_data2_o;
    wire [DATA_WIDTH-1:0] tb_data3_o;
    wire [DATA_WIDTH-1:0] tb_data4_o;
    wire [DATA_WIDTH-1:0] tb_data5_o;
    wire [DATA_WIDTH-1:0] tb_data6_o;
    wire [DATA_WIDTH-1:0] tb_data7_o;

    // Instantiate the AntiMux8 module
    Demux8 #(
        .DATA_WIDTH(DATA_WIDTH),
        .SELECT_SIZE(SELECT_SIZE)
    ) uut (
        .data_i(tb_data_i),
        .select_i(tb_select_i),
        .data0_o(tb_data0_o),
        .data1_o(tb_data1_o),
        .data2_o(tb_data2_o),
        .data3_o(tb_data3_o),
        .data4_o(tb_data4_o),
        .data5_o(tb_data5_o),
        .data6_o(tb_data6_o),
        .data7_o(tb_data7_o)
    );

    // Clock generation (optional, only if needed for sequential signals)
    initial begin
        // Stimulus for the AntiMux8 module
        tb_data_i = 32'h00000000;
        tb_select_i = 3'b000;

        // Test case 1: Select output 0
        #10 tb_data_i = 32'h12345678;
        tb_select_i = 3'b000;

        // Test case 2: Select output 1
        #10 tb_data_i = 32'h87654321;
        tb_select_i = 3'b001;

        // Test case 3: Select output 2
        #10 tb_data_i = 32'hAABBCCDD;
        tb_select_i = 3'b010;

        // Test case 4: Select output 3
        #10 tb_data_i = 32'hDEADBEEF;
        tb_select_i = 3'b011;

        // Test case 5: Select output 4
        #10 tb_data_i = 32'hCAFEBABE;
        tb_select_i = 3'b100;

        // Test case 6: Select output 5
        #10 tb_data_i = 32'hFACEFEED;
        tb_select_i = 3'b101;

        // Test case 7: Select output 6
        #10 tb_data_i = 32'hB16B00B5;
        tb_select_i = 3'b110;

        // Test case 8: Select output 7
        #10 tb_data_i = 32'hFEEDC0DE;
        tb_select_i = 3'b111;

        // Test case 9: Invalid select (default case)
        #10 tb_select_i = 3'bxxx;

        // Finish simulation
        #10 $stop;
    end

    // Monitor outputs
    initial begin
        $monitor(
            "Time: %0t | Select: %b | Data In: %h | Data Out: {0: %h, 1: %h, 2: %h, 3: %h, 4: %h, 5: %h, 6: %h, 7: %h}",
            $time, tb_select_i, tb_data_i, 
            tb_data0_o, tb_data1_o, tb_data2_o, tb_data3_o,
            tb_data4_o, tb_data5_o, tb_data6_o, tb_data7_o
        );
    end

endmodule
