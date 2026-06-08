`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 26.11.2024 01:33:47
// Design Name: 
// Module Name: UART_Controller
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


module UART_Controller(
    input wire          i_clk,
    input wire          i_reset,
    output wire         o_TX_out,
    
    // IMU 1
    output wire         o_IMU1_fifo_rd_en,
    input wire [7:0]    i_IMU1_fifo_dout,
    input wire [9:0]    i_IMU1_fifo_data_count,
    
    // IMU 2
    output wire         o_IMU2_fifo_rd_en,
    input wire [7:0]    i_IMU2_fifo_dout,
    input wire [9:0]    i_IMU2_fifo_data_count,
    
    // EM
    output wire         o_EM_fifo_rd_en,
    input wire [7:0]    i_EM_fifo_dout,
    input wire [9:0]    i_EM_fifo_data_count,
    
    // Test signals
    output wire [7:0]   o_TX_Byte,
    output wire         o_TX_DV
    );
    
    reg                 r_TX_DV = 0;
    reg [7:0]           r_TX_Byte = 8'd0;
    wire                w_TX_Active;
    wire                w_TX_Done;
    
    reg [2:0]           r_IMU_select = 3'b0;
    wire [7:0]          w_fifo_dout;
    wire [9:0]          w_fifo_data_count;
    reg                 r_fifo_rd_en = 0;
    reg [4:0]           r_TX_byte_counter = 5'd0;
    
    // Main FSM States
    localparam          AWAITING_DATA_IMU1 = 2'd0,
                        AWAITING_DATA_IMU2 = 2'd1,
                        AWAITING_DATA_EM = 2'd2,
                        ACTIVE = 2'd3;
    reg [1:0]           r_current_state = AWAITING_DATA_IMU1;
    reg [1:0]           r_next_IMU_in_return = AWAITING_DATA_IMU1;  // When finished transmitting data from one IMU, switch to another one
    
    // TX FSM States
    localparam          DELAY     = 2'd0,
                        LOAD      = 2'd1,
                        TRANSMIT  = 2'd2,
                        STOP_FIFO = 2'd3;
    reg [1:0]           r_current_TX_state = DELAY;
    
    // IMU select to control MUX and DEMUX
    localparam          IMU1      = 3'd0,
                        IMU2      = 3'd1,
                        EM        = 3'd2;
    
    
    always @(posedge i_clk or negedge i_reset) begin
        if (~i_reset) begin
            r_current_state <= AWAITING_DATA_IMU1;
            r_next_IMU_in_return <= AWAITING_DATA_IMU1;
            r_current_TX_state <= DELAY;
            r_IMU_select <= IMU1;
            r_TX_Byte <= 8'd0;
            r_TX_byte_counter <= 5'd0;
            r_TX_DV <= 0;
            r_fifo_rd_en <= 0;
        end
        else begin
            case (r_current_state)
                AWAITING_DATA_IMU1: begin
                    r_TX_byte_counter <= 5'd0;
                    if (w_fifo_data_count >= 10'd25) begin
                        r_current_state <= ACTIVE;  // Enter TX fsm
                        r_next_IMU_in_return <= AWAITING_DATA_IMU2;  // When sample of imu1 finished transmission, we switch to check imu2
                        r_current_TX_state <= DELAY;
                    end
                    else begin
                        r_current_state <= AWAITING_DATA_IMU2;  // Change to check next imu
                        r_IMU_select <= IMU2;  // pre-select the next imu
                    end
                end
                
                
                AWAITING_DATA_IMU2: begin
                    r_TX_byte_counter <= 5'd0;
                    if (w_fifo_data_count >= 10'd25) begin
                        r_current_state <= ACTIVE;  // Enter TX fsm
                        r_next_IMU_in_return <= AWAITING_DATA_EM;  // When sample of imu2 finished transmission, we switch to check imu1
                        r_current_TX_state <= DELAY;
                    end
                    else begin
                        r_current_state <= AWAITING_DATA_EM;
                        r_IMU_select <= EM;
                    end
                end
                
                
                AWAITING_DATA_EM: begin
                    r_TX_byte_counter <= 5'd0;
                    if (w_fifo_data_count >= 10'd25) begin
                        r_current_state <= ACTIVE;  // Enter TX fsm
                        r_next_IMU_in_return <= AWAITING_DATA_IMU1;  // When sample of imu2 finished transmission, we switch to check imu1
                        r_current_TX_state <= DELAY;
                    end
                    else begin
                        r_current_state <= AWAITING_DATA_IMU1;
                        r_IMU_select <= IMU1;
                    end
                end
                
                
                ACTIVE: begin
                    case (r_current_TX_state)
                        DELAY: begin
                            if (~w_TX_Active) begin  // Wait until previous TX finished
                                r_fifo_rd_en <= 1;  // Start fifo
                                r_current_TX_state <= STOP_FIFO;
                            end
                        end
                        
                        STOP_FIFO: begin
                            r_fifo_rd_en <= 0;  // Stop fifo
                            r_current_TX_state <= LOAD;
                        end
                        
                        LOAD: begin
                            
                            r_TX_Byte <= w_fifo_dout;
                            r_current_TX_state <= TRANSMIT;
                            r_TX_DV <= 1;
                            r_TX_byte_counter <= r_TX_byte_counter + 1;
                            
                        end
                        
                        TRANSMIT: begin
                            r_TX_DV <= 0;
                            r_current_TX_state <= DELAY;
                            // TX finished for 25 bytes
                            if (r_TX_byte_counter == 25) begin
                                r_current_state <= r_next_IMU_in_return;  // jump out from TX fsm
                            end
                        end
                    endcase
                end
            endcase
        end
    end
    
    uart_tx #
    (.CLKS_PER_BIT(243)) uart_tx  // 243 for 115200, 28MHz
    (
        .i_Clock(i_clk),
        .i_Tx_DV(r_TX_DV),
        .i_Tx_Byte(r_TX_Byte),
        .i_reset(i_reset),
        .o_Tx_Active(w_TX_Active),
        .o_Tx_Serial(o_TX_out),
        .o_Tx_Done(w_TX_Done)
    );
    
    Mux8 #(
        .DATA_WIDTH(8),        // Width of each data input and output
        .SELECT_SIZE(3)       // Width of the select signal
    ) fifo_dout_mux8 (
        .select_i(r_IMU_select),       // Select signal
        .data0_i(i_IMU1_fifo_dout),                // Input data 0
        .data1_i(i_IMU2_fifo_dout),                // Input data 1
        .data2_i(i_EM_fifo_dout),                // Input data 2
        .data3_i(8'b0),                // Input data 3
        .data4_i(8'b0),                // Input data 4
        .data5_i(8'b0),                // Input data 5
        .data6_i(8'b0),                // Input data 6
        .data7_i(8'b0),                // Input data 7
        .data_o(w_fifo_dout)             // Output data
    );
    
    Mux8 #(
        .DATA_WIDTH(10),        // Width of each data input and output
        .SELECT_SIZE(3)       // Width of the select signal
    ) fifo_data_count_mux8 (
        .select_i(r_IMU_select),       // Select signal
        .data0_i(i_IMU1_fifo_data_count),                // Input data 0
        .data1_i(i_IMU2_fifo_data_count),                // Input data 1
        .data2_i(i_EM_fifo_data_count),                // Input data 2
        .data3_i(10'b0),                // Input data 3
        .data4_i(10'b0),                // Input data 4
        .data5_i(10'b0),                // Input data 5
        .data6_i(10'b0),                // Input data 6
        .data7_i(10'b0),                // Input data 7
        .data_o(w_fifo_data_count)             // Output data
    );
    
    Demux8 #(
        .DATA_WIDTH(1),    // Parameter for data width
        .SELECT_SIZE(3)   // Parameter for select signal size
    ) fifo_rd_en_demux8 (
        .data_i(r_fifo_rd_en),        // Connect input data
        .select_i(r_IMU_select),     // Connect select signal
        .data0_o(o_IMU1_fifo_rd_en),          // Connect output 0
        .data1_o(o_IMU2_fifo_rd_en),          // Connect output 1
        .data2_o(o_EM_fifo_rd_en),          // Connect output 2
        .data3_o(),          // Connect output 3
        .data4_o(),          // Connect output 4
        .data5_o(),          // Connect output 5
        .data6_o(),          // Connect output 6
        .data7_o()           // Connect output 7
    );
    
    assign o_TX_DV = r_TX_DV;
    assign o_TX_Byte = r_TX_Byte;
    
endmodule
