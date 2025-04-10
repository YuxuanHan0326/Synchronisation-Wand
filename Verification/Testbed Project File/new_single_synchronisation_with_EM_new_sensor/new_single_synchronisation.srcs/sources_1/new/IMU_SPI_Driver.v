module IMU_SPI_Driver
#(
    parameter           IMU_ID = 8'd0,
    parameter           TIME_OFFSET = 32'd0)
(
    input  wire         i_Clk,       // System clock
    input  wire         i_Rst_L,     // Reset (active low)
    output wire         o_SPI_Clk,   // SPI clock
    output wire         o_SPI_MOSI,  // Master Out Slave In
    input  wire         i_SPI_MISO,  // Master In Slave Out
    output wire         o_SPI_CS_n,  // CS
    output wire         o_SM_trigger,  // Trigger for single measurement
    input  wire         i_DRDY_synced,      // Data ready signal
    output wire [199:0] o_output_sample,
    output wire         o_sample_valid,
    input  wire [39:0]  i_timestamp,

    // New debug outputs for internal signals
    output wire [7:0]   o_TX_Byte,   // Expose r_TX_Byte
    output wire [7:0]   o_RX_Byte,   // Expose w_RX_Byte
    output wire         o_TX_Valid,  // Expose r_TX_DV
    output wire         o_RX_Valid,  // Expose w_RX_DV
    output wire         o_TX_Ready,   // Expose w_TX_Ready
    output wire [5:0]   o_current_state,
    output wire [7:0]   o_bytes_transmitted,
    output wire         o_timer_IR_flg,
    output wire         o_TX_enable,
    output wire         o_custom_trigger
);

    // Internal signals
    reg [7:0] r_TX_Byte = 8'h00;       // Byte to send via SPI
    reg       r_TX_DV = 1'b0;          // Data valid signal
    wire      w_TX_Ready;              // Ready signal from SPI master
    wire [7:0] w_RX_Byte;              // Byte received via SPI
    wire      w_RX_DV;                 // Data valid signal for received byte
    reg [7:0] lut_ptr = 8'h00;       // Counter for generating test data
    reg lut_ptr_jump = 1;           // Active Low. Jump signal for main loop

    reg TX_enable = 1'b0;
    wire [7:0] lut_output;
    
    reg [7:0] bytes_transmitted = 8'd0;
    reg [31:0] delay_counter = 32'd0;
    
    reg timer_IR_flg = 1;
    reg timer_CC1_flg = 0;
    reg [31:0] timer_autoreload_value = 32'd0;
    reg [31:0] timer_CC1_value = 32'd0;
    reg TX_byte_counter_reset = 1;
    reg custom_trigger = 0;
    reg r_SM_trigger = 0;
    
    localparam MAX_BYTES_PER_CS = 31;  // Max bytes transmitted per CS
    
    reg [$clog2(MAX_BYTES_PER_CS+1)-1:0] r_Master_TX_Count = 5'd2;  // Default transmit 2 bytes per CS
    wire [$clog2(MAX_BYTES_PER_CS+1)-1:0] w_Master_RX_Count;
    
    localparam PACKAGE_HEAD = 8'h2C;
    
    reg [199:0] r_output_sample = {PACKAGE_HEAD, IMU_ID, 184'd0};
    reg [39:0]  r_trigger_timestamp = 0;
    reg r_sample_valid = 40'd0;

    // Instantiate SPI Master
    SPI_Master_With_Single_CS #(
        .SPI_MODE(3),  // Mode 3 for MLX90393
        .CLKS_PER_HALF_BIT(2),
        .MAX_BYTES_PER_CS(MAX_BYTES_PER_CS),
        .CS_INACTIVE_CLKS(10),
        .CS_HOLD_CLKS(1)  // No need fpr MLX
    ) spi_master_inst (
        .i_Rst_L(i_Rst_L),
        .i_Clk(i_Clk),
        
        .i_TX_Count(r_Master_TX_Count),   // Number of bytes per CS
        .i_TX_Byte(r_TX_Byte),
        .i_TX_DV(r_TX_DV),
        .o_TX_Ready(w_TX_Ready),
        
        .o_RX_Count(w_Master_RX_Count), // Index of RX'd byte
        .o_RX_DV(w_RX_DV),
        .o_RX_Byte(w_RX_Byte),
        
        .o_SPI_Clk(o_SPI_Clk),
        .i_SPI_MISO(i_SPI_MISO),
        .o_SPI_MOSI(o_SPI_MOSI),
        .o_SPI_CS_n(o_SPI_CS_n)
    );

    // Instantiate LUT_256 for TX messages
    lut_256 TX_lut256 (
        .addr(lut_ptr),
        .data(lut_output)
    );
    
    
    // Main Loop Jump Position
    localparam MAIN_LOOP_JUMP_POS = 8'd19;
    
    // FSM States
    localparam TX_GROUP_1 = 5'd0,
               PRE_DELAY_1 = 5'd1,
               DELAY_1 = 5'd2,
               SET_TRIG = 5'd3,
               AWAITING_DRDY = 5'd4,
               MAIN_SAMPLING = 5'd5,
               CAPTURING_LAST_BYTE = 5'd6,
               AWAITING_TIMER = 5'd7,
               SEND_TRIG = 5'd8,
               TX_GROUP_2 = 5'd9,
               PRE_DELAY_2 = 5'd10,
               DELAY_2 = 5'd11,
               TX_GROUP_3 = 5'd12,
               PRE_DELAY_3 = 5'd13,
               DELAY_3 = 5'd14;
    
    
    // Timer autoreload value (200, 1000, 200 for simulation)
    localparam TIMER_AUTORELOAD_VALUE_100MS = 32'd2800000;  // Based on a 28 MHz clk (32'd2800000)
    localparam TIMER_AUTORELOAD_VALUE_MAIN_SAMPLING = 32'd1119958;  // 40ms (32'd1119958) -42 ticks for fsm
    localparam TIMER_CC1_VALUE = 32'd8400;  // 300 us (32'd8400)
    localparam TIMER_AUTORELOAD_VALUE_2MS = 32'd56000;  // Based on a 28 MHz clk (32'd56000)
               
    reg [4:0] current_state = TX_GROUP_1;
    
    always @(posedge i_Clk or negedge i_Rst_L) begin
        if (~i_Rst_L) begin
            current_state <= TX_GROUP_1;
            TX_enable <= 1'b0;
            timer_IR_flg <= 1'b1;
            timer_CC1_flg <= 1'b0;
            delay_counter <= 32'd0;
            timer_autoreload_value <= 32'd0;
            timer_CC1_value <= 32'd0;
            r_TX_Byte <= 8'h00;   // Reset transmitted byte
            r_TX_DV <= 1'b0;      // Clear data valid flag
            TX_byte_counter_reset <= 1;
            lut_ptr_jump <= 1;
            custom_trigger <= 0;
            r_Master_TX_Count <= 5'd2;
            r_output_sample <= {PACKAGE_HEAD, IMU_ID, 184'd0};
            r_sample_valid <= 0;
            r_trigger_timestamp <= 40'd0;
        end else begin    
            // Timer logic
            if (~timer_IR_flg) begin
                if (delay_counter == timer_CC1_value) begin
                    timer_CC1_flg <= 1;  // Set CC1 flag
                end else begin
                    timer_CC1_flg <= 0;  // Reset CC1 flag
                end

                if (delay_counter < timer_autoreload_value) begin
                    delay_counter <= delay_counter + 1;
                end else begin
                    timer_IR_flg <= 1;       // Set timer flag
                    delay_counter <= 32'd0; // Reset delay counter
                end
            end
            
            // TX Transmitter Logic
            // If TX is ready and TX required
            if (w_TX_Ready & TX_enable) begin
                // Load the next byte when SPI master is ready
                r_TX_Byte <= lut_output;
                r_TX_DV <= 1'b1;  // Assert data valid flag
            end
            // Otherwise do nothing (Reset DV)
            else begin
                r_TX_DV <= 1'b0;  // Deassert data valid flag
            end
            
    
            // FSM logic
            case (current_state)

                // Exit current mode

                TX_GROUP_1: begin
                    TX_enable <= 1;                                             // Start TX
                    if (bytes_transmitted == 2) begin
                        TX_byte_counter_reset <= 0;                             // Reset byte counter
                        current_state <= PRE_DELAY_1;                           // Enter pre_delay state
                    end
                end

                PRE_DELAY_1: begin
                    TX_enable <= 0;                                             // Stop TX
                    if (w_TX_Ready) begin
                        TX_byte_counter_reset <= 1;                                 // Set back counter reset signal
                        timer_autoreload_value <= TIMER_AUTORELOAD_VALUE_2MS + TIME_OFFSET;     // Set timer auto-reload value
                        timer_IR_flg <= 0;                                          // Start timer
                        current_state <= DELAY_1;                                      // Enter delay state
                    end
                end

                DELAY_1: begin
                    if (timer_IR_flg) begin                                     // Wait for timer
                        current_state <= TX_GROUP_2;                            // Enter main sampling state
                    end
                end

                // Soft Reset

                TX_GROUP_2: begin
                    TX_enable <= 1;                                             // Start TX
                    if (bytes_transmitted == 2) begin
                        TX_byte_counter_reset <= 0;                             // Reset byte counter
                        current_state <= PRE_DELAY_2;                           // Enter pre_delay state
                    end
                end

                PRE_DELAY_2: begin
                    TX_enable <= 0;                                             // Stop TX
                    if (w_TX_Ready) begin
                        TX_byte_counter_reset <= 1;                                 // Set back counter reset signal
                        timer_autoreload_value <= TIMER_AUTORELOAD_VALUE_2MS;     // Set timer auto-reload value
                        timer_IR_flg <= 0;                                          // Start timer
                        current_state <= DELAY_2;                                      // Enter delay state
                    end
                end

                DELAY_2: begin
                    if (timer_IR_flg) begin                                     // Wait for timer
                        current_state <= TX_GROUP_3;                            // Enter main sampling state
                    end
                end

                // Configure Registers

                TX_GROUP_3: begin
                    TX_enable <= 1;                                             // Start TX
                    r_Master_TX_Count <= 5'd5;
                    if (bytes_transmitted == 15) begin
                        TX_byte_counter_reset <= 0;                             // Reset byte counter
                        current_state <= PRE_DELAY_3;                           // Enter pre_delay state
                    end
                end

                PRE_DELAY_3: begin
                    TX_enable <= 0;                                             // Stop TX
                    if (w_TX_Ready) begin
                        TX_byte_counter_reset <= 1;                                 // Set back counter reset signal
                        timer_autoreload_value <= TIMER_AUTORELOAD_VALUE_100MS;     // Set timer auto-reload value
                        timer_IR_flg <= 0;                                          // Start timer
                        current_state <= DELAY_3;                                      // Enter delay state
                    end
                end

                DELAY_3: begin
                    if (timer_IR_flg) begin                                     // Wait for timer
                        current_state <= SEND_TRIG;                            // Enter main sampling state
                    end
                end

                SEND_TRIG: begin
                    custom_trigger <= 1;
                    TX_enable <= 1;                                             // Start TX
                    r_Master_TX_Count <= 5'd2;                                  // Transmit 2 bytes
                    if (bytes_transmitted == 2) begin
                        TX_byte_counter_reset <= 0;                             // Reset byte counter
                        current_state <= SET_TRIG;                           // Enter pre_delay state
                        r_trigger_timestamp <= i_timestamp;                     // Debug
                    end
                end

                SET_TRIG: begin
                    custom_trigger <= 0;
                    TX_byte_counter_reset <= 1;                                 // Set back counter reset signal
                    TX_enable <= 0;
                    // r_SM_trigger <= 1;                                          // Set trigger
                    timer_autoreload_value <= TIMER_AUTORELOAD_VALUE_MAIN_SAMPLING;     // Set timer reload value
                    timer_CC1_value <= TIMER_CC1_VALUE;                        // Set CC1 value
                    timer_IR_flg <= 0;                                          // Start timer
                    current_state <= AWAITING_DRDY;                            // Enter DRDY state
                end

                AWAITING_DRDY: begin
                    // Wait for CC1 flag to switch off SM_trigger
                    if (timer_CC1_flg) begin
                        r_SM_trigger <= 0;                                      // Reset trigger
                    end

                    // Wait for DRDY signal to enter main sampling state and record timestamp
                    if (i_DRDY_synced) begin
                        r_output_sample [183:144] <= i_timestamp;               // Add timestamp
                        r_Master_TX_Count <= 5'd4;                              // Transmit 4 bytes
                        current_state <= MAIN_SAMPLING;                         // Enter main sampling state
                    end
                end

                MAIN_SAMPLING: begin
                    TX_enable <= 1;                                             // Start TX

                    // Data Logging
                    if (w_RX_DV) begin
                        case (w_Master_RX_Count)
                            5'd2: begin
                                r_output_sample [47:40] <= w_RX_Byte;           // Add Hx_upper
                            end

                            5'd3: begin
                                r_output_sample [39:32] <= w_RX_Byte;           // Add Hx_lower
                            end
                        endcase
                    end

                    if (bytes_transmitted == 4) begin
                        TX_byte_counter_reset <= 0;                             // Reset byte counter
                        current_state <= CAPTURING_LAST_BYTE;                        
                    end
                end

                CAPTURING_LAST_BYTE: begin
                    TX_enable <= 0;                                             // Stop TX
                    lut_ptr_jump <= 0;

                    // Capture last byte
                    if (w_RX_DV & (w_Master_RX_Count == 5'd3)) begin
                        r_output_sample [39:32] <= w_RX_Byte;           // Add Hx_lower
                        r_sample_valid <= 1;                                    // Set sample valid and output
                    end else begin
                        r_sample_valid <= 0;                                    // Reset sample valid
                    end

                    if (w_TX_Ready) begin
                        TX_byte_counter_reset <= 1;                                 // Set back counter reset signal
                        current_state <= AWAITING_TIMER;                               // Enter TX_GROUP_1 state
                    end
                end

                AWAITING_TIMER: begin
                    lut_ptr_jump <= 1;
                    if (timer_IR_flg) begin                                     // Wait for timer
                        current_state <= SEND_TRIG;                            // Enter TX_GROUP_1 state
                    end
                end

                default: begin
                    current_state <= TX_GROUP_1;
                end
            endcase
        end
    end


    
    // Progress lut_ptr if one transmission started
    always @(negedge r_TX_DV or negedge i_Rst_L or negedge lut_ptr_jump) begin
        if (~i_Rst_L) begin
            lut_ptr <= 8'h00;   // Reset counter
        end
        else if (~lut_ptr_jump) begin
            lut_ptr <= MAIN_LOOP_JUMP_POS;
        end
        else begin
            if (TX_enable) begin
                lut_ptr <= lut_ptr + 1;
            end
            else begin
                lut_ptr <= lut_ptr;
            end
        end
    end
    
    // Record bytes trasmitted
    always @(negedge r_TX_DV or negedge i_Rst_L or negedge TX_byte_counter_reset) begin
        if (~i_Rst_L) begin
            bytes_transmitted <= 8'd0;
        end
        else if (~TX_byte_counter_reset) begin
            bytes_transmitted <= 8'd0;
        end
        else begin
            if (TX_enable) begin
                bytes_transmitted <= bytes_transmitted + 1;
            end
            else begin
                bytes_transmitted <= bytes_transmitted;
            end
        end
    end
    
    
    // Assign transmit ready to output for monitoring
    assign o_TX_Ready = w_TX_Ready;
    assign o_RX_Byte = w_RX_Byte;
    assign o_TX_Byte = r_TX_Byte;
    assign o_TX_Valid = r_TX_DV;
    assign o_RX_Valid = w_RX_DV;
    assign o_current_state = current_state;
    assign o_bytes_transmitted = bytes_transmitted;
    assign o_timer_IR_flg = timer_IR_flg;
    assign o_TX_enable = TX_enable;
    assign o_custom_trigger = custom_trigger;
    assign o_output_sample = r_output_sample;
    assign o_sample_valid = r_sample_valid;
    assign o_SM_trigger = r_SM_trigger;
    

endmodule
