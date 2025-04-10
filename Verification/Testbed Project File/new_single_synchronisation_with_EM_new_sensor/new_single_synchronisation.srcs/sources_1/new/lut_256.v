module lut_256 (
    input wire [7:0] addr,    // 8-bit address for 256 locations
    output reg [7:0] data     // 1-byte data output
);

    // Define the lookup table as a register array
    reg [7:0] lut [0:255]; // 256 words, each 8 bits wide
    

// ---------------------------------------------------------------------------------------------------------------------------

    initial begin
        // Write Group 1, exit current mode
        lut[0]   = 8'h80;
        lut[1]   = 8'h00;

        // Write Group 2, Reset, total 2 bytes
        lut[2]   = 8'hF0;
        lut[3]   = 8'h00;

        // Write Group 1, Initialisation, total 15 bytes
        // 0x00
        lut[4]   = 8'h60;   // head
        lut[5]   = 8'h00;   // upper byte
        lut[6]   = 8'h70;   // lower byte   8'h7C for normal mode, 8'h70 for super mode
        lut[7]   = 8'h00;   // address << 2
        lut[8]   = 8'h00;   // Read Status

        lut[9]   = 8'h60;   // head
        lut[10]   = 8'h48;   // upper byte
        lut[11]   = 8'h80;   // lower byte
        lut[12]   = 8'h04;   // address << 2
        lut[13]   = 8'h00;   // Read Status

        lut[14]  = 8'h60;   // head
        lut[15]  = 8'h00;   // upper byte
        lut[16]  = 8'h00;   // lower byte   8'h08 for normal mode, 8'h00 for super mode
        lut[17]  = 8'h08;   // address << 2
        lut[18]  = 8'h00;   // Read Status

        // write Group 2, measurement_1, total 2 bytes
        lut[19]  = 8'h32;   // SM, x only
        lut[20]  = 8'h00;
        
        // write Group 3, measurement_2, total 4 bytes
        lut[21]  = 8'h42;   // RD, x only
        lut[22]  = 8'h00;   // Read status
        lut[23]  = 8'h00;   // Read x_upper
        lut[24]  = 8'h00;   // Read x_lower

        // Reserved
        lut[25]  = 8'h00;   // Reserved
        lut[26]  = 8'h00;   // Reserved
        lut[27]  = 8'h00;   // Reserved
        lut[28]  = 8'h00;   // Reserved
        lut[29]  = 8'h00;   // Reserved
        
        // ...
        lut[255] = 8'h00; // Example last value
    end

    // Read data from the lookup table based on the address
    always @(*) begin
        data = lut[addr];
    end

endmodule