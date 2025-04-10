module Demux8 #(
    parameter DATA_WIDTH = 32,
    parameter SELECT_SIZE = 3
) (
    input wire [DATA_WIDTH-1:0] data_i,   // Single input data
    input wire [SELECT_SIZE-1:0] select_i, // Select signal
    output reg [DATA_WIDTH-1:0] data0_o,  // Output 0
    output reg [DATA_WIDTH-1:0] data1_o,  // Output 1
    output reg [DATA_WIDTH-1:0] data2_o,  // Output 2
    output reg [DATA_WIDTH-1:0] data3_o,  // Output 3
    output reg [DATA_WIDTH-1:0] data4_o,  // Output 4
    output reg [DATA_WIDTH-1:0] data5_o,  // Output 5
    output reg [DATA_WIDTH-1:0] data6_o,  // Output 6
    output reg [DATA_WIDTH-1:0] data7_o   // Output 7
);

always @(*) begin
    // Default all outputs to zero
    data0_o = 0;
    data1_o = 0;
    data2_o = 0;
    data3_o = 0;
    data4_o = 0;
    data5_o = 0;
    data6_o = 0;
    data7_o = 0;

    // Assign the input to the selected output
    case (select_i)
        3'b000: data0_o = data_i;
        3'b001: data1_o = data_i;
        3'b010: data2_o = data_i;
        3'b011: data3_o = data_i;
        3'b100: data4_o = data_i;
        3'b101: data5_o = data_i;
        3'b110: data6_o = data_i;
        3'b111: data7_o = data_i;
        default: ; // Do nothing
    endcase
end

endmodule
