module Mux8 #(
    parameter DATA_WIDTH = 32,
    parameter SELECT_SIZE = 3
) (
    input wire [SELECT_SIZE-1:0] select_i,
    input wire [DATA_WIDTH-1:0] data0_i,  // Data input
    input wire [DATA_WIDTH-1:0] data1_i,  // Data input
    input wire [DATA_WIDTH-1:0] data2_i,  // Data input
    input wire [DATA_WIDTH-1:0] data3_i,  // Data input
    input wire [DATA_WIDTH-1:0] data4_i,  // Data input
    input wire [DATA_WIDTH-1:0] data5_i,  // Data input
    input wire [DATA_WIDTH-1:0] data6_i,  // Data input
    input wire [DATA_WIDTH-1:0] data7_i,  // Data input
    output reg [DATA_WIDTH-1:0] data_o    // Output
);

always @(*) begin
    case (select_i)
        3'b000: data_o = data0_i;
        3'b001: data_o = data1_i;
        3'b010: data_o = data2_i;
        3'b011: data_o = data3_i;
        3'b100: data_o = data4_i;
        3'b101: data_o = data5_i;
        3'b110: data_o = data6_i;
        3'b111: data_o = data7_i;
        default: data_o = {DATA_WIDTH{1'b0}};
    endcase
end

endmodule
