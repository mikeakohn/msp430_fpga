// MSP430 FPGA Soft Processor
//  Author: Michael Kohn
//   Email: mike@mikekohn.net
//     Web: https://www.mikekohn.net/
//   Board: iceFUN iCE40 HX8K
// License: MIT
//
// Copyright 2024-2025 by Michael Kohn

module peripherals
(
  input enable,
  input [7:0] address,
  input [15:0] data_in,
  output reg [15:0] data_out,
  //output [7:0] debug,
  input  write_enable,
  input  clk,
  input  raw_clk,
  output ioport_0,
  output ioport_1,
  output ioport_2,
  output ioport_3,
  input  button_0,
  input  reset,
  output spi_clk,
  output spi_mosi,
  input  spi_miso
);

//reg [7:0] storage [3:0];

reg [7:0]  buttons;

reg [7:0] ioport_a = 0;
assign ioport_0 = ioport_a[0];
reg [7:0] ioport_b = 0; // 8'hf0;
assign ioport_1 = ioport_b[0];
assign ioport_2 = ioport_b[1];
assign ioport_3 = ioport_b[2];

//assign debug = ioport_b;
//assign debug = spi_tx_buffer;

wire [7:0] spi_rx_buffer;
reg  [7:0] spi_tx_buffer;
wire spi_busy;
reg spi_start;
//reg spi_width_16;

always @(button_0) begin
  buttons = { 7'b0, ~button_0 };
end

// FIXME: Fix this...
// This should be able to be clk instead of raw_clk, but it seems that
// two consecutive writes this module keeps stale data in data_in. So
// will put 6 into both 0x4008 and 0x400a
// Wiring to RAM in between keeps data_in with the correct result.
always @(posedge raw_clk) begin
//always @(posedge enable) begin
  //if (reset) speaker_value_high <= 0;

  if (write_enable) begin
    case (address[7:1])
      5'h1: spi_tx_buffer <= data_in;
      5'h3:
        begin
          if (data_in[1] == 1) spi_start <= 1;
          //spi_width_16 <= data_in[2];
        end
      5'h8: ioport_a <= data_in;
      5'ha: ioport_b <= data_in;
    endcase
  end else begin
    if (spi_start && spi_busy) spi_start <= 0;

    if (enable) begin
      case (address[7:1])
        5'h0: data_out <= buttons;
        5'h1: data_out <= spi_tx_buffer;
        5'h2: data_out <= spi_rx_buffer;
        5'h3: data_out <= { 6'b000000, 1'b0, spi_busy };
        5'h8: data_out <= ioport_a;
        5'ha: data_out <= ioport_b;
        default: data_out <= 0;
      endcase
    end
  end
end

spi spi_0
(
  .raw_clk  (raw_clk),
  .start    (spi_start),
  //.width_16 (spi_width_16),
  .data_tx  (spi_tx_buffer),
  .data_rx  (spi_rx_buffer),
  .busy     (spi_busy),
  .sclk     (spi_clk),
  .mosi     (spi_mosi),
  .miso     (spi_miso)
);

endmodule

