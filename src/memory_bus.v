// MSP430 FPGA Soft Processor
//  Author: Michael Kohn
//   Email: mike@mikekohn.net
//     Web: https://www.mikekohn.net/
//   Board: iceFUN iCE40 HX8K
// License: MIT
//
// Copyright 2024-2025 by Michael Kohn

// The purpose of this module is to route reads and writes to the 4
// different memory banks.

module memory_bus
(
  input  [15:0] address,
  input  [15:0] data_in,
  input  [1:0] write_mask,
  output [15:0] data_out,
  //output [7:0] debug,
  input bus_enable,
  input write_enable,
  input clk,
  input raw_clk,
  output speaker_p,
  output speaker_m,
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

wire [15:0] rom_data_out;
wire [15:0] ram_data_out_0;
wire [15:0] peripherals_data_out;
wire [15:0] ram_data_out_1;

wire ram_write_enable_0;
wire peripherals_write_enable;
wire ram_write_enable_1;

assign ram_write_enable_0       = (address[15:14] == 2'b00) && write_enable;
assign peripherals_write_enable = (address[15:14] == 2'b01) && write_enable;
assign ram_write_enable_1       = (address[15:14] == 2'b10) && write_enable;

// FIXME: The RAM probably need an enable also.
wire peripherals_enable;
assign peripherals_enable = (address[15:14] == 2'b01) && bus_enable;

// Based on the selected bank of memory (address[15:14]) select if
// memory should read from ram.v, rom.v, peripherals.v.
assign data_out = address[15] == 0 ?
  (address[14] == 0 ? ram_data_out_0 : peripherals_data_out) :
  (address[14] == 0 ? ram_data_out_1 : rom_data_out);

// 0x0000 - 0x0fff: 0000 0000 0000 0000 - 0000 1111 1111 1111
ram ram_0(
  .address      (address[11:0]),
  .data_in      (data_in),
  .data_out     (ram_data_out_0),
  .write_mask   (write_mask),
  .write_enable (ram_write_enable_0),
  .clk          (raw_clk)
);

// 0x4000 - 0x40xx: 0100 0000 0000 0000
peripherals peripherals_0(
  .enable       (peripherals_enable),
  .address      (address[7:0]),
  .data_in      (data_in),
  .data_out     (peripherals_data_out),
  //.debug        (debug),
  .write_enable (peripherals_write_enable),
  .clk          (clk),
  .raw_clk      (raw_clk),
  .speaker_p    (speaker_p),
  .speaker_m    (speaker_m),
  .ioport_0     (ioport_0),
  .ioport_1     (ioport_1),
  .ioport_2     (ioport_2),
  .ioport_3     (ioport_3),
  .button_0     (button_0),
  .reset        (reset),
  .spi_clk      (spi_clk),
  .spi_mosi     (spi_mosi),
  .spi_miso     (spi_miso)
);

// 0x8000 - 0x8fff: 1000 0000 0000 0000 - 1000 1111 1111 1111
ram ram_1(
  .address      (address[11:0]),
  .data_in      (data_in),
  .data_out     (ram_data_out_1),
  .write_mask   (write_mask),
  .write_enable (ram_write_enable_1),
  .clk          (raw_clk)
);

// 0xf000 - 0xffff: 1111 0000 0000 0000 - 1111 1111 1111 1111
rom rom_0(
  .address   (address[11:0]),
  .data_out  (rom_data_out),
  .clk       (raw_clk)
);

endmodule

