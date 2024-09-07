// MSP430 FPGA Soft Processor
//  Author: Michael Kohn
//   Email: mike@mikekohn.net
//     Web: https://www.mikekohn.net/
//   Board: iceFUN iCE40 HX8K
// License: MIT
//
// Copyright 2024 by Michael Kohn

// This creates 2048 bytes of RAM on the FPGA itself. Written this
// way makes it inferred by the IceStorm tools. It seems like it
// only infers it to BlockRam when using double_clk, which based
// on the timing chart in the Lattice documentation seems to make
// sense.

module ram
(
  input [11:0] address,
  input [15:0] data_in,
  output reg [15:0] data_out,
  input [1:0] write_mask,
  input write_enable,
  input clk
);

reg [7:0] storage_0 [2047:0];
reg [7:0] storage_1 [2047:0];

wire [10:0] aligned_address;
assign aligned_address = address[11:1];

//always @(posedge double_clk) begin
always @(posedge clk) begin
  if (write_enable) begin
    if (!write_mask[0]) storage_0[aligned_address] <= data_in[7:0];
    if (!write_mask[1]) storage_1[aligned_address] <= data_in[15:8];
  end else begin
    data_out[7:0]   <= storage_0[aligned_address];
    data_out[15:8]  <= storage_1[aligned_address];
  end
end

endmodule

