MSP430
======

This is an implementation of an MSP430 CPU implemented in Verilog
to be run on an FPGA. The board being used here is an iceFUN with
a Lattice iCE40 HX8K FPGA.

This project was created by forking the RISC-V FPGA project
changing the memory_bus.v module to take a 16 bit databus and
changing the core processor to decode MSP430 instructions.

https://www.mikekohn.net/micro/msp430_fpga.php

Features
========

IO, Button input, speaker tone generator, and SPI.

Registers
=========

    r0    PC
    r1    SP
    r2    SR
    r3    CR  [ V,  SCG1, SCG0, OSC-OFF, CPU-OFF,  GIE, Z, N, C ]
    r4-15

Addressing Modes
================

    As  Reg   Mode         Dest
    00        Rn             *
    01        x(Rn)          *
    10        @Rn
    11        @Rn+

    01  pc    address        *
    11  pc    #immediate

    01  sr    &address       *
    10  sr    #4
    11  sr    #8
    00  cg    #0
    01  cg    #1
    10  cg    #2
    11  cg    #-1

Instructions
============

Single-Operand Arithmetic
-------------------------

    0001 00 000 wSS dddd rrc
    0001 00 001 0SS dddd swpb
    0001 00 010 wSS dddd rra
    0001 00 011 0SS dddd sxt
    0001 00 100 wSS dddd push
    0001 00 101 0SS dddd call
    0001 00 110 000 0000 reti

Conditional Jump
----------------

    001 000 oooooooooo jne/jnz
    001 001 oooooooooo jeq/jz
    001 010 oooooooooo jnc/jlo
    001 011 oooooooooo jc/jhs
    001 100 oooooooooo jn
    001 101 oooooooooo jge
    001 110 oooooooooo jl
    001 111 oooooooooo jmp


Two-Operand Arithmetic
----------------------

    0100 ssss DwSS dddd mov
    0101 ssss DwSS dddd add
    0110 ssss DwSS dddd addc
    0111 ssss DwSS dddd subc
    1000 ssss DwSS dddd sub
    1001 ssss DwSS dddd cmp
    1010 ssss DwSS dddd dadd
    1011 ssss DwSS dddd bit
    1100 ssss DwSS dddd bic
    1101 ssss DwSS dddd bis
    1110 ssss DwSS dddd xor
    1111 ssss DwSS dddd and

* ssss is src reg.
* dddd is dst reg.
* D  is Ad.
* SS is As.
* w  is B/W.
* ooo is offset for jumps.

Memory Map
==========

This implementation of the RISC-V has 4 banks of memory. Each address
contains a 16 bit word instead of 8 bit byte like a typical CPU.

* Bank 0: 0x0000 RAM (4096 bytes)
* Bank 1: 0x4000 Peripherals
* Bank 2: 0x8000 RAM (4096 bytes)
* Bank 3: 0xf000 ROM (4096 bytes)

On start-up by default, the chip will load a program from a AT93C86A
2kB EEPROM with a 3-Wire (SPI-like) interface but wll run the code
from the ROM. To start the program loaded to RAM, the program select
button needs to be held down while the chip is resetting.

The peripherals area contain the following:

* 0x4000: input from push button
* 0x4002: SPI TX buffer
* 0x4004: SPI RX buffer
* 0x4006: SPI control: bit 2: 8/16, bit 1: start strobe, bit 0: busy
* 0x4010: ioport_A output (in my test case only 1 pin is connected)
* 0x4012: MIDI note value (60-96) to play a tone on the speaker or 0 to stop
* 0x4014: ioport_B output (3 pins)

IO
--

iport_A is just 1 output in my test circuit to an LED.
iport_B is 3 outputs used in my test circuit for SPI (RES/CS/DC) to the LCD.

MIDI
----

The MIDI note peripheral allows the iceFUN board to play tones at specified
frequencies based on MIDI notes.

SPI
---

The SPI peripheral has 3 memory locations. One location for reading
data after it's received, one location for filling the transmit buffer,
and one location for signaling.

For signaling, setting bit 1 to a 1 will cause whatever is in the TX
buffer to be transmitted. Until the data is fully transmitted, bit 0
will be set to 1 to let the user know the SPI bus is busy.

There is also the ability to do 16 bit transfers by setting bit 2 to 1.

