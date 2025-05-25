.msp430

.low_address 0xf000

.org 0xf800
main:
  ;mov.w #1, r5
  ;mov.w r5, &0x0002
  ;mov.w &0x0002, r4

  mov.b &data+1, r4
  ;mov.w data+1, r4

loop:
  jmp loop

data:
.db 0x12, 0xf2, 0x99, 0x33

.org 0xfffe
  .dc16 main

