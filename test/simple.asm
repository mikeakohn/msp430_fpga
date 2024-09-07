.msp430

.org 0xf000

main:
  mov.w #0x1234, &0x112
  mov.w &0x112, r4

  mov.w #9, r4
  add.w #11, r4

loop:
  jmp loop

.org 0xfffe
  .dw 0xf000

