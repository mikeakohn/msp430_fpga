.msp430

.org 0xf000

main:
  mov.w #main, r5
  mov.w 4(r5), r4

loop:
  jmp loop

.org 0xfffe
  .dw main

