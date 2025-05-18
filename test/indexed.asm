.msp430

.org 0xf000

main:
  mov.w #main, r5
  ;mov.w #5, r5
  ;mov.w r4, r6
  mov.w 2(r5), r4

loop:
  jmp loop

.org 0xfffe
  .dw main

