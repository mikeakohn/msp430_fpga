.msp430

.org 0xf000

main:
  mov.w #0xf003, r4
  ;mov.w &0xf000, r4
  ;mov.b &0xf001, r4
  ;mov.w 0xf000, r4
  mov.b @r4+, r5

  mov.w #100, r6
  mov.w r6, &0x8000
  mov.w &0x8000, r4
  ;mov.w #0x0f0f, r4

  ;reti
loop:
  jmp loop

.org 0xfffe
  .dw 0xf000

