.msp430

.org 0xf000

main:
  clrc
  mov.w  #0x1234, r4
  dadd.w #0x9168, r4

  ;clrc
  ;mov.w  #0x1234, r4
  ;dadd.b #0x0068, r4

loop:
  jmp loop

.org 0xfffe
  .dw 0xf000

