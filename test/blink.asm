.msp430

.org 0xf000

start:
  mov.w #0x1000, SP

main:

loop:
  xor.b #1, &0x4010
  call #delay
  jmp loop

delay:     
  mov.w #0xffff, r4
delay_loop:
  sub.w #1, r4
  jnz delay_loop
  ret

.org 0xfffe
  .dw start

