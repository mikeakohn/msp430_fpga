.msp430

.include "lcd/ssd1331.inc"

.org 0xf000

;; Registers.
BUTTON     equ 0x4000
SPI_TX     equ 0x4002
SPI_RX     equ 0x4004
SPI_CTL    equ 0x4006
PORT0      equ 0x4010
SOUND      equ 0x4012
SPI_IO     equ 0x4014

;; Bits in SPI_CTL.
SPI_BUSY   equ 1
SPI_START  equ 2
;SPI_16     equ 4

;; Bits in SPI_IO.
LCD_RES    equ 1
LCD_DC     equ 2
LCD_CS     equ 4

;; Bits in PORT0
LED0       equ 1

.macro square_fixed(var)
.scope
  mov.w &var, r10
  bit.w #0x8000, r10
  jz not_signed
  xor.w #0xffff, r10
  add.w #1, r10
not_signed:
  mov.w r10, r9
  call #multiply
  call #shift_right_10
.ends
.endm

;; r10: input 0   (6)
;; r9:  input 1   (7)
;; r12: LSB return
;; r13: MSB return
.macro multiply_fixed(var1, var2)
  mov.w &var1, r10
  mov.w &var2, r9
  call #multiply_signed
  call #shift_right_10
.endm

start:
  ;; Setup stack.
  mov.w #0x1000, SP

  ;; Clear LED.
  bic.b #LED0, &PORT0

main:
  call #lcd_init
  call #lcd_clear

while_1:
  bit.b #1, &BUTTON
  jnz run

  call #delay
  call #toggle_led
  jmp while_1

run:
  call #lcd_clear_2
  call #mandelbrot
  jmp while_1

lcd_init:
  ;mov.b #0, &SPI_CTL
  mov.b #LCD_CS, &SPI_IO
  call #delay
  mov.b #LCD_CS | LCD_RES, &SPI_IO

  call #send_init_data
  ret

lcd_clear:
  ;bis.b #0, &SPI_CTL
  mov.w #(96 * 64), r4
lcd_clear_loop:
  mov.b #0x0f, r15
  call #lcd_send_data
  mov.b #0x0f, r15
  call #lcd_send_data
  dec.w r4
  jnz lcd_clear_loop
  ;bic.b #SPI_16, &SPI_CTL
  ret

lcd_clear_2:
  ;bis.b #SPI_16, &SPI_CTL
  mov.w #(96 * 64), r4
lcd_clear_loop_2:
  mov.b #0xf0, r15
  call #lcd_send_data
  mov.b #0x0f, r15
  call #lcd_send_data
  dec.w r4
  jnz lcd_clear_loop_2
  ;bic.b #SPI_16, &SPI_CTL
  ret

;; uint32_t multiply(int16_t, int16_t);
;; r10: input 0 LSB   (6)
;; r11: input 0 MSB   (11)
;; r9:  input 1       (7)
;; r12: output (LSB)  (8)
;; r13: output (MSB)  (9)
;; r14: counter.
multiply:
  ; Set output to 0.
  mov.w #0, r12
  mov.w #0, r13
  ; Set temporary MSB input to 0.
  mov.w #0, r11
  ; Set counter to 16.
  mov.w #16, r14
multiply_repeat:
  bit.w #1, r9
  jz multiply_ignore_bit
  add.w r10, r12
  addc.w r11, r13
multiply_ignore_bit:
  rla.w r10
  rlc.w r11
  rra.w r9
  sub.w #1, r14
  jnz multiply_repeat
  ret

;; This is only 16x16=16.
;; r10: input 0   (6)
;; r9:  input 1   (7)
;; r12: output (LSB)  (8)
;; r13: output (MSB)  (9)
multiply_signed:
  ;; Keep track of sign bits
  mov.w #0, r4
  bit.w #0x8000, r10
  jz multiply_signed_var0_positive
  ;; abs(var0).
  xor.w #1, r4
  xor.w #0xffff, r10
  add.w #1, r10
multiply_signed_var0_positive:
  bit.w #0x8000, r9
  jz multiply_signed_var1_positive
  ;; abs(var1).
  xor.w #1, r4
  xor.w #0xffff, r9
  add.w #1, r9
multiply_signed_var1_positive:
  call #multiply
  cmp.w #0, r4
  jz multiply_signed_not_neg
  xor.w #0xffff, r12
  xor.w #0xffff, r13
  add.w #1, r12
  addc.w #0, r13
multiply_signed_not_neg:
  ret

;; r12: output (LSB)  (8)
;; r13: output (MSB)  (9)
shift_right_10_slow:
  mov.w #10, r14
shift_right_10_loop:
  rra.w r13
  rrc.w r12
  dec.w r14
  jnz shift_right_10_loop
  ret

;; This reduces the time to generate the Mandelbrot from 54 seconds to
;; 45 seconds (over the shift_right_10_slow function).
;; r12: output (LSB)  (8)
;; r13: output (MSB)  (9)
shift_right_10:
  rra.w r13
  rrc.w r12
  rra.w r13
  rrc.w r12
  swpb r12
  swpb r13
  bic.w #0xff00, r12
  bic.w #0x00ff, r13
  bis.w r13, r12
  ret

curr_x equ 20
curr_y equ 22
curr_r equ 24
curr_i equ 26
color  equ 28
zr     equ 30
zi     equ 32
zr2    equ 34
zi2    equ 36
tr     equ 38
ti     equ 40

mandelbrot:
  ;; final int DEC_PLACE = 10;
  ;; final int r0 = (-2 << DEC_PLACE);
  ;; final int i0 = (-1 << DEC_PLACE);
  ;; final int r1 = (1 << DEC_PLACE);
  ;; final int i1 = (1 << DEC_PLACE);
  ;; final int dx = (r1 - r0) / 96; (0x0020)
  ;; final int dy = (i1 - i0) / 64; (0x0020)

  ;; Set SPI to 16 bit.
  ;bis.b #SPI_16, &SPI_CTL

  ;; for (y = 0; y < 64; y++)
  mov.w #64, &curr_y

  ;; int i = -1 << 10;
  mov.w #0xfc00, &curr_i

mandelbrot_for_y:
  ;; for (x = 0; x < 96; x++)
  mov.w #96, &curr_x

  ;; int r = -2 << 10;
  mov.w #0xf800, &curr_r

mandelbrot_for_x:
  ;; zr = r;
  ;; zi = i;
  mov.w &curr_r, &zr
  mov.w &curr_i, &zi

  ;; for (int count = 0; count < 15; count++)
  mov.w #0, &color

mandelbrot_for_count:
  ;; zr2 = (zr * zr) >> DEC_PLACE;
  square_fixed(zr)
  mov.w r12, &zr2

  ;; zi2 = (zi * zi) >> DEC_PLACE;
  square_fixed(zi)
  mov.w r12, &zi2

  ;; if (zr2 + zi2 > (4 << DEC_PLACE)) { break; }
  ;; cmp does: 4 - (zr2 + zi2).. if it's negative it's bigger than 4.
  mov.w &zi2, r15
  add.w &zr2, r15
  cmp #(4 << 10), r15
  jge mandelbrot_stop

  ;; tr = zr2 - zi2;
  mov.w &zr2, r15
  sub.w &zi2, r15
  mov.w r15, &tr

  ;; ti = ((zr * zi) >> DEC_PLACE) << 1;
  multiply_fixed(zr, zi)
  rla.w r12
  mov.w r12, &ti

  ;; zr = tr + curr_r;
  mov.w &tr, r15
  add.w &curr_r, r15
  mov.w r15, &zr

  ;; zi = ti + curr_i;
  mov.w &ti, r15
  add.w &curr_i, r15
  mov.w r15, &zi

  inc.w &color
  cmp.w #16, &color
  jnz mandelbrot_for_count
mandelbrot_stop:

  mov.w &color, r15
  rla.w r15
  add.w #colors, r15
  mov.w @r15, r15
  swpb r15
  call #lcd_send_data
  swpb r15
  call #lcd_send_data

  ;; r += dx;
  add.w #0x0020, &curr_r
  dec.w &curr_x
  jnz mandelbrot_for_x

  ;; i += dy;
  add.w #0x0020, &curr_i
  dec.w &curr_y
  jnz mandelbrot_for_y

  ;bic.b #SPI_16, &SPI_CTL
  ret

;; lcd_send_cmd(r15)
lcd_send_cmd:
  mov.b r15, &SPI_TX
  bis.b #SPI_START, &SPI_CTL
lcd_send_cmd_wait:
  bit.b #SPI_BUSY, &SPI_CTL
  jnz lcd_send_cmd_wait
  ret

;; lcd_send_data(r15)
lcd_send_data:
  bis.b #LED0, &PORT0
  bis.b #LCD_DC, &SPI_IO
  bic.b #LCD_CS, &SPI_IO

  mov.b r15, &SPI_TX

  bis.b #SPI_START, &SPI_CTL
lcd_send_data_wait:
  bit.b #SPI_BUSY, &SPI_CTL
  jnz lcd_send_data_wait
  bis.b #LCD_CS, &SPI_IO
  ret

delay:
  mov.w #0xffff, r15
delay_loop:
  sub.w #1, r15
  jnz delay_loop
  ret

toggle_led:
  xor.b #LED0, &PORT0
  ret

send_init_data:
  mov.w #init_data_end - init_data, r4
  mov.w #init_data, r5
  bic.b #LCD_DC | LCD_CS, &SPI_IO
send_init_data_loop:
  mov.b @r5+, r15
  call #lcd_send_cmd
  dec.w r4
  jnz send_init_data_loop
  bis.b #LCD_CS, &SPI_IO
  ret

init_data:
  .db SSD1331_DISPLAY_OFF
  .db SSD1331_SET_REMAP
  .db 0x72
  .db SSD1331_START_LINE
  .db 0x00
  .db SSD1331_DISPLAY_OFFSET
  .db 0x00
  .db SSD1331_DISPLAY_NORMAL
  .db SSD1331_SET_MULTIPLEX
  .db 0x3f
  .db SSD1331_SET_MASTER
  .db 0x8e
  .db SSD1331_POWER_MODE
  .db SSD1331_PRECHARGE
  .db 0x31
  .db SSD1331_CLOCKDIV
  .db 0xf0
  .db SSD1331_PRECHARGE_A
  .db 0x64
  .db SSD1331_PRECHARGE_B
  .db 0x78
  .db SSD1331_PRECHARGE_C
  .db 0x64
  .db SSD1331_PRECHARGE_LEVEL
  .db 0x3a
  .db SSD1331_VCOMH
  .db 0x3e
  .db SSD1331_MASTER_CURRENT
  .db 0x06
  .db SSD1331_CONTRAST_A
  .db 0x91
  .db SSD1331_CONTRAST_B
  .db 0x50
  .db SSD1331_CONTRAST_C
  .db 0x7d
  .db SSD1331_DISPLAY_ON
init_data_end:

colors:
  .dc16 0xf800
  .dc16 0xe980
  .dc16 0xcaa0
  .dc16 0xaaa0
  .dc16 0xa980
  .dc16 0x6320
  .dc16 0x9cc0
  .dc16 0x64c0
  .dc16 0x34c0
  .dc16 0x04d5
  .dc16 0x0335
  .dc16 0x0195
  .dc16 0x0015
  .dc16 0x0013
  .dc16 0x000c
  .dc16 0x0000

.org 0xfffe
  .dw start
