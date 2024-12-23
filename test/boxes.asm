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
SPI_16     equ 4

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
  ;call #lcd_clear
  call #delay
  call #draw_horizon
  call #delay
  call #draw_ground
  call #delay
  call #draw_landing_pad
  call #delay

while_1:
  bit.b #1, &BUTTON
  jnz run

  call #delay
  call #toggle_led
  jmp while_1

run:
  call #lcd_clear_2
  call #draw_boxes
  jmp while_1

lcd_init:
  mov.b #0, &SPI_CTL
  mov.b #LCD_CS, &SPI_IO
  call #delay
  mov.b #LCD_CS | LCD_RES, &SPI_IO

  call #send_init_data
  ret

lcd_clear:
  bis.b #SPI_16, &SPI_CTL
  mov.w #(96 * 64), r4
lcd_clear_loop:
  mov.w #0x0f0f, r15
  call #lcd_send_data
  dec.w r4
  jnz lcd_clear_loop
  bic.b #SPI_16, &SPI_CTL
  ret

lcd_clear_2:
  bis.b #SPI_16, &SPI_CTL
  mov.w #(96 * 64), r4
lcd_clear_loop_2:
  mov.w #0xf00f, r15
  call #lcd_send_data
  dec.w r4
  jnz lcd_clear_loop_2
  bic.b #SPI_16, &SPI_CTL
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

  mov.w r15, &SPI_TX

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

draw_boxes:
  mov.w #box_data_end - box_data, r4
  mov.w #box_data, r5
  bic.b #LCD_DC | LCD_CS, &SPI_IO
draw_boxes_loop:
  mov.b @r5+, r15
  call #lcd_send_cmd
  dec.w r4
  jnz draw_boxes_loop
  bis.b #LCD_CS, &SPI_IO
  ret

draw_horizon:
  mov.w #horizon_data_end - horizon_data, r4
  mov.w #horizon_data, r5
  bic.b #LCD_DC | LCD_CS, &SPI_IO
draw_horizon_loop:
  mov.b @r5+, r15
  call #lcd_send_cmd
  dec.w r4
  jnz draw_horizon_loop
  bis.b #LCD_CS, &SPI_IO
  ret

draw_ground:
  mov.w #ground_data_end - ground_data, r4
  mov.w #ground_data, r5
  bic.b #LCD_DC | LCD_CS, &SPI_IO
draw_ground_loop:
  mov.b @r5+, r15
  call #lcd_send_cmd
  dec.w r4
  jnz draw_ground_loop
  bis.b #LCD_CS, &SPI_IO
  ret

draw_landing_pad:
  mov.w #landing_pad_data_end - landing_pad_data, r4
  mov.w #landing_pad_data, r5
  bic.b #LCD_DC | LCD_CS, &SPI_IO
draw_landing_pad_loop:
  mov.b @r5+, r15
  call #lcd_send_cmd
  dec.w r4
  jnz draw_landing_pad_loop
  bis.b #LCD_CS, &SPI_IO
  ret

box_data:
  .db SSD1331_FILL_ENABLE
  .db 0x01, 0x00
  .db SSD1331_DRAW_RECT
  .db 0x10, 0x10, 0x40, 0x30
  .db 0x3e, 0x3f, 0x3e
  .db 0x00, 0x00, 40
box_data_end:

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

  .db SSD1331_FILL_ENABLE
  .db 0x01, 0x00
init_data_end:

horizon_data:
  ;.db SSD1331_FILL_ENABLE
  ;.db 0x01, 0x00
  .db SSD1331_DRAW_RECT
  .db    0,   21,   95,  63
  .db 0x00, 0x00, 0x00
  .db 0x00, 0x00, 0x00
horizon_data_end:

ground_data:
  ;.db SSD1331_FILL_ENABLE
  ;.db 0x01, 0x00
  .db SSD1331_DRAW_RECT
  .db    0,    0,   95,  20
  .db 0x10, 0x10, 0x10
  .db 0x10, 0x10, 0x10
ground_data_end:

landing_pad_data:
  ;.db SSD1331_FILL_ENABLE
  ;.db 0x01, 0x00
  .db SSD1331_DRAW_RECT
  .db   40,   10,   56,  20
  .db 0x00, 0x00, 0xff
  .db 0x00, 0x00, 0xff
landing_pad_data_end:

.org 0xfffe
  .dw start

