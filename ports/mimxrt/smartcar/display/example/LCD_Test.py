
import time
from machine import SPI
from display import *
import random


time.sleep_ms(100)

# 定义DC和Rst引脚
dc = Pin('B9', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
rst = Pin('B8', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)

# 建立LCD_Drv的实体，使用SPI 0（对应芯片的LPSPI1，引脚：PCS0=）
drv = LCD_Drv(SPI_INDEX=0, BAUDRATE=15000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
lcd = LCD(drv)
lcd.clear(0xF800)


import framebuf
fbuf = framebuf.FrameBuffer(bytearray(100 * 10 * 2), 100, 10, framebuf.RGB565)
fbuf.fill(0)
fbuf.text('MicroPython!', 0, 0, 0x1f)
fbuf.hline(0, 9, 96, 0xffff)
lcd.blit(100, 100, 100, 10, fbuf)

################
import framebuf
WIDTH = 200
HEIGHT= 100
xbuf = framebuf.FrameBuffer(bytearray(WIDTH * HEIGHT * 2), WIDTH, HEIGHT, framebuf.RGB565)
xbuf.fill(0xFFE0)   #Yellow
lcd.str24(0,0,"Hello World", 0xF800, xbuf)
lcd.blit(10, 0, WIDTH, HEIGHT, xbuf)

lcd.str24(0,30,"Hello World", 0xF80, xbuf)
lcd.str24(0,60,"Hello World", 0xF8, xbuf)
lcd.str24(0,90,"Hello World", 0xF800, xbuf)
lcd.blit(30, 210, WIDTH, HEIGHT, xbuf)  # Show above 3 lines

### Draw lines with random colors
while True:
    lcd.clear(0)
    for y in range(0,240,2):
        lcd.line(0,0,319,y,color=random.getrandbits(16),thick=2)
    for x in range(320,0, -2):
        lcd.line(0,0,x,240,color=random.getrandbits(16),thick=2)

lcd.line(0,0,100,100,color=0, thick=2)
lcd.line(0,0,200,100,color=0x7E0, thick=2)
lcd.line(0,0,200,50,color=0xFFFF)




cbuf = framebuf.FrameBuffer(bytearray(100 * 24 * 2), 100, 24, framebuf.RGB565)
cbuf.fill(0xFFE0)   # Yellow
lcd.str24(0, 0, "HELLO", 0xF800, cbuf)
lcd.blit(100, 100, 100, 24, cbuf)






import random
while True:
    for y in range(0, 240, 16):
        lcd.str16(0, y, "HELLO WORLD 1234567890 abcdefghijklmnopqrstuvwxyz", random.getrandbits(16), fbuf)



##########################
# 以下测试使用framebuf显示Hello World
import struct
bf = bytearray(struct.pack('HH', 0x7FF, 0xF800))
plt = framebuf.FrameBuffer(bf, 2, 1, framebuf.RGB565)

y = 20
x = 10
for cc in "Hello World! ":
    ch = font.lcd_12x24(cc)
    chbuf = framebuf.FrameBuffer(ch, 12, 24, framebuf.MONO_HMSB)
    xbuf.blit(chbuf, x, y, 0, plt)
    x += 12
lcd.blit(10, 0, WIDTH, HEIGHT, xbuf)
