# 这个例子演示如何获取字符点阵，并映射到framebuf中，最终显示到屏幕
from LCD200_init import init_lcd
from display import font

import framebuf

# 获得LCD实体
lcd = init_lcd()
lcd.clear(0)

# 创建一个200x100的framebuf
WIDTH = 260
HEIGHT= 60
xbuf = framebuf.FrameBuffer(bytearray(WIDTH * HEIGHT * 2), WIDTH, HEIGHT, framebuf.RGB565)
xbuf.fill(0xFFE0)   #Yellow

# 创建一个只有2个颜色的调色板，分别为浅蓝色和红色
import struct
palette = bytearray(struct.pack('HH', 0x7FF, 0xF800))
plt = framebuf.FrameBuffer(palette, 2, 1, framebuf.RGB565)

y = 20
x = 10
font_width = 16
font_height= 32
for cc in "Hello World! ":
    ch = font.lcd_16x32(cc)     # 获取大小为16x32的字库
    # 将字库数据转换为一个framebuf
    chbuf = framebuf.FrameBuffer(ch, font_width, font_height, framebuf.MONO_HMSB)
    # 把字库对应的framebuf，拷贝到整体的framebuf
    xbuf.blit(chbuf, x, y, 0, plt)
    x += font_width

# 把已经在framebuf中写好的字串，显示到屏幕上
lcd.blit(10, 50, WIDTH, HEIGHT, xbuf)
