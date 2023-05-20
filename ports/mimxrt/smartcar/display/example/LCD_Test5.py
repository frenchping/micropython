# 这个例子演示用随机颜色在屏幕画色块

from LCD200_init import *
import random
import time

# 获得LCD实体
lcd = init_lcd()
lcd.clear(0)    # 黑色清屏

################################

import framebuf
LOOP = 10000

# 无限循环，显示随机矩形，线条颜色是随机的
# Ctrl+C 可以停止运行
start = time.ticks_ms()
for loop in range(LOOP):
    x = random.getrandbits(9) % PANEL_WIDTH
    y = random.getrandbits(8) % PANEL_HEIGHT
    w = random.getrandbits(6) + 10
    h = random.getrandbits(6) + 10
    color=random.getrandbits(16)
    xbuf = framebuf.FrameBuffer(bytearray(w * h * 2), w, h, framebuf.RGB565)
    xbuf.fill(color)
    lcd.blit(x, y, w, h, xbuf)
print("Spending time: %dms in loop=%d" % (time.ticks_ms() - start, LOOP))
