# 这个例子演示用随机颜色画线条

from LCD200_init import *
import random
import time

PANEL_WIDTH = 320
PANEL_HEIGHT= 240

# 获得LCD实体
lcd = init_lcd()

# 无限循环，显示线条，线条颜色是随机的
# Ctrl+C 可以停止运行
while True:
    lcd.clear(0)    # 黑色清屏
    start = time.ticks_ms()
    for y in range(0, PANEL_HEIGHT, 2):
        lcd.line(0, y, PANEL_WIDTH-1, y, color=random.getrandbits(16), thick=2)
    print("Spending time: %dms" % (time.ticks_ms() - start))
