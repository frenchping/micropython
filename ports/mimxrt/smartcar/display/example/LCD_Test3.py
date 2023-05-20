# 这个例子演示如何输出字串，可以直接输出到屏幕，也可以输出到framebuf，再显示到屏幕

from LCD200_init import init_lcd

# 获得LCD实体
lcd = init_lcd()

lcd.clear(0)    # 黑色清屏

# 设置颜色
lcd.color(0xFFFF, 0xF800)           # 前景=白色，背景=红色

# 输出字串
lcd.str24(0,0,"Hello World!")        # 坐标(0,0) 显示字符串
lcd.str16(200,20,"Hello World!")     # 坐标(200,20) 显示字符串
lcd.str32(100,50,"Hello World!")     # 坐标(100,50) 显示字符串

# 在framebuf输出字串，再显示到屏幕
import framebuf
WIDTH = 260
HEIGHT= 40
xbuf = framebuf.FrameBuffer(bytearray(WIDTH * HEIGHT * 2), WIDTH, HEIGHT, framebuf.RGB565)
xbuf.fill(0xFFE0)           # 黄色
lcd.str32(4, 4, "Hello framebuf!", 0xF800, xbuf)   # 红色字串
lcd.blit(10, 100, WIDTH, HEIGHT, xbuf)          # 将framebuf显示到屏幕(10,100)

lcd.blit(50, 150, WIDTH, HEIGHT, xbuf)          # 同一个framebuf显示到屏幕的不同位置
