# 这个例子演示如何配置显示驱动和屏幕操作
# 还演示了如何使用micropython内置的framebuf，并显示到屏幕

from display import *

# 定义DC和Rst引脚
dc = Pin('J2_28', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
rst = Pin('J2_26', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)

# 建立LCD_Drv的实体
# 使用SPI 0（对应芯片的LPSPI1，引脚：PCS0 = GPIO_AD_B0_11, SDO = GPIO_AD_B0_12, SCK = GPIO_AD_B0_10)
# LCD模块是预定义的一款320x240的2寸屏，驱动芯片为ST7789
drv = LCD_Drv(SPI_INDEX=0, BAUDRATE=15000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)

# 获得LCD实体
lcd = LCD(drv)
lcd.clear(0xF800)   # 红色清屏

# 以下使用标准framebuf类的例子
import framebuf
fbuf = framebuf.FrameBuffer(bytearray(100 * 10 * 2), 100, 10, framebuf.RGB565)
fbuf.fill(0)
fbuf.text('MicroPython!', 0, 0, 0x1f)
fbuf.hline(0, 9, 96, 0xffff)

# 将framebuf里的内容显示到屏幕的(100, 100)处
lcd.blit(100, 100, 100, 10, fbuf)
