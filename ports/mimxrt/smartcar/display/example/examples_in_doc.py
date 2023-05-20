
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
lcd.clear(0xF800) # 红色清屏 

# 以下使用标准framebuf类的例子 
import framebuf 
fbuf = framebuf.FrameBuffer(bytearray(100 * 10 * 2), 100, 10, framebuf.RGB565) 
fbuf.fill(0) 
fbuf.text('MicroPython!', 0, 0, 0x1f) 
fbuf.hline(0, 9, 96, 0xffff) 

# 将framebuf里的内容显示到屏幕的(100, 100)处
lcd.blit(100, 100, 100, 10, fbuf)

###########################################

from display import * 
from machine import Pin

PANEL_WIDTH = 240
PANEL_HEIGHT= 240

# 定义一个tuple，包含一系列的初始化命令
init_CMD154 = ( 
    b'\x11', 
    bytearray((0x3A, 0x05)), 
    bytearray((0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33)), 
    bytearray((0xB7, 0x35)), bytearray((0xBB, 0x32)), # VCOM Setting 
    bytearray((0xC0, 0x2C)), # ?? 
    bytearray((0xC2, 0x01)), 
    bytearray((0xC3, 0x15)), # VRH Setting 
    bytearray((0xC4, 0x20)), 
    bytearray((0xC6, 0x0F)), 
    bytearray((0xD0, 0xA4, 0xA1)), 
    bytearray((0xE0, 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34)), 
    bytearray((0xE1, 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34)), 
    b'\x21', 
    b'\x29'
)

class LCD154_Drv(LCD_Drv): 
    def init(self): 
        for cmd in init_CMD154: 
            self.write_cmd(cmd) 
            
    def set_mode(self, mm): 
        self.mode = mm 
        mode_cmd = ( 
            b'\x36\x00', # ORIENT_0 
            b'\x36\x60', # ORIENT_90 
            b'\x36\xC0', # ORIENT_180 
            b'\x36\xA0', # ORIENT_270 
            b'\x36\x80', # ORIENT_MIRROR_0 
            b'\x36\x20', # ORIENT_MIRROR_90 
            b'\x36\x40', # ORIENT_MIRROR_180 
            b'\x36\xE0', # ORIENT_MIRROR_270
        ) 
        self.write_cmd(mode_cmd[mm]) 
        
    def set_area(self, x0, y0, width, height): 
        if self.mode == 2: # 按照显示方向，调整坐标 
            y0 += 320-240 
        if self.mode == 3: # 按照显示方向，调整坐标 
            x0 += 320-240 
        right = x0 + width - 1 
        cmdX = bytearray( (0x2A, x0>>8, x0, right>>8, right)) 
        self.write_cmd(cmdX) 
        bottom = y0 + height - 1 
        cmdY = bytearray( (0x2B, y0>>8, y0, bottom>>8, bottom)) 
        self.write_cmd(cmdY) 
        self.write_cmd(b'\x2C')
    
def init_lcd():
    BLK = Pin('GPIO_AD_B0_02', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
    
    # 定义DC和Rst引脚 
    dc = Pin('GPIO_SD_B1_04', Pin.OUT, pull=Pin.PULL_UP_22K, value=1) 
    rst = Pin('GPIO_AD_B0_13', Pin.OUT, pull=Pin.PULL_UP_22K, value=1) 
    
    # 建立LCD_Drv的实体 
    # 使用SPI 0（对应芯片的LPSPI1，引脚：PCS0 = GPIO_AD_B0_11, SDO = GPIO_AD_B0_12, SCK = GPIO_AD_B0_10) 
    # LCD模块是自定义的一款240x240的1.54寸屏，驱动芯片为ST7789 
    drv = LCD154_Drv(SPI_INDEX=0, BAUDRATE=25000000, DC_PIN=dc, RST_PIN=rst, 
                                    LCD_WIDTH=240, LCD_HEIGHT=240, LCD_TYPE=-1)
                                    
    # 返回LCD实体 
    return LCD(drv)

#####################################################
# 这个例子演示用随机颜色画线条 
"""
from LCD154_init import * 
import random 
import time 

# 获得LCD实体 
lcd = init_lcd() 

# 无限循环，显示线条，线条颜色是随机的 
while True:
    lcd.clear(0) # 黑色清屏
    start = time.ticks_ms()
    for y in range(0, PANEL_HEIGHT, 2):
        lcd.line(0, y, PANEL_WIDTH-1, y, color=random.getrandbits(16), thick=2)
    print("Spending time: %dms" % (time.ticks_ms() - start)) # 计算并显示完成一屏的时间
"""
#########################################################
# 这个例子演示如何输出字串，可以直接输出到屏幕，也可以输出到framebuf，再显示到屏幕
from LCD154_init import init_lcd

# 获得LCD实体
lcd = init_lcd()
lcd.clear(0) # 黑色清屏 

# 设置颜色
lcd.color(0xFFFF, 0xF800) # 前景=白色，背景=红色

# 输出字串
lcd.str24(0,0,"Hello World!")       # 坐标(0,0) 显示字符串
lcd.str16(200,20,"Hello World!")    # 坐标(200,20) 显示字符串
lcd.str32(100,50,"Hello World!")    # 坐标(100,50) 显示字符串

# 在framebuf输出字串，再显示到屏幕
import framebuf 
WIDTH = 200 
HEIGHT= 40
xbuf = framebuf.FrameBuffer(bytearray(WIDTH * HEIGHT * 2), WIDTH, HEIGHT, framebuf.RGB565)
xbuf.fill(0xFFE0)       # 黄色
lcd.str32(4, 4, "Hello World!", 0xF800, xbuf)   # 红色字串
lcd.blit(10, 100, WIDTH, HEIGHT, xbuf)  # 将framebuf显示到屏幕(10,100)

lcd.blit(50, 150, WIDTH, HEIGHT, xbuf)  # 同一个framebuf显示到屏幕的不同位置

###############################################

# 这个例子演示如何获取字符点阵，并映射到framebuf中，最终显示到屏幕
from LCD200_init import init_lcd 
from display import font 
import framebuf

# 获得LCD实体
lcd = init_lcd()
lcd.clear(0xF800) # 红色清屏

# 创建一个200x100的framebuf
WIDTH = 260 
HEIGHT= 60 
xbuf = framebuf.FrameBuffer(bytearray(WIDTH * HEIGHT * 2), WIDTH, HEIGHT, framebuf.RGB565) 
xbuf.fill(0xFFE0)       # 黄色填充 

# 创建一个只有2个颜色的调色板，分别为浅蓝色和红色 
import struct
palette = bytearray(struct.pack('HH', 0x7FF, 0xF800))
plt = framebuf.FrameBuffer(palette, 2, 1, framebuf.RGB565)

y = 20
x = 10
font_width = 16

font_height= 32
for cc in "Hello World! ":  # 循环处理每一个字符
    ch = font.lcd_16x32(cc) # 获取大小为16x32的字库
    # 将字库数据转换为一个framebuf
    chbuf = framebuf.FrameBuffer(ch, font_width, font_height, framebuf.MONO_HMSB)
    # 把字库对应的framebuf，拷贝到整体的framebuf
    xbuf.blit(chbuf, x, y, 0, plt)
    x += font_width

# 把已经在framebuf中写好的字串，显示到屏幕上
lcd.blit(10, 50, WIDTH, HEIGHT, xbuf)

########################################################
# 这个例子演示用随机颜色在屏幕画色块
from LCD200_init import * 
import random
import time

# 获得LCD实体
lcd = init_lcd()
lcd.clear(0)    # 黑色清屏

import framebuf
LOOP = 10000

# 显示随机矩形，线条颜色是随机的
start = time.ticks_ms()
for loop in range(LOOP):
    x = random.getrandbits(9) % PANEL_WIDTH     # 生成随机的x坐标
    y = random.getrandbits(8) % PANEL_HEIGHT    # 生成随机的y坐标
    w = random.getrandbits(6) + 10              # 生成随机的色块宽度，范围是10~2^6
    h = random.getrandbits(6) + 10              # 生成随机的色块高度，范围是10~2^6
    color=random.getrandbits(16)                # 生成随机的色块颜色
    xbuf = framebuf.FrameBuffer(bytearray(w * h * 2), w, h, framebuf.RGB565)
    xbuf.fill(color)        # 用随机生成的颜色填充缓冲区
    lcd.blit(x, y, w, h, xbuf)
print("Spending time: %dms in loop=%d" % (time.ticks_ms() - start, LOOP))
