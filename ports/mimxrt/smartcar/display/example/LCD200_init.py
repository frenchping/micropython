# 这里定义了一个函数，初始化LCD驱动并返回LCD实体

from display import *
from machine import Pin

PANEL_WIDTH = 320
PANEL_HEIGHT= 240

def init_lcd():
    BLK = Pin('GPIO_AD_B0_02', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)

    # 定义DC和Rst引脚
    dc = Pin('GPIO_SD_B1_04', Pin.OUT, pull=Pin.PULL_UP_22K, value=1)
    rst = Pin('GPIO_AD_B0_13', Pin.OUT, pull=Pin.PULL_UP_22K, value=1)

    # 建立LCD_Drv的实体
    # 使用SPI 0（对应芯片的LPSPI1，引脚：PCS0 = GPIO_AD_B0_11, SDO = GPIO_AD_B0_12, SCK = GPIO_AD_B0_10)
    # LCD模块是预定义的一款320x240的2寸屏，驱动芯片为ST7789
    drv = LCD_Drv(SPI_INDEX=0, BAUDRATE=25000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)

    # 获得LCD实体
    return LCD(drv)
