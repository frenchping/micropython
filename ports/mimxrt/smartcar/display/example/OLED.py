
import time
import font
from machine import SPI
from machine import Pin

X_WIDTH = 128
Y_WIDTH = 64

led = SPI(0)

RST = Pin('GPIO_AD_B0_08', Pin.OUT)
DC = Pin('GPIO_AD_B0_13', Pin.OUT)

Init_Cmds = (
    0xAE, # turn off oled panel
    0x00, # set low column address
    0x10, # set high column address
    0x40, # set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    0x81, # set contrast control register
    0x7f, # Set SEG Output Current Brightness
    0xA1, # Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    0xC8, # Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    
    0xA6, # set normal display
    0xa8, # set multiplex ratio(1 to 64)
	0x3f, # 1/64 duty
	0xd3, # set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	0x00, # not offset
	0xd5, # set display clock divide ratio/oscillator frequency
	0x80, # set divide ratio, Set Clock as 100 Frames/Sec
	0xd9, # set pre-charge period
	0xf1, # Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	0xda, # set com pins hardware configuration
	0x12,
	0xdb, # set vcomh
	0x40, # Set VCOM Deselect Level
	0x20, # Set Page Addressing Mode (0x00/0x01/0x02)
	0x02,
	0x8d, # set Charge Pump enable/disable
	0x14, # set(0x10) disable
	0xa4, # Disable Entire Display On (0xa4/0xa5)
	0xa6, # Disable Inverse Display On (0xa6/a7)
	0xaf, # turn on oled panel
)

def oled_wrdat(data):
    DC.high()
    buf = bytearray(1)
    buf[0] = data
    led.write(buf)
    
def oled_wrcmd(cmd):
    DC.low()
    buf = bytearray(1)
    buf[0] = cmd
    led.write(buf)

def oled_set_pos(x, y):
    DC.low()
    pos = bytearray( (0xb0+y, ((x & 0xf0) >> 4) | 0x10, x & 0x0F) )
    led.write(pos)
    
def oled_fill(data):
    for y in range(8):
        DC.low()
        buf = bytearray((0xb0+y, 0x01, 0x10))
        led.write(buf)
        DC.high()
        buf = bytearray((data,)*X_WIDTH)
        led.write(buf)

def oled_str6(x, y, data):
    pos = bytearray(3)
    pos[0]= 0xb0 + y;
    pos[1] = ((x & 0xf0) >> 4) | 0x10
    pos[2] = x & 0x0F
    DC.low()
    led.write(pos)
    for ch in data:
        x += 6
        if x > X_WIDTH:
            break
        ft = font.oled6x8(ch)
        DC.high()
        led.write(ft)

def oled_str8(x, y, data):
    pos = bytearray(3)
    for ch in data:
        if x+8 > X_WIDTH:
            break
        ft = font.oled8x16(ch)
        
        pos[0] = 0xb0 + y;
        pos[1] = ((x & 0xf0) >> 4) | 0x10
        pos[2] = x & 0x0F
        DC.low()
        led.write(pos)

        DC.high()
        led.write(ft[:8])
        
        pos[0] += 1
        DC.low()
        led.write(pos)
        DC.high()
        led.write(ft[8:])
        
        x += 8

def oled_init():
    RST.low()
    time.sleep_ms(50)  # Wait 50ms
    RST.high()
    
    buf = bytearray(Init_Cmds)
    DC.low()
    led.write(buf)
    # for cmd in Init_Cmds:
    #     oled_wrcmd(cmd)
    oled_fill(0xff)

oled_init()

        