# 这个例程演示了使用自定义的LCD_Drv，驱动一个非预定义的1.80寸屏幕
# 这个屏的驱动芯片是ST7735，分辨率是160x128
# 这里还演示了不同显示方向时需要进行不同位移补偿

from display import *
import time
PANEL_WIDTH = 160
PANEL_HEIGHT= 128

init_CMD180 = (
    bytearray((0xB1, 0x01, 0x2C, 0x2D)),
    bytearray((0xB2, 0x01, 0x2C, 0x2D)),
    bytearray((0xB3, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D)),
    bytearray((0xB4, 0x07)),
    bytearray((0xC0, 0xA2, 0x02, 0x84)),
    bytearray((0xC1, 0xC5)),
    bytearray((0xC2, 0x0A, 0x00)),
    bytearray((0xC3, 0x8A, 0x2A)),
    bytearray((0xC4, 0x8A, 0xEE)),
    bytearray((0xC5, 0x0E)),
    # bytearray((0x36, (1<<7 | 1<<6 | 0<<5))),  # Vertical
    bytearray((0xE0, 0x0F, 0x1A, 0x0F, 0x18,
                0x2F, 0x28, 0x20, 0x22,
                0x1F, 0x1B, 0x23, 0x37,
                0x00, 0x07, 0x02, 0x10)),
    bytearray((0xE1, 0x0F, 0x1B, 0x0F, 0x17,
                0x33, 0x2C, 0x29, 0x2E,
                0x30, 0x30, 0x39, 0x3F,
                0x00, 0x07, 0x03, 0x10)),
    bytearray((0x2A, 0x00, 0x02, 0x00, 0x82)),
    bytearray((0x2B, 0x00, 0x03, 0x00, 0x83)),
    bytearray((0xF0, 0x01)),
    bytearray((0xF6, 0x00)),
    bytearray((0x3A, 0x05)),
    b'\x29'
)

class LCD180Drv(LCD_Drv):
    def init(self):
        # print("Hello init")
        self.write_cmd(b'\x11')
        time.sleep_ms(100)
        for cmd in init_CMD180:
            self.write_cmd(cmd)

    def set_mode(self, mm):
        self.mode = mm      # 记录显示方向，在显示时需要进行位移补偿
        mode_cmd = (
            b'\x36\xC0',    # ORIENT_0
            b'\x36\xA0',    # ORIENT_90
            b'\x36\x00',    # ORIENT_180
            b'\x36\x60',    # ORIENT_270
            b'\x36\xD0',    # ORIENT_MIRROR_0
            b'\x36\xB0',    # ORIENT_MIRROR_90
            b'\x36\x10',    # ORIENT_MIRROR_180
            b'\x36\x70',    # ORIENT_MIRROR_270
        )
        self.write_cmd(mode_cmd[mm])

    def set_area(self, x0, y0, width, height):
        left = x0 + (1 if self.mode & 1 else 2)     # 需要根据不同的方向进行位移补偿
        right= left + width - 1
        cmdX = bytearray( (0x2A, 0, left, 0, right))
        self.write_cmd(cmdX)
        top = y0 + (2 if self.mode & 1 else 1)      # 需要根据不同的方向进行位移补偿
        bottom = top + height - 1
        cmdY = bytearray( (0x2B, 0, top, 0, bottom))
        self.write_cmd(cmdY)
        self.write_cmd(b'\x2C')


dc  = Pin('J3_40', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
rst = Pin('J3_36', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
drv = LCD180Drv(SPI_INDEX=2, BAUDRATE=24000000, DC_PIN=dc, RST_PIN=rst, LCD_WIDTH=PANEL_WIDTH, LCD_HEIGHT=PANEL_HEIGHT, LCD_TYPE=-1)
lcd = LCD(drv)

lcd.clear(0xF800)
