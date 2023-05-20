
init_CMD200 = (
    bytearray((0x3A, 0x05)),
    bytearray((0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33)),
    bytearray((0xB7, 0x35)),
    bytearray((0xBB, 0x29)),    # Vcom=1.35V
    bytearray((0xC2, 0x01)),
    bytearray((0xC3, 0x19)),    # GVDD=4.8V 
    bytearray((0xC4, 0x20)),    # VDV, 0x20:0v
    bytearray((0xC5, 0x1A)),    # VCOM Offset Set
    bytearray((0xC6, 0x1F)),    # 0x0F:60Hz
    bytearray((0xD0, 0xA4, 0xA1)),
    bytearray((0xE0, 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34)),
    bytearray((0xE1, 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34)),
    b'\x21',
)
mode_CMD200 = (
    b'\x36\xC0',    # ORIENT_0
    b'\x36\xA0',    # ORIENT_90
    b'\x36\x00',    # ORIENT_180
    b'\x36\x60',    # ORIENT_270
    b'\x36\xD0',    # ORIENT_MIRROR_0
    b'\x36\xB0',    # ORIENT_MIRROR_90
    b'\x36\x10',    # ORIENT_MIRROR_180
    b'\x36\x70',    # ORIENT_MIRROR_270
)

from display import LCD, LCD_Drv
import time
class LCD200_Drv(LCD_Drv):
    def init(self):
        print("Hello init")
        self.write_cmd(b'\x11')
        time.sleep_ms(100)
        for cmd in init_CMD200:
            self.write_cmd(cmd)
        time.sleep_ms(100)
        self.write_cmd(b'\x29')

    def set_mode(self, mm):
        print("Hello mode(%d)" % mm)
        self.write_cmd(mode_CMD200[mm])

    def set_area(self, x0, y0, width, height):
        # print(x0, y0, width, height)
        # print("set_area: x0=%d, y0=%d, w=%d, h=%d" % (x0, y0, width, height))
        x1 = x0 + width - 1
        cmdX = bytearray( (0x2A, x0>>8, x0, x1>>8, x1))
        self.write_cmd(cmdX)
        y1 = y0 + height - 1
        cmdY = bytearray( (0x2B, y0>>8, y0, y1>>8, y1))
        self.write_cmd(cmdY)
        self.write_cmd(b'\x2C')

dc  = Pin('J2_28', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
rst = Pin('J2_26', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)

drv = LCD200_Drv(SPI_INDEX=0, BAUDRATE=15000000, DC_PIN=dc, RST_PIN=rst, LCD_WIDTH=320, LCD_HEIGHT=240, LCD_TYPE=-1)
lcd = LCD(drv)
lcd.clear(0xF800)


#######################################
init_CMD154 = (
    b'\x11',
    bytearray((0x3A, 0x05)),
    bytearray((0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33)),
    bytearray((0xB7, 0x35)),
    bytearray((0xBB, 0x32)),    # VCOM Setting
    bytearray((0xC0, 0x2C)),	# ??
    bytearray((0xC2, 0x01)),
    bytearray((0xC3, 0x15)),	# VRH Setting
    bytearray((0xC4, 0x20)),
    bytearray((0xC6, 0x0F)),
    bytearray((0xD0, 0xA4, 0xA1)),
    bytearray((0xE0, 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34)),
    bytearray((0xE1, 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34)),
    b'\x21',
    b'\x11',
    b'\x29'
)

class LCD154_Drv(RT1020.LCD_Drv):
    def init(self):
        print("Hello init")
        for cmd in init_CMD154:
            self.write_cmd(cmd)

    def set_mode(self, mm):
        print("Hello mode(%d)" % mm)
        mode_cmd = (
            b'\x36\x00',    # ORIENT_0
            b'\x36\x60',    # ORIENT_90
            b'\x36\xC0',    # ORIENT_180
            b'\x36\xA0',    # ORIENT_270
            b'\x36\x80',    # ORIENT_MIRROR_0
            b'\x36\x20',    # ORIENT_MIRROR_90
            b'\x36\x40',    # ORIENT_MIRROR_180
            b'\x36\xE0',    # ORIENT_MIRROR_270
        )
        self.write_cmd(mode_cmd[mm])

    def set_area(self, x0, y0, width, height):
        # print(x0, y0, width, height)
        # print("set_area: x0=%d, y0=%d, w=%d, h=%d" % (x0, y0, width, height))
        cmdX = bytearray( (0x2A, 0, x0, 0, x0+width-1))
        self.write_cmd(cmdX)
        cmdY = bytearray( (0x2B, 0, y0, 0, y0+height-1))
        self.write_cmd(cmdY)
        self.write_cmd(b'\x2C')

drv = LCD154_Drv(SPI_INDEX=3, DC_PIN='AD_B1_15', RST_PIN='AD_B1_10', LCD_WIDTH=240, LCD_HEIGHT=240, LCD_TYPE=-1)
