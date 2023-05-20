# 这个例程演示了直接使用machine.SPI和Pin驱动一个LCD屏幕

import time
import machine

LCD200_WIDTH  = 320
LCD200_HEIGHT = 240

LCD200_InitCmd = (
    (0x3A, 0x05),
    (0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33),
    (0xB7, 0x35),
    (0xBB, 0x29),   # Vcom=1.35V
    (0xC2, 0x01),
    (0xC3, 0x19),   # GVDD=4.8V 
    (0xC4, 0x20),   # VDV, 0x20:0v
    (0xC5, 0x1A),   # VCOM Offset Set
    (0xC6, 0x1F),   # 0x0F:60Hz
    (0xD0, 0xA4, 0xA1),
    (0xE0, 0xD0, 0x08, 0x0E, 0x09,
        0x09, 0x05, 0x31, 0x33,
        0x48, 0x17, 0x14, 0x15,
        0x31, 0x34 ),
    (0xE1, 0xD0, 0x08, 0x0E, 0x09,
        0x09, 0x15, 0x31, 0x33,
        0x48, 0x17, 0x14, 0x15,
        0x31, 0x34 ),
    (0x21, )
)

LCD200_MODE = (
    0b11000000,     # ORIENT_0
    0b10100000,     # ORIENT_90
    0b00000000,     # ORIENT_180
    0b01100000,     # ORIENT_270
                # MIRROR
    0b11010000,     # ORIENT_0
    0b10100100,     # ORIENT_90
    0b00010000,     # ORIENT_180
    0b01100100,     # ORIENT_270
)

class LCD200():
    def __init__(self, spi, dc_pin, cs_pin, rst_pin = None):
        cs = machine.Pin(cs_pin, machine.Pin.OUT)
        cs.high()
        time.sleep_ms(10)  # Wait 10ms
        self.lcd = machine.SPI(spi, baudrate=4000000, polarity=1, phase=1, cs=0)
        self.dc = machine.Pin(dc_pin, machine.Pin.OUT)
        # cs.low()

        if rst_pin:
            self.rst = machine.Pin(rst_pin, machine.Pin.OUT)
            self.rst.low()
            time.sleep_ms(100)  # Wait 100ms
            self.rst.high()

        self.wreg(0x11)     # sleep out
        time.sleep_ms(100)  # Wait 100ms

        for cmd in LCD200_InitCmd:
            self.wcmd(bytearray(cmd))
        time.sleep_ms(100)  # Wait 100ms

        self.wreg(0x29)     # Turn on the LCD display

        self.set_mode(1)

    def wreg(self, val):
        reg_buf = bytearray(1)
        reg_buf[0] = val
        self.dc.low()
        self.lcd.write(reg_buf)
        self.dc.high()

    def wdat(self, data):
        self.lcd.write(data)

    def wcmd(self, cmd):
        self.wreg(cmd[0])
        if len(cmd) > 1:
            self.wdat(cmd[1:])

    def set_mode(self, mode):
        if mode > 7:
            return
        self.mode = mode
        self.wcmd(bytearray( (0x36, LCD200_MODE[mode]) ))
        if mode & 1:
            self.width = LCD200_WIDTH
            self.height= LCD200_HEIGHT
        else:
            self.width = LCD200_HEIGHT
            self.height= LCD200_WIDTH
        
    def set_win(self, x0, y0, wd, hg):
        right = x0 + wd
        self.wcmd(bytearray( (0x2A, x0 >> 8, x0 & 0xFF, right >> 8, right & 0xFF)))
        bottom = y0 + hg
        self.wcmd(bytearray( (0x2B, y0 >> 8, y0 & 0xFF, bottom >> 8, bottom & 0xFF)))
        self.wreg(0x2C)
    
    def fill(self, data):
        self.set_win(0, 0, self.width, self.height)
        self.buf = bytearray( (data >> 8, data & 0xFF) * self.width)
        for _ in range(self.height):
            self.wdat(self.buf)


lcd = LCD200(2, 'GPIO_EMC_35', 'GPIO_EMC_33', 'GPIO_EMC_27')

lcd.fill(0xF8)
