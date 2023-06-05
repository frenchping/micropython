
from LCD200_init import *

lcd = init_lcd()
lcd.mode(1)
lcd.clear(0xFFFF)

###################
from camera import MT9V032 as CAM
from machine import UART, Pin
import time


# tpin = Pin("GPIO_AD_B0_06", Pin.OUT)

u6 = UART(6, 9600)

cam = CAM(u6)
cam.init()
time.sleep_ms(80)
print(u6.read())

cam.start()

while True:
    cam.grab(lcd)
