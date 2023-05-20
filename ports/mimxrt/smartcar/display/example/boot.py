from machine import UART 
import os
u1 = UART(0, 115200) 
os.dupterm(u1)
