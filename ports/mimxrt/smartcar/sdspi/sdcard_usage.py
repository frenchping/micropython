import os, sdcard, machine

cs_pin = "GPIO_EMC_33"
spi = machine.SPI(2) # SPI2 with cs at Pin "GPIO_EMC_33" used for SDCARD
cs = machine.Pin(cs_pin, machine.Pin.OUT, value=1)
sd = sdcard.SDCard(spi, cs)
vfs = os.VfsFat(sd)
os.mount(vfs, "/sdcard")





