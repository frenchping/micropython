from machine import SPI
from SD_SPI import SD_SPI

spi4 = SPI(2, cs=0)   # sw_id = 2; hw_id = SPI4
sd = SD_SPI(spi4)


def read(blk):
    buf = bytearray(512)
    sd.readblocks(blk, buf)
    cnt = 0
    for data in buf:
        print("0x%02x" % data, end=' ')
        cnt += 1
        if (cnt == 16):
            print()
            cnt = 0

try:
    os.stat('/sd')
except:
    try:
        fat = os.VfsFat(sd)
        os.mount(fat,'/sd')
        sys.path.append('/sd')
        print("Mount SPI-SD successfully.")
    except:
        print("Fail on mount SPI-SD")

