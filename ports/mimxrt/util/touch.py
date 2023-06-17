
import os
import sys

def touch(name):
    Newfile = True
    try:
        info = os.stat(name)
        if info.st_size == 0:
            os.remove(name)
        else:
            Newfile = False
    except:
        pass

    fd = os.open(name, os.O_RDWR | os.O_CREAT)
    if not Newfile:
        os.lseek(fd, 0, os.SEEK_SET)
        chr = os.read(fd, 1)
        os.lseek(fd, 0, os.SEEK_SET)
        os.write(fd, chr)
        # print(name, "touched with", chr)
    os.close(fd)


if __name__ == "__main__":

    for arg in sys.argv[1:]:
        touch(arg)

