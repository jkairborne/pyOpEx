import time
from pyb import UART
from ubinascii import hexlify

class RPReceiver:
    def __init__(self,expectedlength=10, stchar = 0xFF, enchar = 0x49):
        self.uart = UART(3, 57600)
        self.uart.init(57600, bits=8, parity=None, stop=1,timeout=1000, flow=0, timeout_char=0, read_buf_len=256)
        self.startbyte = stchar
        self.endbyte = enchar
        self.xlength = expectedlength
        self.buf = bytearray(256)
        self.numbytes =0

    def deinit(self):
        self.uart.deinit()
        self.uart=None

    def readbuf(self):
        self.numbytes = self.uart.readinto(self.buf)
  #      for i in range(self.numbytes,0,-1):
  #          print(self.buf[i]),

    def sync(self):
        self.readbuf()
        print("sync called, # of bytes is %d" % self.numbytes)
        for i in range((self.numbytes-self.xlength),0,-1): # This maybe needs to go to -1?

            if ((self.buf[i] == self.startbyte) and (self.buf[i+self.xlength] == self.endbyte)):
                print("got a msg")
                self.decode(i)
                return (1)
        print("didn't find the message")
        print("start and end characters")
        print(self.startbyte)
        print(self.endbyte)

    def decode(self,bkmk):
        for i in range(bkmk,(bkmk+self.xlength)):
            print("byte %d: " % bkmk)


if __name__=="__main__":
    print("at the start")
    rec = RPReceiver()
    while(True):
        rec.sync()
        time.sleep(1000)
        rec.sync()
        time.sleep(1000)
        rec.sync()
        time.sleep(1000)
        rec.sync()
        time.sleep(1000)
        rec.sync()
        time.sleep(1000)
        rec.sync()
        time.sleep(1000)
        rec.sync()
        time.sleep(1000)


