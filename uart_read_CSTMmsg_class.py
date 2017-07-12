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
        for i in range(self.numbytes,0,-1):
            print(self.buf[i]),

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
        res = checksum(self.buf,bkmk,10)
        print("value for all 10 is: %d" % res)

def verify_checksum(data,length):
    sum = 0
    for i in range(0,length):
        sum+=data[i]
    sum = sum & 0xFF
    if (sum == 0xFF):
        return True
    else:
        return False


if __name__=="__main__":
    print("at the start")
    rec = RPReceiver()
    arr = [65,66,67,68,69,70,71,72,73,80, 66]
    print("arr: ")
    for i in range(0,len(arr)):
        print(arr[i])
    res = verify_checksum(arr,11)
    print(res)
    arr = [65,66,67,68,69,70,71,35]
    print("arr2: ")
    res = verify_checksum(arr,8)
    print(res)

    arr = [65,66,67,68,69,176]
    print("arr3: ")
    res = verify_checksum(arr,6)
    print(res)


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

