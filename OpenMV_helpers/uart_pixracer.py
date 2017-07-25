import time
import ustruct
from pyb import UART
from ubinascii import hexlify

import sensor, image, time, mjpeg, pyb

baudrate = 921600
bufSize = 1000

class RPReceiver:
    def __init__(self,expectedlength=8, stchar = 0x58): #0x58 corresponds to 'X'
        self.uart = UART(3, baudrate)
        self.uart.init(baudrate, bits=8, parity=None, stop=1,timeout=10, flow=0, timeout_char=0, read_buf_len=bufSize)
        self.startbyte = stchar
        self.xlength = expectedlength
        self.buf = bytearray(bufSize)
        self.numbytes =0
        self.potmsg = bytearray(self.xlength)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def deinit(self):
        self.uart.deinit()
        self.uart=None

    def readbuf(self):
        self.numbytes = self.uart.readinto(self.buf)
        if(self.numbytes == None):
            print("ERROR: No data incoming on UART")
            self.numbytes =0

    def getrpy(self):
        return [self.roll,self.pitch,self.yaw]

    def checkmsg(self,i):
        if (self.buf[i] == self.startbyte):
    # Potential message should not contain the header (hence the i+1). It should include checksum.
            self.potmsg = self.buf[i+1:i+self.xlength]
            if (verify_checksum(self.potmsg)):
                return True
            else:
                return False

    def sync(self):
        self.readbuf()
        for i in range((self.numbytes-self.xlength),0,-1): # This maybe needs to go to -1?
            if self.checkmsg(i):
                self.decode()
                return 1
        print("ERROR: didn't find the message, numbytes = %d" % self.numbytes)
        for i in range(0,len(self.buf)):
        	print(self.buf[i])
        return 0

    def decode(self):
        arr = ustruct.unpack_from('hhhc',self.potmsg)
        self.roll = arr[0]/10000
        self.pitch = arr[1]/10000
        self.yaw = arr[2]/10000

def verify_checksum(data):
    sum = 0
    for i in range(0,len(data)):
        sum+=data[i]
    sum = sum & 0xFF
    if (sum == 0xFF):
        return True
    else:
        print("checksum failed!")
        return False
'''
if __name__=="__main__":
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time = 2000)
    clock = time.clock()
    rec = RPReceiver()
    while(True):
        clock.tick()
        img = sensor.snapshot()
        rec.sync()
        [tst,tst1,tst2] = rec.getrpy()
        time.sleep(100)
        print("roll and pitch and yaw:  %f   %f  %f " % (tst,tst1,tst2))
'''
