import time
import ustruct
from pyb import UART
from ubinascii import hexlify

class RPReceiver:
    def __init__(self,expectedlength=6, stchar = 0x58): #0x58 corresponds to 'X'
        self.uart = UART(3, 57600)
        self.uart.init(57600, bits=8, parity=None, stop=1,timeout=1000, flow=0, timeout_char=0, read_buf_len=300)
        self.startbyte = stchar
        self.xlength = expectedlength
        self.buf = bytearray(300)
        self.numbytes =0
        self.potmsg = bytearray(self.xlength)

    def deinit(self):
        self.uart.deinit()
        self.uart=None

    def readbuf(self):
        self.numbytes = self.uart.readinto(self.buf)
        if(self.numbytes == None):
            print("ERROR: No data incoming on UART")
            self.numbytes =0
        print("list of all the bytes received: ")
        for i in range(self.numbytes,0,-1):
            print(self.buf[i]),

    def checkmsg(self,i):
        if (self.buf[i] == self.startbyte):
            #if (i<15 or i>25) == True: REENABLE TO CHECK "CORRUPTED" DATA OR "BAD" DATA
            #    self.buf[i+1] = 100
    # Potential message should not contain the header (hence the i+1). It should include checksum (hence the 5).
            self.potmsg = self.buf[i+1:i+self.xlength]
            if (verify_checksum(self.potmsg)):
                return True
            else:
                return False

    def sync(self):
        self.readbuf()
#        print("sync called, # of bytes is %d" % self.numbytes)
        for i in range((self.numbytes-self.xlength),0,-1): # This maybe needs to go to -1?
            print("i = %d" % i)
            if self.checkmsg(i):
                self.decode(i)
                return 1
        print("ERROR: didn't find the message")
        return 0

    def decode(self,bkmk):
        print("in decode")
        for i in range(0,4):
            print(self.potmsg[i])
        arr = ustruct.unpack_from('hhc',self.potmsg)
        roll = arr[0]
        pitch = arr[1]
        print("roll and pitch data: ")
        print(roll)
        print(pitch)

    def corrupt(self):
        self.potmsg[1] = 100
        print("corrupting data, %c" % self.potmsg[1])

def verify_checksum(data):
    sum = 0
    print("in verify checksum")
    for i in range(0,len(data)):
        sum+=data[i]
    sum = sum & 0xFF
    if (sum == 0xFF):
        print("checksum passed!")
        return True
    else:
        print("checksum failed!")
        return False

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

'''

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
'''
