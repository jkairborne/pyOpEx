from time import sleep
from pyb import UART
from ubinascii import hexlify

class MavReceiver:
    def __init__(self,expectedlength=15, stchar = b'10', enchar = b'102'):
        self.uart = UART(3, 19200)
        self.startbyte = stchar
        self.endbyte = enchar
        self.xlength = expectedlength
        self.buf = bytearray(2*self.xlength)

    def deinit(self):
        self.uart.deinit()
        self.uart=None


    def sync(self):
        #Need to make sure none of the bits in the array are the stop bit...
        self.buf[self.xlength] = 0
        while(True):
#            char = self.uart.read(1)
            if self.uart.any():
                print("in synced up")
                self.attemptread()

                return (1)

    def attemptread(self):
        while(True):
            idx=0
            while idx < self.xlength:
                if self.uart.any():
                    bytes_read = self.uart.readinto(self.buf[idx:])
                    print('Got {} bytes of data'.format(bytes_read), hexlify(self.buf[idx:idx+bytes_read], b':'))
                    idx += bytes_read

            if (self.buf[self.xlength] == self.endbyte):
                [self.roll, self.pitch] = self.decode(self.buf[0:self.xlength])
                self.attemptread()
            else:
                sync()

    def decode(self):
        for i in range(len(self.buf)):
            print(self.buf[i])
        # MEAT Ox   F THE FCT SHOULD GO HERE

    def publish(self):
        self.uart.write("HloWrld!")


if __name__=="__main__":
    print("at the start")
    rec = MavReceiver()
    rec.publish()
    rec.sync()
    rec.publish()
    print("published first time")
    time.sleep(1000)
    rec.publish()
    print("published second time")
    time.sleep(1000)
    rec.publish()
    print("published third time")
    time.sleep(1000)
    rec.publish()
    print("published fourth time")
    time.sleep(1000)
    rec.publish()
    print("published fifth time")
    time.sleep(1000)
    rec.publish()
    print("published final time")

