import time
import ustruct
from pyb import UART
from ubinascii import hexlify


import sensor, image, time, mjpeg, pyb

class RPReceiver:
    def __init__(self,expectedlength=6, stchar = 0x58): #0x58 corresponds to 'X'
        self.uart = UART(3, 57600)
        self.uart.init(57600, bits=8, parity=None, stop=1,timeout=10, flow=0, timeout_char=0, read_buf_len=300)
        self.startbyte = stchar
        self.xlength = expectedlength
        self.buf = bytearray(300)
        self.numbytes =0
        self.potmsg = bytearray(self.xlength)
        self.roll = 0
        self.pitch = 0

    def deinit(self):
        self.uart.deinit()
        self.uart=None

    def readbuf(self):
        self.numbytes = self.uart.readinto(self.buf)
        if(self.numbytes == None):
            print("ERROR: No data incoming on UART")
            self.numbytes =0
       # print("list of all the bytes received: ")
        for i in range(self.numbytes,0,-1):
            print(self.buf[i]),

    def getrp(self):
        return [self.roll,self.pitch]

    def checkmsg(self,i):
        if (self.buf[i] == self.startbyte):
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
          #  print("i = %d" % i)
            if self.checkmsg(i):
                self.decode(i)
                return 1
        print("ERROR: didn't find the message")
        return 0

    def decode(self,bkmk):
        #print("in decode")
       # for i in range(0,4):
       #     print(self.potmsg[i])
        arr = ustruct.unpack_from('hhc',self.potmsg)
        self.roll = arr[0]/10000
        self.pitch = arr[1]/10000
  #      print("roll and pitch: %f   %f " % ((roll/10000),(pitch/10000)))
#        print(roll/300)
 #       print(pitch/300)


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
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QQVGA) # we run out of memory if the resolution is much bigger...
    #sensor.skip_frames(time = 2000)
    sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
    sensor.set_auto_whitebal(False)
    clock = time.clock()
    rec = RPReceiver()
    while(True):
        clock.tick()
        img = sensor.snapshot()
        rec.sync()
        [tst,tst1] = rec.getrp()
        print("roll and pitch: %f   %f " % (tst,tst1))
        for tag in img.find_apriltags(): # defaults to TAG36H11 without "families".
            img.draw_rectangle(tag.rect(), color = (255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
            pt1x = tag.corners()[0][0]# x top left
            pt1y = tag.corners()[0][1]
            pt2x = tag.corners()[1][0]# x top left
            pt2y = tag.corners()[1][1]
            pt3x = tag.corners()[2][0]# x top left
            pt3y = tag.corners()[2][1]
            pt4x = tag.corners()[3][0]# x top left
            pt4y = tag.corners()[3][1]


            roll = 0 #tst
            pitch = 0 #tst1
    #        cPts = [10,20,-10,20,10,-20,-10,-20]
    #        cPts = [15,25,-5,25,15,-15,-5,-15]
            dPts = [120,20,-10,20,10,-20,-10,-20]
            #cPts = [pt1x,pt1y,pt2x,pt2y,pt3x,pt3y,pt4x,pt4y]
            cPts = [90,60,90,60,90,60,90,60]

            img.draw_circle(dPts[0],dPts[1],2,color=(255,50,50))
           # img.draw_circle(dPts[2],dPts[3],2,color=(255,50,50))
           # img.draw_circle(dPts[4],dPts[5],2,color=(255,50,50))
          #  img.draw_circle(dPts[6],dPts[7],2,color=(255,50,50))

            [v_c,virtcam] = tag.ibvs_calc(roll,pitch,cPts,dPts)
            print("virtcam pts: %.2f   %.2f" % (virtcam[0],virtcam[1]))
            img.draw_line([pt1x, pt1y, pt2x, pt2y],color = (0,0,255))
            img.draw_line([pt2x, pt2y, pt3x, pt3y],color = (0,0,255))
            img.draw_line([pt3x, pt3y, pt4x, pt4y],color = (0,0,255))
            img.draw_line([pt4x, pt4y, pt1x, pt1y],color = (0,0,255))
            print_args1 = (pt1x,pt1y,pt2x,pt2y,pt3x,pt3y,pt4x,pt4y)
        #    print("corners_py: %d %d %d %d %d %d %d %d" % print_args1)



     #       img.draw_rectangle(rectangle(pt1x,pt1y,pt2x,pt2y,pt3x,pt3y,pt4x,pt4y),color = (0,0,255))
            print_args = (family_name(tag), tag.id(), (180 * tag.rotation()) / math.pi)
         #   print("Tag Family %s, Tag ID %d, rotation %f (degrees)" % print_args)
            print("framerate: %f" % clock.fps())



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
