import time
import ustruct
from pyb import UART
from ubinascii import hexlify

import uart_pixracer
import sensor, image, time, mjpeg, pyb, math

f_x = (2.8 / 3.984) * 160 # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120 # find_apriltags defaults to this if not set
c_x = 160 * 0.5 # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5 # find_apriltags defaults to this if not set (the image.h * 0.5)

tag_families = 0
tag_families |= image.TAG16H5 # comment out to disable this family
tag_families |= image.TAG25H7 # comment out to disable this family
tag_families |= image.TAG25H9 # comment out to disable this family
tag_families |= image.TAG36H10 # comment out to disable this family
tag_families |= image.TAG36H11 # comment out to disable this family (default family)
tag_families |= image.ARTOOLKIT # comment out to disable this family

# What's the difference between tag families? Well, for example, the TAG16H5 family is effectively
# a 4x4 square tag. So, this means it can be seen at a longer distance than a TAG36H11 tag which
# is a 6x6 square tag. However, the lower H value (H5 versus H11) means that the false positve
# rate for the 4x4 tag is much, much, much, higher than the 6x6 tag. So, unless you have a
# reason to use the other tags families just use TAG36H11 which is the default family.

def family_name(tag):
    if(tag.family() == image.TAG16H5):
        return "TAG16H5"
    if(tag.family() == image.TAG25H7):
        return "TAG25H7"
    if(tag.family() == image.TAG25H9):
        return "TAG25H9"
    if(tag.family() == image.TAG36H10):
        return "TAG36H10"
    if(tag.family() == image.TAG36H11):
        return "TAG36H11"
    if(tag.family() == image.ARTOOLKIT):
        return "ARTOOLKIT"

def to_img_frame(pts):
    for i in range(0,len(pts)):
        if(i%2==0):
            pts[i] = pts[i]-c_x
        else:
            pts[i] = pts[i]-c_y
    return pts

def to_cam_frame(pts):
    for i in range(0,len(pts)):
        if(i%2!=0):
            pts[i] = pts[i]+c_x
        else:
            pts[i] = pts[i]+c_y
    return pts

if __name__=="__main__":
    print(image.__name__)
    print("at the start")
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QQVGA) # we run out of memory if the resolution is much bigger...
    #sensor.skip_frames(time = 2000)
    sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
    sensor.set_auto_whitebal(False)
    clock = time.clock()
    rec = uart_pixracer.RPReceiver()
    while(True):
        clock.tick()
        img = sensor.snapshot()
        rec.sync()
        [roll,pitch,yaw] = rec.getrpy()

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

            dPts_ctr = [10,20,-10,20,10,-20,-10,-20]
            dPts = to_cam_frame(dPts_ctr)
            cPts = [pt1x,pt1y,pt2x,pt2y,pt3x,pt3y,pt4x,pt4y]

            print("roll and pitch: %f   %f " % (roll,pitch))
            [v_c,virtcam_ctr] = tag.ibvs_calc(roll,pitch,cPts,dPts)
            #img.draw_line([pt1x, pt1y, pt2x, pt2y],color = (0,0,255))
            #img.draw_line([pt2x, pt2y, pt3x, pt3y],color = (0,0,255))
            #img.draw_line([pt3x, pt3y, pt4x, pt4y],color = (0,0,255))
            #img.draw_line([pt4x, pt4y, pt1x, pt1y],color = (0,0,255))
            print_args1 = (pt1x,pt1y,pt2x,pt2y,pt3x,pt3y,pt4x,pt4y)
            print("corners_py: %d %d %d %d %d %d %d %d" % print_args1)
            virtcam = to_cam_frame(virtcam_ctr)
            print_args2 = (virtcam[0],virtcam[1],virtcam[2],virtcam[3],virtcam[4],virtcam[5],virtcam[6],virtcam[7])
            print("virtcam_py: %d %d %d %d %d %d %d %d" % print_args2)
            img.draw_circle(int(virtcam[0]),int(virtcam[1]),2,color=(255,50,50))
            img.draw_circle(int(virtcam[2]),int(virtcam[3]),2,color=(255,50,50))
            img.draw_circle(int(virtcam[4]),int(virtcam[5]),2,color=(255,50,50))
            img.draw_circle(int(virtcam[6]),int(virtcam[7]),2,color=(255,50,50))



