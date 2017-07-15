# AprilTags Example
#
# This example shows the power of the OpenMV Cam to detect April Tags
# on the OpenMV Cam M7. The M4 versions cannot detect April Tags.

import sensor, image, time, math

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # we run out of memory if the resolution is much bigger...
#sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()

# Note! Unlike find_qrcodes the find_apriltags method does not need lens correction on the image to work.

# The apriltag code supports up to 6 tag families which can be processed at the same time.
# Returned tag objects will have their tag family and id within the tag family.

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

while(True):
    clock.tick()

    img = sensor.snapshot()
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

        desPt1x = tag.desired_pts()[0]# x top left
        desPt1y = tag.desired_pts()[1]
        desPt2x = tag.desired_pts()[2]# x top left
        desPt2y = tag.desired_pts()[3]
        desPt3x = tag.desired_pts()[4]# x top left
        desPt3y = tag.desired_pts()[5]
        desPt4x = tag.desired_pts()[6]# x top left
        desPt4y = tag.desired_pts()[7]

        img.draw_circle(int(desPt1x),int(desPt1y),2,color=(255,50,50))
        img.draw_circle(int(desPt2x),int(desPt2y),2,color=(255,50,50))
        img.draw_circle(int(desPt3x),int(desPt3y),2,color=(255,50,50))
        img.draw_circle(int(desPt4x),int(desPt4y),2,color=(255,50,50))
       # tag.desired_pts[0] = 1000

        roll = 21
        pitch = -22
#        cPts = [10,20,-10,20,10,-20,-10,-20]

        cPts = [15,25,-5,25,15,-15,-5,-15]
        dPts = [10,20,-10,20,10,-20,-10,-20]

        tag.ibvs_calc(roll,pitch,cPts,dPts)



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
