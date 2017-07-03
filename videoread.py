# Image Reader Example
#
# USE THIS EXAMPLE WITH A USD CARD!
#
# This example shows how to use the Image Reader object to replay snapshots of what your
# OpenMV Cam saw saved by the Image Writer object for testing machine vision algorithms.

import sensor, image, time, math

snapshot_source = False # Set to true once finished to pull data from sensor.

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()

img_reader = None if snapshot_source else image.ImageReader("/stream.bin")


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
    img = sensor.snapshot() if snapshot_source else img_reader.next_frame(copy_to_fb=True, loop=True)
    # Do machine vision algorithms on the image here.
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

    for tag in img.find_apriltags(families=tag_families): # defaults to TAG36H11 without "families".
        img.draw_rectangle(tag.rect(), color = (255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
        print_args = (family_name(tag), tag.id(), (180 * tag.rotation()) / math.pi)
        print("Tag Family %s, Tag ID %d, rotation %f (degrees)" % print_args)
    print(clock.fps())
print(clock.fps())
