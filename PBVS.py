import ustruct
import sensor, image, time, mjpeg, math, pyb
import struct

import uart_pixracer
import PID

M_PI = 3.14159265358979323846

def rpy_to_quat(roll, pitch,yaw):
    q=[0,0,0,0]
    q[0] = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2)+math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    q[1] = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2)-math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    q[2] = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)+math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
    q[3] = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)-math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
    return q

def to_rad(in_deg):
    return (in_deg*M_PI/180)

def to_deg(in_rad):
    return (in_rad*180/M_PI)

def z_to_mm(z_translation, tag_size): # z_translation is in decimeters...
    return (((z_translation * 100) * tag_size) / 165) - lens_to_camera_mm

# Helper Stuff

packet_sequence = 0

def checksum(data, extra): # https://github.com/mavlink/c_library_v1/blob/master/checksum.h
    output = 0xFFFF
    for i in range(len(data)):
        tmp = data[i] ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (output & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return output

MAV_target_component_id = 1
MAV_SET_ATTITUDE_TARGET_extra_crc = 49
MAV_SET_ATTITUDE_TARGET_message_id = 82
MAV_SET_ATTITUDE_TARGET_mask =0x00
MAV_system_id = 1
MAV_component_id = 0x54

def send_set_attitude_target_packet(uart,thrust,quat):
    global packet_sequence
    temp = struct.pack('<I4fffffBBB',
                       1,
                       quat[0],
                       quat[1],
                       quat[2],
                       quat[3],
                       0,
                       0,
                       0,
                       thrust,
                       MAV_system_id,
                       MAV_target_component_id,
                       MAV_SET_ATTITUDE_TARGET_mask)
    temp = struct.pack("<bbbbb39s",
                       39,
                       packet_sequence & 0xFF,
                       MAV_system_id,
                       MAV_component_id,
                       MAV_SET_ATTITUDE_TARGET_message_id,
                       temp)
    temp = struct.pack("<b44sh",
                       0xFE,
                       temp,
                       checksum(temp, MAV_SET_ATTITUDE_TARGET_extra_crc))
    packet_sequence += 1
    uart.write(temp)



x_res = 160 # QQVGA
y_res = 120 # QQVGA
f_x = (2.8 / 3.984) * x_res
f_y = (2.8 / 2.952) * y_res
c_x = x_res / 2
c_y = y_res / 2
h_fov = 2 * math.atan((3.984 / 2) / 2.8)
v_fov = 2 * math.atan((2.952 / 2) / 2.8)

dx = 0
dy = 0
dz = 0

tag_families = 0
#tag_families |= image.TAG16H5 # comment out to disable this family
#tag_families |= image.TAG25H7 # comment out to disable this family
#tag_families |= image.TAG25H9 # comment out to disable this family
#tag_families |= image.TAG36H10 # comment out to disable this family
tag_families |= image.TAG36H11 # comment out to disable this family (default family)
#tag_families |= image.ARTOOLKIT # comment out to disable this family

# What's the difference between tag families? Well, for example, the TAG16H5 family is effectively
# a 4x4 square tag. So, this means it can be seen at a longer distance than a TAG36H11 tag which
# is a 6x6 square tag. However, the lower H value (H5 versus H11) means that the false positve
# rate for the 4x4 tag is much, much, much, higher than the 6x6 tag. So, unless you have a
# reason to use the other tags families just use TAG36H11 which is the default family.

def sat_fct(inp,sat):
    if (inp > sat):
        return sat
    elif (inp < -sat):
        return -sat
    else:
        return inp


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

#This simply multiplies a 4x4 transform matrix by the 4x1 vector [x,y,z,1]^T
def trsfm_mat(r,p,x,y,z,psi):
    res = [0,0,0,1]
    sr = math.sin(r)
    cr = math.cos(r)
    sp = math.sin(p)
    cp = math.cos(p)
    res[0] = cr*x + sr*z + dx
    res[1] = sr*sp*x + cp*y - sp*cr*z + dy
    res[2] = -cp*sr*x+sp*y + cr*cp*z + dz
    return res

if __name__=="__main__":
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QQVGA) # we run out of memory if the resolution is much bigger...
    sensor.skip_frames(time = 2000)
    sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
    sensor.set_auto_whitebal(False)
    clock = time.clock()
    hoverthrust = 0.385

    rec = uart_pixracer.RPReceiver()
    pidx = PID.PID()
    pidy = PID.PID()
    pidz = PID.PID()
    pidpsi = PID.PID()

#    latkP = 0.034
#    latkP = 0.017
    latkP = 0.010
    pidx.setKp(latkP)
    pidy.setKp(latkP)

#    latkD = 0.017
    latkD = 0.005
    pidx.setKd(latkD)
    pidy.setKd(latkD)

    pidz.setKp(0.02)
    pidz.setKd(0.01)
    desZ = -10 #1m in real world.
    pidz.setPoint(desZ)


    #imagewriter stuff
    img_writer = image.ImageWriter("/stream.bin")
    #m = mjpeg.Mjpeg("PBVS2.mjpeg")
    # Red LED on means we are capturing frames.
    red_led = pyb.LED(1)
    blue_led = pyb.LED(3)
    red_led.on()
    start = pyb.millis()
    framesToCapture = 300


    while(framesToCapture>0):
        img = sensor.snapshot()
        rec.sync()
        [roll,pitch,yaw] = rec.getrpy()

        tags = sorted(img.find_apriltags(), key = lambda x: x.w() * x.h(), reverse = True)
        #lambda is just an undeclared function. In this case it means the key is x.w()*x.h(), and we reverse the sort
        #From python help: key specifies a function of one argument that is used to extract a comparison key from each list element
        #Thus the above returns the largest of the detected apriltags (Or the one furthest to the right?).

        for tag in tags: # defaults to TAG36H11 without "families".
            img.draw_rectangle(tag.rect(), color = (255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
        if tags:
            blue_led.on()
            delx = tags[0].x_translation()
            dely = tags[0].y_translation()
            delz = tags[0].z_translation()
            delpsi = tags[0].z_rotation()

            fm_uav = trsfm_mat(roll,pitch,delx,dely,delz,delpsi)
            # The delx in pixracer frame is dely in camera frame.
            # The dely in pixracer frame is delx in camera frame. See July 23rd/24th notes
            desroll = sat_fct(-pidx.update(fm_uav[0]),0.17)
            despitch = sat_fct(pidy.update(fm_uav[1]),0.17)
            dthrust = sat_fct(-pidz.update(fm_uav[2]),0.20)

            desquat = rpy_to_quat(desroll,despitch,yaw)
            desthrst = hoverthrust+dthrust
            #TODO pidpsi
            send_set_attitude_target_packet(rec.uart,desthrst,desquat)

          #  print("roll, pitch: %.2f  %.2f" % (roll, pitch))
          #  print("delx,y,z,psi: %.2f  %.2f  %.2f" % (delx,dely,delz))
          #  print("corrected: %.2f  %.2f  %.2f" % (fm_uav[0],fm_uav[1],fm_uav[2]))
          #  print("desroll, pitch, thrust: %.2f  %.2f  %.2f" % (to_deg(desroll),to_deg(despitch),desthrst))


            if(framesToCapture>0):
                clock.tick()
                img.draw_string(0,00, "d: %.2f %.2f %.2f" % (delx, dely, delz), color = (0xFF, 0x00, 0x00))
                img.draw_string(0,20, "r, p: %.2f %.2f" % (roll,pitch), color = (0xFF, 0x00, 0x00))
                img.draw_string(0,40, "%.2f %.2f %.2f"%(fm_uav[0],fm_uav[1],fm_uav[2]), color = (0xFF, 0x00, 0x00))
                img.draw_string(0,60, "%.2f %.2f %.2f" % (desroll,despitch,dthrust), color = (0xFF, 0x00, 0x00))
                #img.draw_string(0,80, "thr: %.3f %.3f" %(dthrust,desthrst), color = (0xFF, 0x00, 0x00))

                x = tags[0].x_translation()
                y = tags[0].y_translation()
                z = tags[0].z_translation()
                xrot = tags[0].x_rotation()
                yrot = tags[0].y_rotation()
                zrot = tags[0].z_rotation()

             #   img.draw_string(0,00, "x,    y,     z", color = (0xFF, 0x00, 0x00))
             #   img.draw_string(0,20, "%.2f %.2f %.2f" % (x,y,z), color = (0xFF, 0x00, 0x00))
             #   img.draw_string(0,40, "xrot, yrot,  zrot", color = (0xFF, 0x00, 0x00))
             #   img.draw_string(0,60, "%.2f %.2f %.2f" % (xrot,yrot,zrot), color = (0xFF, 0x00, 0x00))
                img_writer.add_frame(img)
#                m.add_frame(img)
                framesToCapture -=1
        else:
            blue_led.off()
            #m.add_frame(sensor.snapshot())
            img_writer.add_frame(img)
            framesToCapture -=1
            pidx.setDerivator(0)
            pidy.setDerivator(0)
            pidz.setDerivator(0)

            pidx.setIntegrator(0)
            pidy.setIntegrator(0)
            pidz.setIntegrator(0)

            q = [1,0,0,0]
          #  print("In the else -213")
            send_set_attitude_target_packet(rec.uart,hoverthrust,q)
    img_writer.close()
#    m.close(7)
    print("done recording\n\n\n")
    red_led.off()
    blue_led.off()
