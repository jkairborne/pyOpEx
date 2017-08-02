#MsgSender.py

import image, math, pyb, sensor, struct, time
import uart_pixracer


# Parameters #################################################################
M_PI = 3.14159265358979323846
uart_baudrate = 921600

MAV_system_id = 1
MAV_component_id = 0x54
MAX_DISTANCE_SENSOR_enable = True

lens_mm = 2.8 # Standard Lens.
lens_to_camera_mm = 22 # Standard Lens.
sensor_w_mm = 3.984 # For OV7725 sensor - see datasheet.
sensor_h_mm = 2.952 # For OV7725 sensor - see datasheet.

# Only tags with a tag ID in the dictionary below will be accepted by this
# code. You may add as many tag IDs to the below dictionary as you want...

# For each tag ID you need to provide then length of the black tag border
# in mm. Any side of the tag black border square will work.

valid_tag_ids = {
                  0 : 165, # 8.5" x 11" tag black border size in mm
                  1 : 165, # 8.5" x 11" tag black border size in mm
                  2 : 165, # 8.5" x 11" tag black border size in mm
                }

##############################################################################

# Camera Setup

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)

x_res = 160 # QQVGA
y_res = 120 # QQVGA
f_x = (lens_mm / sensor_w_mm) * x_res
f_y = (lens_mm / sensor_h_mm) * y_res
c_x = x_res / 2
c_y = y_res / 2
h_fov = 2 * math.atan((sensor_w_mm / 2) / lens_mm)
v_fov = 2 * math.atan((sensor_h_mm / 2) / lens_mm)

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

# Link Setup

uart = pyb.UART(3, uart_baudrate, timeout_char = 1000)

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
# http://mavlink.org/messages/common#DISTANCE_SENSOR
# https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_distance_sensor.h
def send_set_attitude_target_packet(thrust,quat):
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


MAV_SET_POSITION_TARGET_LOCAL_NED_extra_crc = 143
MAV_SET_POSITION_TARGET_LOCAL_NED_message_id = 84
POSITION_Type_mask = 0
def send_set_position_target_local_ned_packet(x,y,z,yaw):
    global packet_sequence
    temp = struct.pack('<IfffffffffffHBBB',
                       0,
                       x,
                       y,
                       z,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       yaw,
                       0,
                       POSITION_Type_mask,
                       MAV_system_id,
                       MAV_target_component_id,
                       8)
    temp = struct.pack("<bbbbb53s",
                       53,
                       packet_sequence & 0xFF,
                       MAV_system_id,
                       MAV_component_id,
                       MAV_SET_POSITION_TARGET_LOCAL_NED_message_id,
                       temp)
    temp = struct.pack("<b58sh",
                       0xFE,
                       temp,
                       checksum(temp, MAV_SET_POSITION_TARGET_LOCAL_NED_extra_crc))
    packet_sequence += 1
    uart.write(temp)

# Main Loop
count = 0
clock = time.clock()
rec = uart_pixracer.RPReceiver()
while(True):
    clock.tick()
    img = sensor.snapshot()
    tags = sorted(img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y), key = lambda x: x.w() * x.h(), reverse = True)
    time.sleep(100)
    rec.sync()
    [roll,pitch,desyaw] = rec.getrpy()
    thrust = 0.382
    desroll=0
    despitch=0
    quatern = rpy_to_quat(to_rad(desroll),to_rad(despitch),desyaw)
    send_set_attitude_target_packet(thrust,quatern)
    print("des roll, pitch, yaw: %.2f %.2f %.2f" % (desroll,despitch,desyaw))
    #print(quatern)
   # print("FPS %f" % clock.fps())
    count +=1

