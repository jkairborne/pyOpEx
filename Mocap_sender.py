import image, math, pyb, sensor, struct, time
import uart_pixracer


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
MAV_MOCAP_extra_crc = 109
MAV_MOCAP_message_id = 138
MAV_system_id = 1
MAV_component_id = 0x54

def send_att_pos_mocap_packet(uart,quat,x,y,z):
    global packet_sequence
    temp = struct.pack('<Q4ffff',
                       0,
                       quat[0],
                       quat[1],
                       quat[2],
                       quat[3],
                       x,
                       y,
                       z)
    temp = struct.pack("<bbbbb36s",
                       36,
                       0 & 0xFF,#packet_sequence & 0xFF,
                       MAV_system_id,
                       MAV_component_id,
                       MAV_MOCAP_message_id,
                       temp)
    temp = struct.pack("<b41sh",
                       0xFE,
                       temp,
                       checksum(temp, MAV_MOCAP_extra_crc))
    packet_sequence += 1
    print("\n\nPacket to be sent:")
    for i in range(len(temp)):
        print(temp[i])
    uart.write(temp)



MAV_SET_POSITION_TARGET_LOCAL_NED_extra_crc = 143
MAV_SET_POSITION_TARGET_LOCAL_NED_message_id = 84
POSITION_Type_mask = 0
def send_set_position_target_local_ned_packet(uart,x,y,z,yaw):
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

count = 0
clock = time.clock()

uart_baudrate = 921600
uart = pyb.UART(3, uart_baudrate, timeout_char = 1000)

while(True):
    clock.tick()
    time.sleep(100)
    send_att_pos_mocap_packet(uart,[1,0,0,0],11,22,33)
   # send_set_position_target_local_ned_packet(uart,-10,-20,-30,1.2)
   # print("in loop")
   # print("FPS %f" % clock.fps())
    count +=1
