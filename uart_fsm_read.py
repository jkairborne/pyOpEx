#This code is taken from example 17-Pixy emulation, the uart section
#Just shortened it down a little bit.

uart_baudrate = 19200

import image, math, pyb, sensor, struct, time

# Camera Setup

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = pyb.UART(3, uart_baudrate, timeout_char = 1000)

def write(data):
    uart.write(data)

def available():
    return uart.any()

def read_byte():
    return uart.readchar()

# Helper Stuff

def checksum(data):
    checksum = 0
    for i in range(0, len(data), 2):
        checksum += ((data[i+1] & 0xFF) << 8) | ((data[i+0] & 0xFF) << 0)
    return checksum & 0xFFFF

def get_normal_signature(code):
    for i in range(len(e_lab_color_signatures)):
        if code & (1 << i):
            return e_lab_color_signatures[i]
    return 0

# FSM Code

fsm_state = 0
last_byte = 0

FSM_STATE_NONE = 0
FSM_STATE_ZERO = 1
FSM_STATE_ACTION1 = 2
FSM_STATE_ACTION2 = 3

def parse_byte(byte):
    global fsm_state
    global last_byte

    if fsm_state == FSM_STATE_NONE:
        if byte == 0x00:
            fsm_state = FSM_STATE_ZERO
            print("byte was 0x00")
        else:
            fsm_state = FSM_STATE_NONE
            print("byte was not %d" % byte)

    elif fsm_state == FSM_STATE_ZERO:
        if byte == 0xFF:
            fsm_state = FSM_STATE_ACTION1
            print("byte was 0xFF")
            fsm_state = FSM_STATE_NONE
        elif byte == 0xFE:
            fsm_state = FSM_STATE_ACTION2
            print("byte was 0xFE")
            fsm_state = FSM_STATE_NONE
        else:
            fsm_state = FSM_STATE_NONE

    last_byte = byte

# Main Loop

clock = time.clock()
while(True):
    clock.tick()
    img = sensor.snapshot()
    # Parse Commands #

    for i in range(available()):
        parse_byte(read_byte())
