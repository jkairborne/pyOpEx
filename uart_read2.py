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
uart.init(57600,bits=8,parity=None,stop=1,timeout=1000,timeout_char=0,read_buf_len=512)

def available():
    return uart.any()

def read_byte():
    return uart.readchar()

# FSM Code

fsm_state = 0
last_byte = 0

FSM_STATE_NONE = 0
FSM_STATE_ZERO = 1
FSM_STATE_ACTION1 = 2
FSM_STATE_ACTION2 = 3

def read_message(msg)
    if(check_chksum()):
        unpack("ii",msg[0:7])
    else:


def parse_byte(byte):
    if (byte == 0xFE):
        print("byte 0xFE")

    elif (byte == 0xFF):
        print("byte 0xff")
    else:
        print("not one\n")


# Main Loop

clock = time.clock()
while(True):
    clock.tick()
    img = sensor.snapshot()
    # Parse Commands #

    for i in range(available()):
        parse_byte(read_byte())