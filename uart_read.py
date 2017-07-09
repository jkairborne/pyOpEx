
# UART Control
#
# This example shows how to use the serial port on your OpenMV Cam. Attach pin
# P4 to the serial input of a serial LCD screen to see "Hello World!" printed
# on the serial LCD display.

import time, struct, ubinascii
from pyb import UART

# Always pass UART 3 for the UART number for your OpenMV Cam.
# The second argument is the UART baud rate. For a more advanced UART control
# example see the BLE-Shield driver.
uart = UART(3, 57600)
print("values received: \n")
while(True):
    buf = bytearray(10)
    if(uart.any()):
        uart.readinto(buf,10)


    print(buf)
#    if (buf[0] == 72):
#        print("We received : %d", buf)
#          print("in struct: %d%d%d%d%d%d%d%d",struct.unpack("8h",tmp2))
#    else:
#        tmp2 = struct.pack(binary)
#        print("in the else")
