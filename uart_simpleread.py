# UART Control
#
# This example shows how to use the serial port on your OpenMV Cam. Attach pin
# P4 to the serial input of a serial LCD screen to see "Hello World!" printed
# on the serial LCD display.

import time
import pyb

# Always pass UART 3 for the UART number for your OpenMV Cam.
# The second argument is the UART baud rate. For a more advanced UART control
# example see the BLE-Shield driver.
uart = pyb.UART(3, 115200, timeout_char = 2000)

while(True):
    print("in the True loop")
    if uart.any():
        print("data on the uart:")
        print(uart.read(1000))
    time.sleep(1000)
