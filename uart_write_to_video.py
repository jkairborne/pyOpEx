import time
import ustruct
from pyb import UART
from ubinascii import hexlify


import sensor, image, time, mjpeg, pyb

INCLUDE RPRECEIVER CLASS


RED_LED_PIN = 1
BLUE_LED_PIN = 3

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.QVGA) # or sensor.QQVGA (or others)
clock = time.clock() # Tracks FPS.



if __name__=="__main__":
    print("at the start")
    rec = RPReceiver()
    pyb.LED(RED_LED_PIN).on()
    sensor.skip_frames(time = 2000) # Give the user time to get ready.

    pyb.LED(RED_LED_PIN).off()
    pyb.LED(BLUE_LED_PIN).on()
    m = mjpeg.Mjpeg("example9.mjpeg")
    for i in range(150):
        clock.tick()
        image = sensor.snapshot()
       # if(i>10):
        rec.sync()
        [rll,ptc] = rec.getrp()
        image.draw_string(0, 0, "data from UART: %.2f %.2f"%(rll,ptc), color = (0xFF, 0x00, 0x00))
        print("roll, pitch: %.2f %.2f" % (rll,ptc))
        m.add_frame(image)
        print("frame #%d" % i)
        print(clock.fps())


    m.close(clock.fps())
    pyb.LED(BLUE_LED_PIN).off()
    print("Done! Reset the camera to see the saved recording.")
