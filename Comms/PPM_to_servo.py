#!/usr/bin/env python

# PPM_to_servo.py
# 2019-10-09
# Public Domain

import time
import pigpio # http://abyz.me.uk/rpi/pigpio/python.html

IN_GPIO=4 # the PPM input GPIO
OUT_GPIO=[5, 6, 7, 8, 9, 10, 11, 12] # The servo output GPIO

start_of_frame = False
channel = 0
last_tick = None 

def cbf(gpio, level, tick):
   """
   Callback function that is triggered whenever the specified edge is detected

   Parameters
      gpio: GPIO pin number (0 to 31)
      level: type of edge (0 - FALLING_EDGE, 1 - RISING_EDGE, 2 - no change / timeout)
      tick: number of microseconds since boot
            WARNING: THIS WRAPS ROUGHLY EVERY 72 MINUTES
   """

   global start_of_frame, channel, last_tick
   if last_tick is not None:
      diff = pigpio.tickDiff(last_tick, tick)
      if diff > 3000: # start of frame
         start_of_frame = True
         channel = 0
      else:
         if start_of_frame:
            print("Channel #" + str(channel) + ": " + str(diff) + " microseconds")
            """
            if channel < len(OUT_GPIO):
               pi.set_servo_pulsewidth(OUT_GPIO[channel], diff)
               channel += 1
            """
   last_tick = tick
   
pi = pigpio.pi()
if not pi.connected:
   exit()

pi.set_mode(IN_GPIO, pigpio.INPUT)

cb = pi.callback(IN_GPIO, pigpio.RISING_EDGE, cbf)

time.sleep(60)

cb.cancel()

pi.stop()

