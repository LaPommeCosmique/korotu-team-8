#!/usr/bin/env python

# PPM_to_servo.py
# 2019-10-09
# Public Domain

import time
import pigpio # http://abyz.me.uk/rpi/pigpio/python.html

IN_GPIO=4 # the PPM input GPIO
channel_values=[None] * 8

start_of_frame = False
channel = 0
last_tick = None

def cbf(gpio, level, tick):
   global start_of_frame, channel, last_tick
   if last_tick is not None:
      diff = pigpio.tickDiff(last_tick, tick)
      if diff > 3000: # start of frame
         start_of_frame = True
         channel = 0
      else:
         if start_of_frame:
            if channel < 8:
               channel_values[channel] = diff
            
            channel += 1
            if channel == 8:
               print(channel_values)
   last_tick = tick
   
pi = pigpio.pi()
if not pi.connected:
   exit()

pi.set_mode(IN_GPIO, pigpio.INPUT)

cb = pi.callback(IN_GPIO, pigpio.FALLING_EDGE, cbf)

time.sleep(60)

cb.cancel()

pi.stop()

