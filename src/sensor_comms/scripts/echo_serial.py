#!/usr/bin/env python3
import serial

device_name = '/dev/ttyACM0'

if __name__ == '__main__':
    ser = serial.Serial(device_name, 9600, timeout=1)
    ser.reset_input_buffer()
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print("Received, echoing: "+ line)
            ser.write(line.encode('utf-8'))

