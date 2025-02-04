#!/usr/bin/env python3
import serial
import time


ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
if __name__ == '__main__':
    
    ser.reset_input_buffer()

    while True:
        ser.write(b"Hello from Raspberry Pi!\n")
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        time.sleep(1)


def send_command(command:str)->str:
    ser.write(command)
    line = ser.readline().decode('utf-8').rstrip()
    return line