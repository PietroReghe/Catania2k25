
#! /home/pietro/myenv/bin/python


import serial
import time


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
    ser.reset_input_buffer()
    ser.write(b"HELLO ARDUINO!\n")
    line = ser.readline().decode('utf-8').rstrip()
    print("Input", line, '\n')
    time.sleep(1)
    
        
        



