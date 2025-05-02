
#! /home/pietro/myenv/bin/python


from os import link
import serial
import time





    

def TRY_SERIAL(ser, line):
    if line == "Hello from Arduino!" :
        ser.write (b"START\n")
    else :
        if line == "Ready to work?":
            ser.write(b"YESS\n")

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    while True:
        
        line = ser.readline().decode('utf-8').rstrip()
        print("input", line, '\n')
        time.sleep(1)

        TRY_SERIAL(ser, line)
    
    
        
        



