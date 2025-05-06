#!/usr/bin/env python3
import serial
import time


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
    ser.reset_input_buffer()
    ser.write(b"HELLO ARDUINO!\n")
    line = ser.readline().decode('utf-8').rstrip()
    print("Input", line, '\n')
    time.sleep(1)


def send_command(command:str, ser)->str:
    ser.reset_input_buffer()
    ser.write(command)
    line = ser.readline().decode('utf-8').rstrip()
    while line == "":
        print("Waiting for response...")
        time.sleep(0.1)
        line = ser.readline().decode('utf-8').rstrip()

    print("Output for command", command, "->" ,line)
    return line