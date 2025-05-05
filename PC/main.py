 # camera.py


from SerialBusRaspberry_send_and_receive import send_command as serial_send
import time
import serial
#from test_serial import TestSerial
from commands import *
from camera import read_from_camera

steps = 0; range(1,18)

officina : str| None =  None

class CarHolder:  
    seats = []

    def add(self, car: str) -> int:
        self.seats.append(car)
        return len(self.seats)

    def remove(self, position: int) -> str:
        car = self.seats[position]
        self.seats.pop(position)
        return car

    def count_blue(self) -> int:
        return len(list(filter(lambda t: t == BLUE_CAR, self.seats)))

    def count_red(self) -> int:
        return len(list(filter(lambda t: t == RED_CAR, self.seats)))


car_holder = CarHolder()  

def read_color() -> str:
    color = read_from_camera()
    if not color:
        print("!NO COLOR!")
        return "NO_COLOR"
    return color
    

def send_command(command:str) -> str:
    return serial_send(command,ser)


def read_station_color() -> str:
    print("Read station color")
    line = send_command(START_COMMAND)
    officina = read_color()
    print("Station color", officina)
    return send_command(officina)     
    
    

def round_one(station_status:str):
    print("Round one", station_status)
    line = ""
    if station_status == START_END:
        line = send_command(ALLIGNE_COMMAND)
    while line != ALLIGNE_END:
        time.sleep(1)
    line = send_command(TRIAL_BEGIN)
    trial_steps = 1
    while car_holder.count_blue() < 5 and trial_steps <= 9:
        car_color = read_color()
        if car_color == BLUE_CAR:
            line = send_command(PICK_UP)
            car_holder.add(BLUE_CAR)
        while line !=  PICKED_UP and trial_steps <= 9:
           time.sleep(1)
        check_trial_steps(trial_steps)
        line = send_command(TRIAL_BEGIN)



def check_trial_steps(trial_steps):
    trial_steps = trial_steps + 1
    if trial_steps == 9:
        line = send_command(ROTATE)


def round_two(station_status:str):
    print("Round two", station_status)
    line = ""
    while line != ROTATE_END:
        time.sleep(1)
    line = send_command(TRIAL_BEGIN)
    trial_steps = 1
    while car_holder.count_blue() < 5 and 9 <= trial_steps <= 18:
        car_color = read_color()
    if car_color == BLUE_CAR:
        line = send_command(PICK_UP)
        car_holder.add(BLUE_CAR)
        while line != PICKED_UP and 9 <= trial_steps <= 18:
            time.sleep(1)
    check_trial_steps(trial_steps)
    line = send_command(TRIAL_BEGIN)

    

def deliver_blues():
    print("Deliver blu at", station_status)
    line = ""
    if car_holder.count_blue() == 4 : 
        line = send_command(STOP_COMMAND)
        line = send_command(DROP_BLUE)


def deliver_red():
    if car_holder.count_red() == 5 :
        send_command(STOP_COMMAND)
        line = send_command(DROP_RED)

def reset ():
     print ("Resetting")
     line = ""
     if line == DROP_END:
           line = send_command(RESET_COMMAND)


global ser

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    #ser = TestSerial()
    ser.reset_input_buffer()
    while True:
        
        line = ser.readline().decode('utf-8').rstrip()
        print("input", line, '\n')
        time.sleep(1)

        while True:
            station_status = read_station_color()
            time.sleep(2)
            round_one(station_status)
            time.sleep(2)
            round_two(station_status)
            time.sleep(2)
            deliver_blues()
            time.sleep(2)
            reset()
            time.sleep(2)
        
            
        
        
        
        
        
        
                 
        

         
        




#colori_bot a quanto pare funziona





    
    












    
        












   
    
    

