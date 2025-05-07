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
    time.sleep(2)
    officina = read_color()
    print("Station color", officina)
    return send_command(officina)     
    
def demo(station_status:str) :
    print("demo", station_status)
    line = ""
    if  station_status:
        print("demo",TRIAL_BEGIN,  station_status)
        line = send_command(TRIAL_BEGIN)
        print("vado avanti...")
    while car_holder.count_blue() < 4:
        print("looking for blue car")
        car_color = read_color()
        print(car_color)
        if car_color == BLUE_CAR:
            print("Bluecar spotted")
            line = send_command(PICK_UP)
            car_holder.add(BLUE_CAR)
        else:
            print("no BLUECAR")
        line = send_command(TRIAL_BEGIN)
        print("looking for NEXT blue car")
    if car_holder.count_blue() == 4 : 
        line = send_command(STOP_COMMAND)


def round_one(station_status:str) :
    print("Round_one", station_status)
    line = ""
    if  station_status:
        trial_steps = 1
        print("Round_one",ALLIGNE_COMMAND,  station_status)
        line = send_command(ALLIGNE_COMMAND)
        print("Mi allineo")
    while trial_steps <= 9:
        print("looking for blue car", trial_steps)
        car_color = read_color()
        print(car_color)
        if car_color == BLUE_CAR:
            print("Bluecar spotted")
            trial_steps = check_trial_steps(trial_steps)
            line = send_command(PICK_UP)
            car_holder.add(BLUE_CAR)
        else:
            trial_steps = check_trial_steps(trial_steps)
            print("no BLUECAR")
        line = send_command(TRIAL_BEGIN)
        print("looking for NEXT blue car")
    line = send_command(ROTATE)
    time.sleep(0.2)
    check_blues_early()
    


def round_two(station_status:str):
    trial_steps = 10
    print("Round_two", station_status)
    line = ""
    if  station_status:
        print("Round_two",TRIAL_BEGIN,  station_status)
        line = send_command(TRIAL_BEGIN)
        print("Mi muovo")
    while car_holder.count_blue() < 4 and trial_steps <= 18:
        print("looking for blue car", trial_steps)
        car_color = read_color()
        print(car_color)
        if car_color == BLUE_CAR:
            print("Bluecar spotted")
            trial_steps = check_trial_steps(trial_steps)
            line = send_command(PICK_UP)
            car_holder.add(BLUE_CAR)
        else:
            trial_steps = check_trial_steps(trial_steps)
            print("no BLUECAR")
        line = send_command(TRIAL_BEGIN)
        print("looking for NEXT blue car")
    

 
def check_blues_early(station_status: str, trial_steps):
    
    print("4 blues", station_status)
    line = ""
    if car_holder.count_blue() == 4 : 
        while trial_steps <= 18:
            line = send_command(TRIAL_BEGIN)
            trial_steps = check_trial_steps(trial_steps)
        line = send_command(BLUES_EARLY)
        return True
    return False
       
def check_trial_steps(trial_steps):
    trial_steps = trial_steps + 1
    return trial_steps
    

def deliver_blues(station_status:str):
    print("Deliver blu at", station_status)
    line = "" 
    line = send_command(STOP_COMMAND)
    line = send_command(DROP_BLUE)


def deliver_red():
    if car_holder.count_red() == 5 :
        send_command(STOP_COMMAND)
        line = send_command(DROP_RED)

def reset():
     print ("Resetting")
     line = ""
     if line == DROP_END:
           line = send_command(RESET_COMMAND)

def resetting_early():
    print("Resetting Early")
    line = ""
    line = send_command(RESETTING_EARLY_SEND)


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
            time.sleep(1)

            round_one(station_status)
            time.sleep(1)

            handled = check_blues_early(station_status)

            if handled:
                resetting_early(station_status)  
            else:
                round_two(station_status)
                time.sleep(1)

                deliver_blues(station_status)
                time.sleep(1)

                reset(station_status)
                time.sleep(1)
        
            #demo(station_status)
            
        
            #se ho già raccolto 4 blu salto la lettura del colore e vado alla poszione di riferimento
        
        
        
        
        
        
                 
        

         
        




#colori_bot a quanto pare funziona





    
    












    
        












   
    
    

