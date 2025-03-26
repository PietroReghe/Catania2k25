 # camera.py
from CameraPI import read_camera_pi
from SerialBusRaspberry_send_and_receive import send_command, ser
from CameraPI import cv2
import time
import serial


steps = 0; range(1,18)


officina = "Undefined"

START_COMMAND = b"START\n"
BLUE_CAR = b"BLUECAR\n"
RED_CAR = b"REDCAR\n"
ALLIGNE_COMMAND = b"ALLIGNE\n"
TRIAL_BEGIN = b"TRIALBEGIN\n"
STOP_COMMAND = b"STOP\n"
PICK_UP = b"PickUp\n"
DROP_BLUE = b"DropBlue\n"
ALLIGNE_2 = b"ALLIGNE2\n"
DROP_RED = b"DROPRED\n"
RESET_COMMAND = b"RESET\n"
PICKED_UP = b"PickedUp\n"
ROTATE = b"ROTATE\n"
GREEN = b"GREEN\n"
YELLOW = b"YELLOW\n"



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

def read_color():
    pass

def read_station_color():
    line = send_command(START_COMMAND)
    #camera on
    station_color = read_color()
    if station_color == "YELLOWCAR":
                officina == "YELLOW" 
                send_command(YELLOW)     
    elif station_color == "GREENCAR" :
        officina == "GREEN"
        send_command(GREEN)
    



def round_one():
    if line == "START_END":
        line = send_command(ALLIGNE_COMMAND)
    if line == "ALLIGNE_END":
        line = send_command(TRIAL_BEGIN)
        # camera on
        trial_steps = 1
    while car_holder.count_red() < 2 and car_holder.count_blue() < 5 and trial_steps <= 18:
        car_color = read_color()
        if car_color == "BLUECAR":
            line = send_command(PICK_UP)
            car_holder.add(BLUE_CAR)
        elif car_color == "REDCAR" and car_holder.count_red() < 2:
            line = send_command(PICK_UP)
            car_holder.add(RED_CAR)
        elif line == "Picked_Up" and trial_steps <= 18:
            line = send_command(TRIAL_BEGIN)
        line = send_command(TRIAL_BEGIN)# per poter andare avndi deve aspettare che l'arduino dia la line Picked_Up
        check_trial_steps(trial_steps)# gli step me li controlla ogni volta e aggiunge +1 ogni volta che chiamo questa funzione?


def check_trial_steps(trial_steps):
    trial_steps = trial_steps + 1
    if trial_steps == 9:
        line = send_command(ROTATE)

        
        
        
    



def round_two():
    trial_steps = 1
    if line == "DropBlueEnd" : 
            line = send_command(ALLIGNE_2)
            line = send_command(TRIAL_BEGIN)
            #camera on
    while  car_holder.count_red() < 5 and trial_steps <= 18:
        car_color = read_color()
        if car_color == "REDCAR" and car_holder.count_red() < 5:
            line = send_command(PICK_UP)
            car_holder.add(RED_CAR)
        elif line == "Picked_Up" and trial_steps <= 18:
            line = send_command(TRIAL_BEGIN)
        line = send_command(TRIAL_BEGIN)
        check_trial_steps(trial_steps)


    
    

def deliver_blues():
    if car_holder.count_blue() == 4 and car_holder.count_red() == 2:
        send_command(STOP_COMMAND)
        line = send_command(DROP_BLUE)


def deliver_red():
    if car_holder.count_red() == 5 :
        send_command(STOP_COMMAND)
        line = send_command(DROP_RED)

def reset ():
     if line == "DROPRED_END":
           line = send_command(RESET_COMMAND)




if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
   
    while True:
        read_station_color()
        time.sleep(2)
        round_one()
        time.sleep(2)
        deliver_blues()
        time.sleep(2)
        round_two()
        time.sleep(2)
        deliver_red()
        time.sleep(2)
        reset()
        time.sleep(2)
       
        
        
        
        
        
        
        
                 
        

         
        


# Contraddizione tra fermarsi per prendere e nadare avanti
#Carcolor yello e green step+1
#colori_bot a quanto pare funziona





    
    












    
        












   
    
    

