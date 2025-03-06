 # camera.py
from CameraPI import read_camera_pi
from SerialBusRaspberry_send_and_receive import send_command, ser
from camera import cv2

officina = "Undefined"

START_COMMAND = b"START\n"
BLUE_CAR = "BLUECAR"
RED_CAR = "REDCAR"
ALLIGNE_COMMAND = b"ALLIGNE\n"
TRIAL_BEGIN = b"TRIALBEGIN\n"
STOP_COMMAND = b"STOP\n"
PICK_UP = b"PickUp\n"
DROP_BLUE = b"DropBlue\n"
ALLIGNE_2 = b"ALLIGNE2\n"
DROP_RED = b"DROPRED\n"
RESET_COMMAND = b"RESET\n"

line = send_command(START_COMMAND)

class Holder: 
    seats = []
    def add(self, thing:str)->int:
        self.seats.append(thing)
        return len(self.seats)
    
    def remove(self, position:int)->str:
        thing=self.seats[position]
        self.seats.pop(position)
        return thing
    
    def count_blue(self)->int:

        return len(list(filter(lambda t:t== BLUE_CAR , self.seats)))
    
    def count_red(self)->int:

        return len(list(filter(lambda t:t== RED_CAR , self.seats)))


thing_holder = Holder()

ser.write(START_COMMAND)
if line == "START END" :
    thing = read_camera_pi()
    if thing == "YELLOWCAR":
        officina == "YELLOW"
        cv2.destroyAllWindows() #modifica per rasberry 
        send_command(ALLIGNE_COMMAND)       
    else:
         if thing == "GREENCAR":
            officina == "GREEN"
            cv2.destroyAllWindows()                     
            send_command(ALLIGNE_COMMAND)


if line == "ALLIGNEND":
    thing = read_camera_pi()
    
    ser.write(TRIAL_BEGIN)
    if thing == "REDCAR" and thing_holder.count_red() < 2:    
        ser.write(STOP_COMMAND)
        ser.write (PICK_UP)
        thing_holder.add(RED_CAR)
        ser.write(ALLIGNE_COMMAND)

    else :

        if thing == "BLUECAR" and thing_holder.count_blue() < 4:
            ser.write(STOP_COMMAND)
            ser.write (PICK_UP)
            thing_holder.add(BLUE_CAR)

            ser.write(TRIAL_BEGIN)
cv2.destroyAllWindows()

if thing_holder.count_blue() == 4 and thing_holder.count_red() == 2:
    
    ser.write(DROP_BLUE)
if line == "DropBlueEnd" : 
    ser.write(ALLIGNE_2)
if line == "ALLIGNE2END":
    thing = read_camera_pi()
    ser.rite(TRIAL_BEGIN)
    if thing == "REDCAR" and thing_holder.count_red() < 5:
        ser.write(STOP_COMMAND)
        ser.write (PICK_UP)
        thing_holder.add(RED_CAR)
        ser.write(TRIAL_BEGIN)
cv2.destroyAllWindows()

if thing_holder.count_red() == 5:
    
    ser.write(DROP_RED)
if line == "DROPREDEND":
    
    ser.write(RESET_COMMAND)


#Capisci come funziona il sensore di colore del rasberry
#Fai il count per chiudere le finestra





    
    












    
        












   
    
    


