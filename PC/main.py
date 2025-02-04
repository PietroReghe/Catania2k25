 # camera.py
from camera import read_from_camera
from SerialBusRaspberry_send_and_receive import send_command, ser
from camera import cv2

tonoli = "drera"

officina = "Undefined"

START_COMMAND = b"START\n"
BLUE_CAR = "BLUECAR"
RED_CAR = "REDCAR"

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

ser.write(b"START\n")
if line == "START END" :
    thing = read_from_camera()
    if thing == "YELLOWCAR":
        officina == "YELLOW"
        cv2.destroyAllWindows() #modifica per rasberry       
    else:
         if thing == "GREENCAR":
            officina == "GREEN"
            cv2.destroyAllWindows()                     
ser.write(b"ALLIGNE\n")

if line == "ALLIGNEND":
    thing = read_from_camera()
    ser.rite(b"TRIALBEGIN\n")
    if thing == "REDCAR" and thing_holder.count_red() < 2:
        ser.write(b"STOP\n")
        ser.write (b"PickUp\n")
        thing_holder.add(RED_CAR)
        ser.write(b"TRIALBEGIN\n")

    else :

        if thing == "BLUECAR" and thing_holder.count_blue() < 4:
            ser.write(b"STOP\n")
            ser.write (b"PickUp\n")
            thing_holder.add(BLUE_CAR)
            
            ser.write(b"TRIALBEGIN\n")
cv2.destroyAllWindows()

if thing_holder.count_blue() == 4 and thing_holder.count_red() == 2:
    ser.write(b"DropBlue\n")
if line == "DropBlueEnd" :
    ser.write(b"ALLIGNE2\n")
if line == "ALLIGNE2END":
    thing = read_from_camera()
    ser.rite(b"TRIALBEGIN\n")
    if thing == "REDCAR" and thing_holder.count_red() < 5:
        ser.write(b"STOP\n")
        ser.write (b"PickUp\n")
        thing_holder.add(RED_CAR)
        ser.write(b"TRIALBEGIN\n")
cv2.destroyAllWindows()

if thing_holder.count_red() == 5:
    ser.write(b"DROPRED\n")
if line == "DROPREDEND":
    ser.write(b"RESET\n")




    
    












    
        












   
    
    


