"""
Autonomous car management program for Catania2k25 project
Controls car detection, pickup, and delivery based on color
"""

from SerialBusRaspberry_send_and_receive import send_command as serial_send
import time
import serial
#from test_serial import TestSerial
from commands import *
from camera import read_from_camera

# Global variables
STATION_COLOR = None
MAX_TRIAL_STEPS_ROUND_ONE = 9
MAX_TRIAL_STEPS_ROUND_TWO = 18


class CarHolder:  
    """Keeps track of collected cars by color"""
    
    def __init__(self):
        self.seats = []

    def add(self, car: str) -> int:
        """Add a car to holder and return total count"""
        self.seats.append(car)
        return len(self.seats)

    def remove(self, position: int) -> str:
        """Remove car at specified position and return its color"""
        car = self.seats[position]
        self.seats.pop(position)
        return car

    def count_blue(self) -> int:
        """Count number of blue cars currently held"""
        return len(list(filter(lambda t: t == BLUE_CAR, self.seats)))

    def count_red(self) -> int:
        """Count number of red cars currently held"""
        return len(list(filter(lambda t: t == RED_CAR, self.seats)))


def read_color() -> str:
    """Read color from camera sensor"""
    color = read_from_camera()
    if not color:
        print("!NO COLOR!")
        return "NO_COLOR"
    return color
    

def send_command(command: str) -> str:
    """Send command through serial connection"""
    try:
        return serial_send(command, ser)
    except Exception as e:
        print(f"Error sending command: {e}")
        return ""


def read_station_color() -> str:
    """Read color from station and send it through serial"""
    print("Read station color")
    send_command(START_COMMAND)
    time.sleep(2)
    color = read_color()
    print(f"Station color: {color}")
    return send_command(color)     
    

def demo(station_status: str):
    """Demo function to collect 4 blue cars"""
    print(f"demo {station_status}")
    if station_status:
        print(f"demo {TRIAL_BEGIN}, {station_status}")
        send_command(TRIAL_BEGIN)
        print("vado avanti...")
        
    while car_holder.count_blue() < 4:
        print("looking for blue car")
        car_color = read_color()
        print(car_color)
        if car_color == BLUE_CAR:
            print("Bluecar spotted")
            send_command(PICK_UP)
            car_holder.add(BLUE_CAR)
        else:
            print("no BLUECAR")
        send_command(TRIAL_BEGIN)
        print("looking for NEXT blue car")
        
    if car_holder.count_blue() == 4: 
        send_command(STOP_COMMAND)


def check_trial_steps(trial_steps):
    """Increment and return trial steps counter"""
    return trial_steps + 1


def round_one(station_status: str):
    """First round: collect blue cars in the first 9 steps"""
    print(f"Round_one {station_status}")
    trial_steps = 1
    
    if station_status:
        print(f"Round_one {ALLIGNE_COMMAND}, {station_status}")
        send_command(ALLIGNE_COMMAND)
        
    while trial_steps <= MAX_TRIAL_STEPS_ROUND_ONE:
        print(f"Looking for blue car {trial_steps}")
        car_color = read_color()
        print(car_color)
        
        if car_color == BLUE_CAR:
            print("Bluecar spotted")
            send_command(PICK_UP)
            car_holder.add(BLUE_CAR)
        else:
            print("no BLUECAR")
            
        trial_steps = check_trial_steps(trial_steps)
        send_command(TRIAL_BEGIN)
        print("looking for NEXT blue car")
        
    send_command(ROTATE)
    time.sleep(0.2)
    return trial_steps


def check_blues_early():
    """Check if we've already collected all 4 blue cars early"""
    return car_holder.count_blue() == 4


def round_two(station_status: str, trial_steps: int):
    """Second round: continue collecting blue cars if needed"""
    print(f"Round_two {station_status}, starting at step {trial_steps}")
    
    if station_status:
        print(f"Round_two {TRIAL_BEGIN}, {station_status}")
        send_command(TRIAL_BEGIN)
        print("Mi muovo")
        
    while trial_steps <= MAX_TRIAL_STEPS_ROUND_TWO:
        print(f"looking for blue car {trial_steps}")
        car_color = read_color()
        print(car_color)
        
        if car_color == BLUE_CAR:
            print("Bluecar spotted")
            send_command(PICK_UP)
            car_holder.add(BLUE_CAR)
            if check_blues_early():
                while trial_steps <= MAX_TRIAL_STEPS_ROUND_TWO:
                        send_command(TRIAL_BEGIN)
                        trial_steps = check_trial_steps(trial_steps)
        else:
            print("no BLUECAR")
            
        trial_steps = check_trial_steps(trial_steps)

        send_command(TRIAL_BEGIN)
        print("looking for NEXT blue car")


def deliver_blues(station_status: str):
    """Deliver collected blue cars"""
    print(f"Deliver blu at {station_status}")
    send_command(STOP_COMMAND)
    send_command(DROP_BLUE)


def deliver_red():
    """Deliver collected red cars if we have 5"""
    if car_holder.count_red() == 5:
        send_command(STOP_COMMAND)
        send_command(DROP_RED)


def reset():
    """Reset the system state"""
    print("Resetting")
    response = send_command(RESET_COMMAND)
    return response


def resetting_early():
    """Reset after early collection of all blue cars"""
    print("Resetting Early")
    send_command(RESETTING_EARLY_SEND)


def main():
    """Main program execution loop"""
    global ser
    
    try:
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        ser.reset_input_buffer()
        
        while True:
            try:

                # Main workflow
                station_status = read_station_color()
                time.sleep(1)

                trial_steps = round_one(station_status)
                time.sleep(1)

                handled = check_blues_early(station_status, trial_steps)

                if handled:
                    resetting_early()  
                else:
                    round_two(station_status, trial_steps)
                    time.sleep(1)

                    deliver_blues()
                    time.sleep(1)

                    reset()
                    time.sleep(1)
                    
                # Optional demo mode
                # demo(station_status)
                
            except Exception as e:
                print(f"Error in main loop: {e}")
                time.sleep(1)
                
    except KeyboardInterrupt:
        print("Program terminated by user")
    except Exception as e:
        print(f"Error initializing: {e}")
    finally:
        if 'ser' in globals() and ser.is_open:
            ser.close()
            print("Serial connection closed")


# Initialize car holder
car_holder = CarHolder()

# Execute main program if run as script
if __name__ == '__main__':
    main()