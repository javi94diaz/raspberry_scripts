
# Script to be uploaded, and executed in the RasPi when flying
# Uses API javikit.py to read messages from the autopilot, 
# check the battery level, the mode and the GPS location.
# With that information, moves through the states of a state machine
# (changes the flight mode or moves a servo when needed)

# Always running: method who saves battery level in an array every 1 second

from itertools import count
import os
import sys
import time
sys.path.append(os.path.abspath("/home/pi/raspberry_scripts"))
from javikit import *


def save_battery_level():
    pass


class State:

    '''
    Represent one situation of the drone depending on the value of several signals onboard
    '''
    def __init__(self, name, vehicle, new_mode, servo_n, servo_pos):
        pass
        self.name = name
        self.vehicle = vehicle
        self.mode = new_mode
        self.servo_n = servo_n
        self.servo_pos = servo_pos

    def on_change(self):
        self.vehicle.set_mode(self.mode)
        self.vehicle.move_servo(self.servo_n, self.servo_pos)



def catch_ashes():

    drone = Vehicle()
    raspi = CompanionComputer()

    drone.connect('/dev/ttyAMA0', 921600)
    raspi.output("[OK] Connected")

    drone.request_all_msgs(4)

    state1 = State("NORMAL", drone, "AUTO", 10, 1100)
    state3 = State("CATCH_ASH", drone, "AUTO", 10, 1900)
    state2 = State("LOW_BATTERY", drone, "RTL", 10, 1500)
    curr_state = state1
    curr_state.on_change()
    print("Initial state: " + curr_state.name)

    countdown(5)
    
    for i in range(0,50):
        
        print("///////////////////////")
        print("Loop count: " + str(i))

        # OJO, ESMEJRO QUE PRIMERO SE FILTRE EL TIPO, Y QUE EL HANDLER DEVUELVA 
        # LA PROPIEDAD BUSCADAL, QUE SE ACCEDA AL MENSAJE SOLO DENTRO DEL HANDLER, 
        # ESO TIENE MAS SENTIDO

        # Read values from messages

        #voltage_battery = drone.read_messages("SYS_STATUS", "voltage_battery")
        voltage_battery = drone.read_next_msg("SYS_STATUS", "voltage_battery")
        print("voltage_battery battery: ", end='')
        print(voltage_battery)
        
        yaw = drone.read_next_msg("ATTITUDE", "yaw")
        print("yaw: ", end='')
        print(yaw)

        curr_countdown = 10 - i

        if i == 40:
            voltage_battery = -401

        # Determine current state
        print("Current state: " + curr_state.name)

        if voltage_battery <= -400 and curr_state.name != "LOW_BATTERY":
            curr_state = state2
            curr_state.on_change()

        elif yaw <= 300 and curr_state.name == "NORMAL":
            curr_state = state3
            curr_state.on_change()

        elif curr_countdown == 0 and curr_state.name == "CATCH_ASH":
            curr_state = state1
            curr_state.on_change()

        print("New state: " + curr_state.name)
        time.sleep(0.5)



if __name__ == '__main__':
	catch_ashes()


# State 1: Normal
    # Battery > half of the battery
    # Mode: Auto
    # Servo: MIN position

    # Action: put servo to MIN position. Change mode to AUTO.

# State 2: Low battery
    # Battery < half of the battery
    # Mode: RTL
    # Servo: MIN position

    # Action: changes mode to RTL and puts servo to MIN position.

# State 3: Catch ashes
    # Battery > half of the battery
    # Mode: Auto (or Stabilize, see more modes in documentation)
    # Servo: MAX position during 10 seconds

    # Action: moves servo to MAX position during 10 seconds, 
    # and then switches to State 1

## TRANSITIONS BETWEEN STATES
# (State 1 Normal) ---> [if battery < half of the battery] ---> (State 2 Low battery)
# (State 1 Normal) ---> [if waypoint reached] ---> (State 3 Catch ashes)
# (State 3 Catch ashes) ---> [if battery < half of the battery] ---> (State 2 Low battery)
# (State 3 Catch ashes) ---> [if countdown of 10 seconds reaches 0] ---> [State 1 Normal]

