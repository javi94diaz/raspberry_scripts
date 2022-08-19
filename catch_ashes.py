
# Script to be uploaded, and executed in the RasPi when flying
# Uses API javikit.py to read messages from the autopilot, 
# check the battery level, the mode and the GPS location.
# With that information, moves through the states of a state machine
# (changes the flight mode or moves a servo when needed)

# Always running: method who saves battery level in an array every 1 second

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
        self.vehicle.set_mode(new_mode)
        self.vehicle.move_servo(servo_n, servo_pos)

    def change_state(next_state): # creo q no es necesario ya
        pass

    def start_countdown():
        pass




def main():

    drone = Vehicle()
    raspi = CompanionComputer()

    drone.connect('/dev/ttyAMA0', 921600)
    raspi.output("[OK] Connected")

    state1 = State("NORMAL", drone, "AUTO", 10, 1100)
    state2 = State("LOW_BATTERY", drone, "RTL", 10, 1100)
    state3 = State("CATCH_ASH", drone, "AUTO", 10, 1900)
    curr_state = state1


    # Read following variables from messages
    # call vehicle.read_messages()
    battery = 16.2 #V
    GPS_pos = [0,1,2] # alt, lat, lon 
    curr_waypoint_num = 1
    curr_countdown = 7 # seconds remaining in state3 # CAMBIAR COMO PROPIEDAD DE STATE


    # Define the transitions to set the state depending 
    # on the variables read before
    if battery <= 14.5:
        curr_state = state2

    elif GPS_pos == [4,5,6] and curr_state.name == "NORMAL":
        curr_state = state3

    elif curr_countdown == 0 and curr_state.name == "CATCH_ASH":
        curr_state = state1



if __name__ == '__main__':
	main()


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

