
# Script to be uploaded, and executed in the Raspberry Pi when flying
# Uses the API javikit.py to read messages from the autopilot, 
# check the battery level, the attitude, etc.
# With that information, evolves through the states of a state machine
# (changes the flight mode or moves a servo when needed)

import os
import sys
import time
import json
sys.path.append(os.path.abspath("/home/pi/raspberry_scripts"))
from javikit import *


class State:

    '''
    Represent one situation of the drone depending on the value 
    of several signals onboard

    The servo represents the ash catching system in the aircraft
    being deployed of folded depending on the situation
    '''
    def __init__(self, name, vehicle, new_mode, servo_n, servo_pos):
        pass
        self.name = name
        self.vehicle = vehicle
        self.mode = new_mode
        self.servo_n = servo_n
        self.servo_pos = servo_pos

    def update(self):
        self.vehicle.set_mode(self.mode)
        self.vehicle.move_servo(self.servo_n, self.servo_pos)


def catch_ashes(drone, raspi):
    '''
    Contains the state machine and the transition between states
    Saves the data in a JSON file
    '''

    drone.request_all_msgs(4)

    # Create the states
    normal_state = State("NORMAL", drone, "AUTO", 10, 1100)
    low_battery_state = State("LOW_BATTERY", drone, "RTL", 10, 1500)
    catch_ash_state = State("CATCH_ASH", drone, "LOITER", 10, 1900)
    
    # Set NORMAL as initial state
    curr_state = normal_state
    curr_state.update()
    print("Initial state: " + curr_state.name)
    countdown(2)

    # Dictionary to save the values
    data = {
        'xValues': list(range(1,11)),
        'voltage_battery': [],
        'battery_remaining': [], #percentage
        'airspeed': [],
        'climb': [],
        'alt': [],
        'groundspeed': [],
        'roll': [],
        'pitch': [],
        'yaw': []
    }

    end = 10 # Number of iterations
    for i in range(0, end):
        
        print("///////////////////////")
        raspi.output("Loop count: " + str(i))
        
        # Read values from messages
        [curr_voltage, curr_battery] = drone.read_message("SYS_STATUS", "voltage_battery", "battery_remaining")
        raspi.output("[INFO] voltage_battery: {}".format(curr_voltage))
        raspi.output("[INFO] battery_remaining: {}".format(curr_battery))

        [curr_airspeed, curr_climb, curr_alt, curr_groundspeed] = drone.read_message("VFR_HUD", "airspeed", "climb", "alt", "groundspeed")
        raspi.output("[INFO] airspeed: {}".format(curr_airspeed))
        raspi.output("[INFO] climb: {}".format(curr_climb))
        raspi.output("[INFO] alt: {}".format(curr_alt))
        raspi.output("[INFO] groundspeed: {}".format(curr_groundspeed))
        
        [curr_roll, curr_pitch, curr_yaw] = drone.read_message("ATTITUDE", "roll", "pitch", "yaw")
        raspi.output("[INFO] roll: {}".format(curr_roll))
        raspi.output("[INFO] pitch: {}".format(curr_pitch))
        raspi.output("[INFO] yaw: {}".format(curr_yaw))

        raspi.output("**********")

        # Save values to dictionary
        data['voltage_battery'].append(curr_voltage)
        data['battery_remaining'].append(curr_battery)
        data['airspeed'].append(curr_airspeed)
        data['climb'].append(curr_climb)
        data['alt'].append(curr_alt)
        data['groundspeed'].append(curr_groundspeed)
        data['roll'].append(curr_roll)
        data['pitch'].append(curr_pitch)
        data['yaw'].append(curr_yaw)

        curr_countdown = 7 - i

        # Force a change to LOW_BATTERY
        #if i == end/2:
        #    curr_voltage = -401

        # Determine current state
        nom_voltage = 11100 # millivolts
        print("Current state: " + curr_state.name)
        
        if curr_voltage <= nom_voltage and curr_state.name != "LOW_BATTERY":
            curr_state = low_battery_state
            curr_state.update()

        elif curr_yaw <= 300 and curr_state.name == "NORMAL":
            curr_state = catch_ash_state
            curr_state.update()

        elif curr_countdown == 0 and curr_state.name == "CATCH_ASH":
            curr_state = normal_state
            curr_state.update()

        print("New state: " + curr_state.name)
        time.sleep(0.5)

    raspi.output("[OK] Finished catch_ashes")
    
    # Saving dictionary to JSON file
    raspi.output("[INFO] Writing data to file...")

    with open('saved_data/data.json', 'w') as fp:
        json.dump(data, fp)

    raspi.output("[OK] Data saved!")


if __name__ == '__main__':
	
    drone = Vehicle()
    raspi = CompanionComputer()
    raspi.output("[INFO] Waiting to connect to autopilot...")

    # Adapt these values depending on the serial port
    connection_string = '/dev/ttyAMA0' 
    baud_rate = 57600

    drone.connect(connection_string, baud_rate)
    raspi.output("[OK] Connected")
    catch_ashes(drone, raspi)



# State 1: Normal
    # Battery > half of the battery
    # Mode: Auto
    # Servo: MIN position = 1100

    # Action: put servo to MIN position. Change mode to AUTO.

# State 2: Low battery
    # Battery < half of the battery
    # Mode: RTL
    # Servo: TRIM position = 1500

    # Action: changes mode to RTL and puts servo to TRIM position.

# State 3: Catch ashes
    # Battery > half of the battery
    # Mode: Loiter (or Stabilize, see more modes in documentation)
    # Servo: MAX position during 10 seconds

    # Action: moves servo to MAX position during 10 seconds, 
    # and then switches to State 1

## TRANSITIONS BETWEEN STATES
# (State 1 Normal) ---> [if battery < half of the battery] ---> (State 2 Low battery)
# (State 1 Normal) ---> [if waypoint reached] ---> (State 3 Catch ashes)
# (State 3 Catch ashes) ---> [if battery < half of the battery] ---> (State 2 Low battery)
# (State 3 Catch ashes) ---> [if countdown of 10 seconds reaches 0] ---> [State 1 Normal]

