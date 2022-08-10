"""
Example of how to directly control a Pixhawk servo output with pymavlink.
"""

import time
# Import mavutil
from pymavlink import mavutil

def set_servo_pwm(servo_n, microseconds):
    """ Sets AUX 'servo_n' output PWM pulse-width.

    Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
    'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

    """
    master.set_servo(servo_n, microseconds)
    '''
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        servo_n,  # servo instance
        microseconds, # PWM pulse-width
        0,0,0,0,0     # unused parameters
    )
    '''

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyAMA0", 921600)
# Wait a heartbeat before sending commands
master.wait_heartbeat()


# command servo_10 to go from min to max in steps of 50us, over 2 seconds

while True: # uncomment this for a nice beat
    for us in range(1100, 1900, 100):
        set_servo_pwm(10, us)
        time.sleep(0.125)
        #set_servo_pwm(9, us)
        print("set position " + str(us))
        time.sleep(0.125)