''' ******************************************************************
    JAVIKIT v1.0
    Python API for easy use of pymavlink library from a companion computer
****************************************************************** '''

import os
import re
import sys
import time
import pprint
import datetime
import subprocess
from urllib.request import urlopen, URLError
from pymavlink import mavutil


# Chequiar la clase "mavmmaplog" que es un log file, para mi parte de post-flight
# Usar Streamlit para la web
# Buscar javascript drag and drop box for a file, to upload it in the web to the raspi


def countdown(seconds):
    '''
    Show a countdown of given seconds onscreen
    Example
        countdown(3)

    Output
        Count 1
        Count 2
        Count 3

    '''
    print("Counting {} seconds".format(seconds))
    for i in range (1, seconds+1):
        print("Count " + str(i))
        time.sleep(1)


def get_datetime():
    '''
    Returns an string with current date and time
    using dd/mm/yy hh:mm:ss format
    Example:
        print(get_datetime())
    
    Output
        08/17/22 09:47:16

    '''
    x = datetime.datetime.now()
    return x.strftime("%d/%m/%y %X") # don't know why some of them are orange
  

class Vehicle:
    '''
    Represents a connection with the autopilot via MAVlink.
    Provides methods to receive and send information to the autopilot (messages, flightmode, arming and more)
    '''

    modelist = [
        'STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 
        'GUIDED', 'LOITER', 'RTL', 'CIRCLE', 
        'POSITION', 'LAND', 'OF_LOITER', 'DRIFT', 
        'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 
        'BRAKE', 'THROW', 'AVOID_ADSB', 'GUIDED_NOGPS', 
        'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG', 
        'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL'
        ]
    

    def __init__(self):
        self.mode = "default"
        self.master = None


    def connect(self, connection_string='/dev/ttyAMA0', baudrate=921600):
        '''
        Connect to the autopilot
        Returns a mavutil connection object to handle all methods in the vehicle
        Example:
            # Connecting to an autopilot via USB cable
            drone = Vehicle()
            drone.connect('/dev/ttyAMA0', 921600)
        
        Output:
            Waiting to connect...
            [OK] Connected
        '''
        try:
            self.master = mavutil.mavlink_connection(connection_string, baudrate)
            print("Waiting to connect...")
            self.master.wait_heartbeat()
            print("[OK] Connected")
        except:
            print("[ERROR] Connection not established")

    
    def receive_heartbeat(self):
        '''
        Autopilot receives the heartbeat from the python script to prove the companion computer is alive
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.receive_heartbeat()

        Output:
            (none)
        '''
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 
            0, 0, 0)


    def get_mode(self):
        '''
        Read current flight mode
        There are two ways, the attribute "flightmode", or using the 
        heartbeat message to infer the current mode.
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.get_mode()

        Output:
            Current mode: 'STABILIZE'
        '''

        mode_num = 0
        for i in range(0, 2):
            try:
                msg = self.master.recv_match(type="HEARTBEAT", blocking=True).to_dict()
                if msg['custom_mode'] > mode_num:
                    mode_num = msg['custom_mode']
            except Exception as e:
                print(e)
            time.sleep(0.1)

        self.mode =  self.modelist[mode_num]
        return self.mode


    def set_mode(self, mode):
        '''
        Set a new mode to the aircraft
        First, it checks the current mode and then tries to change it
        The 'mode' parameter is not case sensitive
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            set_mode('LOITER')
        
        Output:
            Current mode: 'STABILIZE'
            Mode set to: 'LOITER'
        '''

        print("Current mode: " + self.get_mode())

        mode = mode.upper()
        
        # Check if given mode is valid
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit(1)

        # Get mode ID
        mode_id = self.master.mode_mapping()[mode]

        # Set new mode
        self.master.mav.command_long_send(
        self.master.target_system, self.master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
        0,
        1, 
        mode_id, 0, 0, 0, 0, 0)

        while True:
            # Wait for acknowledgement command
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()

            # Continue waiting if the acknowledged command is not `set_mode`
            if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                continue

            # Print the ACK result
            print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
            break

        print("Mode set to: " + self.get_mode())

        
    def arm(self):
        '''
        Arms the aircraft using the mavutil connection object
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.arm()

        Output:
            Waiting for the vehicle to arm...
            [OK] Armed succesfully

        '''
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0)

            print("Waiting for the vehicle to arm...")
            self.master.motors_armed_wait()

            if self.master.motors_armed():
                print('[OK] Armed succesfully')
        except:
            print('[ERROR] Arming failed')


    def disarm(self):
        '''
        Disarms the aircraft using the mavutil connection object
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.disarm()

        Output:
            Waiting for the vehicle to disarm...
            [OK] Disarmed succesfully
        '''
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0)

            print("Waiting for the vehicle to disarm...")
            self.master.motors_disarmed_wait()

            if not self.master.motors_armed():
                print('[OK] Disarmed succesfully')
        except:
            print('[ERROR] Disarming failed')


    def move_servo(self, servo_n=10, microsec=1100):
        '''
        Moves the desired servomotor servo_n to a position determined by the microseconds given for the PWM.
        The default MIN position is 1100 microseconds
        The default TRIM position is 1500 microseconds
        The default MAX position is 1900 microseconds
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.move_servo(10, 1100) # Moves servo number 10 to MIN position (1100 microseconds)
        
        Output:
            (the servo moves to desired position)
        '''

        try:

            self.master.mav.command_long_send(
                self.master.target_system, 
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,            # first transmission of this command
                servo_n,  # servo instance
                microsec, # PWM pulse-width
                0,0,0,0,0     # unused parameters
            )

            # self.master.set_servo(servo_n, microsec)
            
            print("[OK] Servo "  + str(servo_n) + " moved to " + str(microsec))
        except:
            print("[ERROR] Servo position not set")


    # Methods to handle different types of messages
    def handle_heartbeat(self, msg):
        mode = mavutil.mode_string_v10(msg)
        is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        print("Mode: {}".format(mode))
        print("Armed?: {}".format(is_armed))
        print("Enabled?: {}".format(is_enabled))

    def handle_rc_channels(self, msg):
        channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, 
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
                msg.chan9_raw, msg.chan10_raw, msg.chan11_raw, msg.chan12_raw, 
                msg.chan13_raw, msg.chan14_raw, msg.chan15_raw, msg.chan16_raw, 
                msg.chan17_raw, msg.chan18_raw)
        
        the_keys = msg.to_dict().keys()
        r = re.compile("^chan[0-2]*[0-9]_raw$") # regex for channels up to 18
        #r = re.compile("^chan[1-8]{1}_raw$") # regex for channels 1-8
        new_keys = list(filter(r.match, the_keys))

        for i in range(0, len(channels)):
            print("{}: {:0.2f}".format(new_keys[i], channels[i]))


    def handle_hud(self, msg):
        hud_data = (msg.airspeed, msg.groundspeed, msg.heading, 
                    msg.throttle, msg.alt, msg.climb)
        hud_vars = ("Airspeed", "GroundSpeed", "Heading", "Throttle", "Alt", "Climb")
        
        for i in range(0, len(hud_data)):
            print("{}: {:0.2f}".format(hud_vars[i], hud_data[i]))


    def handle_attitude(self, msg):
        attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, 
                    msg.pitchspeed, msg.yawspeed)
        attitude_vars = ("Roll", "Pitch", "Yaw", "RollSpeed", "PitchSpeed", "YawSpeed")

        for i in range(0, len(attitude_data)):
            print("{}: {:0.2f}".format(attitude_vars[i], attitude_data[i]))


    def handle_sys_status(self, msg):
        sys_status_data = (msg.battery_remaining, msg.current_battery, msg.load, msg.voltage_battery)
        sys_status_vars = ("Battery remaining", "Current battery", "Load", "Voltage battery")
        
        for i in range(0, len(sys_status_data)):
            print("{}: {:0.2f}".format(sys_status_vars[i], sys_status_data[i]))


    def request_all_msgs(self, freq=4):
        '''
        Request all types of messages with given frequency (4 by default)
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            freq = 8
            drone.request_all_msgs(freq)
        
        Output:
            (none)
        '''

        self.master.mav.request_data_stream_send(
            self.master.target_system, 
            self.master.target_component, 
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 
            freq, 1)


    def read_messages(self):
        '''
        Read all messages from mavlink stream
        Uses an especific handler method for each type of message
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.read_messages()

        Output
            (See 'handle_[type_of_message]()' methods for each type of message)
        '''

        # First, request all types of messages to the stream
        freq = 4
        self.request_all_msgs(freq)

        while(True):

            msg = self.master.recv_match(blocking=False)
            if not msg:
                #return
                print("No message")
            else:
                #pprint.pprint(msg.to_dict())
                # handle the message based on its type
                msg_type = msg.get_type()
                print("Message " + msg_type)

                if msg_type == "BAD_DATA":
                    if mavutil.all_printable(msg.data):
                        sys.stdout.write(msg.data)
                        sys.stdout.flush()
                elif msg_type == "RC_CHANNELS": 
                    self.handle_rc_channels(msg)
                elif msg_type == "HEARTBEAT":
                    self.handle_heartbeat(msg)
                elif msg_type == "VFR_HUD":
                    self.handle_hud(msg)
                elif msg_type == "ATTITUDE":
                    self.handle_attitude(msg)
                elif msg_type == "SYS_STATUS":
                    self.handle_sys_status(msg)

                print("\n*****************")
            time.sleep(0.2)



class CompanionComputer:

    def __init__(self):
        self.log_file = self.open_logfile("log")


    def open_logfile(self, filename):
        ''' 
        Open a new log file to keep track of activity
        Example:
            my_computer = CompanionComputer()
            my_log = my_computer.open_logfile("my_log")
            my_log.write("This will be written in the log file\n")

        Output (in "mylog.txt"):
            *** LOGFILE: mylog.txt - 08/15/22 10:31:59 ***
            This will be written in the log file
        '''

        file_name = filename + ".txt"
        try:
            log_file = open(file_name, "x")
        except:
            log_file = open(file_name, "a")

        curr_time = datetime.datetime.now()
        log_file.write("\n*** LOGFILE: " + file_name + " - " + curr_time.strftime("%d/%m/%y %X") + " ***\n")
        return log_file


    def output(self, msg):
        '''
        Print a message on terminal and in a log file
        Example:
            my_computer = CompanionComputer()
            my_log = my_computer.open_logfile("my_log")
            my_computer.output("[OK] This will be printed and logged")

        Output:
            (in console)        [OK] This will be printed and logged
            (in 'my_log.txt')   [OK] This will be printed and logged
        '''
        print(msg)
        self.log_file.write(msg + "\n")


    def set_wifi(self, command):
        '''
        Turns ON/OFF the Wi-Fi network in a Linux companion computer
        The 'command' argument must be "up" or "down" to perform a valid Linux command
        Example:
            my_computer = CompanionComputer()
            my_computer.set_wifi("up")

        Output:
            [OK] Wi-Fi turned up
            '''

        if command == "up" or command == "down":
            cmd = "sudo ifconfig wlan0 " + command

            try:
                os.system(cmd)
                print("[OK] Wi-Fi turned " + command)
            except:
                print("[ERROR] Wi-Fi not set")
        else:
            print("Command not valid (must be 'up' or 'down')")


    def check_internet(self, url):
        '''
        Check if internet connection is available opening a given url
        Example:
            my_computer = CompanionComputer()
            my_computer.check_internet('http://216.58.192.142'):
            
        
        Output:
            [OK] Internet available (in both console and log file)
        '''
        try:
            urlopen(url, timeout=10)
            self.output("[INFO] Internet available")
            return True
        except Exception as exc:
            print(exc)
            self.output("[INFO] Internet not available")
            return False

    def check_wifi(self):
        '''
        Check if Wi-Fi internet is set up or not with a shell command
        Example:
            my_computer = CompanionComputer()
            my_computer.check_wifi()
            
        Output (in console and log file):
            [INFO] Wi-Fi Up
        '''
        result = subprocess.run(['iwconfig', 'wlan0'], stdout=subprocess.PIPE)
        str_stdout = result.stdout.decode('utf-8')
        index = str_stdout.find('ESSID:') + 6

        if str_stdout[index:index+7] == 'off/any':
            self.output("[INFO] Wi-Fi Down")
        else:
            self.output("[INFO] Wi-Fi Up")



# Get mode, set mode, move servo
def demo1(drone, raspi):
    raspi.output("[INFO] Demo 1")

    print("Initial mode: " + drone.get_mode())
    newmode = input("Introduce the new mode: ")
    newmode.upper()

    drone.set_mode(newmode)
    raspi.output("[OK] Mode set to " + newmode)

    pos = int(input("Introduce the servo position: "))
    drone.move_servo(10, pos)
    raspi.output("[OK] Servo position set")


# Arm, turn Wi-Fi off, wait, disarm, turn Wi-Fi on
def demo2(drone, raspi):
    raspi.output("[INFO] Demo 2")


    drone.arm()
    raspi.set_wifi("down")

    # Give 3 seconds to turn off Wi-Fi
    countdown(3)

    #raspi.check_internet('http://216.58.192.142')
    raspi.check_wifi()

    drone.disarm()
    raspi.set_wifi("up")

    # Give 7 seconds to reconnect to a Wi-Fi network
    countdown(7)
 
    #raspi.check_internet('https://bing.com')
    raspi.check_wifi()

# Read all messages and display the info
def demo3(drone, raspi):
    raspi.output("[INFO] Demo 3")
    drone.read_messages()


def main():
    print("Main " + get_datetime())
    drone = Vehicle()
    raspi = CompanionComputer()

    drone.connect('/dev/ttyAMA0', 921600)
    raspi.output("[OK] Connected")
    
    n = int(input("Demo number?: "))
    if n==1:
        demo1(drone, raspi)
    elif n==2:
        demo2(drone, raspi)
    elif n==3:
        demo3(drone, raspi)



if __name__ == '__main__':
	main()
