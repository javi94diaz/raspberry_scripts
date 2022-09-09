''' ***********************************************************************
    bristol_API
    Python API for easy use of pymavlink library from a companion computer
*********************************************************************** '''

import os
import re
import sys
import time
import pprint
import datetime
import subprocess
from urllib.request import urlopen
from pymavlink import mavutil


def countdown(seconds):
    '''
    Show a countdown of given seconds onscreen
    Example:
        countdown(3)

    Output:
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
    Returns a string with current date and time
    using dd/mm/yy hh:mm:ss format
    Example:
        print(get_datetime())
    
    Output:
        08/17/22 09:47:16

    '''
    x = datetime.datetime.now()
    return x.strftime("%d/%m/%y %X")
  

class Vehicle:
    '''
    Represents a connection with the autopilot via MAVlink.
    Provides methods to receive and send information to the 
    autopilot (messages, flightmode, arming and more).
    
    '''
    modelist_copter = [
        'STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 
        'GUIDED', 'LOITER', 'RTL', 'CIRCLE', 
        'POSITION', 'LAND', 'OF_LOITER', 'DRIFT', 
        'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 
        'BRAKE', 'THROW', 'AVOID_ADSB', 'GUIDED_NOGPS', 
        'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG', 
        'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL'
        ]

    modelist_plane = [
        'MANUAL', 'CIRCLE', 'STABILIZE', 'TRAINING', 
        'ACRO', 'FBWA', 'FBWB', 'CRUISE', 
        'AUTOTUNE','no_mode','AUTO', 'RTL', 'LOITER', 
        'TAKEOFF', 'AVOID_ADSB', 'GUIDED', 'INITIALISING', 
        'QSTABILIZE', 'QHOVER', 'QLOITER', 'QLAND', 
        'QRTL', 'QAUTOTUNE', 'QACRO', 'THERMAL', 
        'LOITERALTQLAND'
        ]

    mode_map_plane = {
        'MANUAL': 0, 
        'CIRCLE': 1, 
        'STABILIZE': 2, 
        'TRAINING': 3, 
        'ACRO': 4, 
        'FBWA': 5, 
        'FBWB': 6, 
        'CRUISE': 7, 
        'AUTOTUNE': 8, 
        'AUTO': 10, 
        'RTL': 11, 
        'LOITER': 12, 
        'TAKEOFF': 13, 
        'AVOID_ADSB': 14, 
        'GUIDED': 15, 
        'INITIALISING': 16, 
        'QSTABILIZE': 17, 
        'QHOVER': 18, 
        'QLOITER': 19, 
        'QLAND': 20, 
        'QRTL': 21, 
        'QAUTOTUNE': 22, 
        'QACRO': 23, 
        'THERMAL': 24, 
        'LOITERALTQLAND': 25
    }


    def __init__(self):
        self.mode = "default"
        self.master = None


    def connect(self, connection_string, baudrate=57600):
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
            drone.set_mode('AUTO')

            # Check if the mode was succesfully updated
            drone.get_mode()

        Output:
            Current mode: 'AUTO'
        
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

        #self.mode =  self.modelist_copter[mode_num]
        self.mode =  self.modelist_plane[mode_num]
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
        mode = mode.upper()

        #print("Current mode list: ")
        #print(self.master.mode_mapping())

        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit(1)

        # Get mode number ID
        mode_id = self.master.mode_mapping()[mode]

        # Set new mode
        self.master.mav.command_long_send(
        self.master.target_system, 
        self.master.target_component,
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
            #print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
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
                0,
                servo_n,
                microsec,
                0,0,0,0,0
            )

            # self.master.set_servo(servo_n, microsec)
            
            print("[OK] Servo "  + str(servo_n) + " moved to " + str(microsec))
        except:
            print("[ERROR] Servo position not set")


    def request_logs(self):
        '''
        Request the log list from the autopilot
        Example:
            (NOT TESTED)

        Output:
            (UNKNOWN)

        '''
        self.master.mav.command_long_send(
        self.master.target_system, 
        self.master.target_component,
        mavutil.mavlink.LOG_REQUEST_LIST, 
        0,
        0, 
        hex(0XFFFF), 0, 0, 0, 0, 0)

        while True:
            # Wait for acknowledgement command
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()

            #print(ack_msg)

            # Continue waiting if the acknowledged command is not `set_mode`
            if ack_msg['command'] != mavutil.mavlink.LOG_REQUEST_LIST:
                print("not yet ack")
                continue

            # Print the ACK result
            print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
            break


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


    def handle_heartbeat(self, msg, req_properties):
        '''
        Read three fields of the heartbeat messages to display them
        Example:
            handle_heartbeat(msg)

        Output:
            Message HEARTBEAT
            Mode: RTL
            Armed?: 0
            Enabled?: 0
        
        '''
        print("*** We are in handle_heartbeat ***")
        pprint.pprint(req_properties)

        mode = mavutil.mode_string_v10(msg)
        is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        print("Mode: {}".format(mode))
        print("Armed?: {}".format(is_armed))
        print("Enabled?: {}".format(is_enabled))
        
        ret = []
        for prop in req_properties:
            ret.append( eval(prop) )

        return ret
    

    def read_message(self, req_type, *req_properties): 
        '''
        Listen to the last message of the type indicated.
        It is also possible to request some specific properties
        inside that message
        
        Example:
            [airspeed, alt, groundspeed] = drone.read_message("VFR_HUD", "airspeed", "alt", "groundspeed")
        
        Output:
            (The variables airspeed, alt and groundspeed contain the values read)
        
        '''
        #pprint.pprint(req_properties)

        while(True):

            msg = self.master.recv_match(blocking=False)
            if not msg:
                print("No message")
            else:
                msg_type = msg.get_type()
                #print("Message " + msg_type)
                #print("Req_type " + req_type)

                if msg_type == "BAD_DATA":
                    if mavutil.all_printable(msg.data):
                        sys.stdout.write(msg.data)
                        sys.stdout.flush()

                elif msg_type == req_type:

                    if msg_type == "HEARTBEAT" and req_type == "HEARTBEAT":
                        return self.handle_heartbeat(msg, req_properties)
                    else:
                        ret = []
                        for prop in req_properties:
                            ret.append( eval("msg." + prop) )

                        return ret

                else:
                    #pprint.pprint(msg.to_dict())
                    pass

            #print("\n*****************")
            time.sleep(0.1)
   

    def read_all_messages(self): 
        '''
        Read all messages from mavlink stream
        Blocking method
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.read_all_messages()

        Output:
            (The message with all its fields is printed. The output varies depending on the type of message)
        
        '''
        while(True):
            
            msg = self.master.recv_match(blocking=False)
            if not msg:
                print("No message\n*****************")
            else:
                msg_type = msg.get_type()
                print("Message " + msg_type)

                if msg_type == "BAD_DATA":
                    if mavutil.all_printable(msg.data):
                        sys.stdout.write(msg.data)
                        sys.stdout.flush()
                else:
                    pprint.pprint(msg.to_dict())

                print("\n*****************")
            time.sleep(0.1)


class CompanionComputer:
    '''
    Represents a companion computer, such as a Raspberry Pi.
    It wraps some methods to log information an manage wlan interface
    
    '''

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
            return False
        else:
            self.output("[INFO] Wi-Fi Up")
            return True


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


# Get and set mode
def demo1(drone, raspi):

    raspi.output("[INFO] Demo 1 - get/set mode")

    print("Initial mode: " + drone.get_mode())
    newmode = input("Introduce the new mode: ")
    newmode.upper()

    drone.set_mode(newmode)
    raspi.output("[OK] Mode set to " + newmode)


# Move servo
def demo2(drone, raspi):
    raspi.output("[INFO] Demo 2 - move servo")

    num = int(input("Introduce the servo number: "))
    pos = int(input("Introduce the servo position: "))
    drone.move_servo(num, pos)
    raspi.output("[OK] Servo position set")


# Arm, turn Wi-Fi off, wait, disarm, turn Wi-Fi on
def demo3(drone, raspi):
    raspi.output("[INFO] Demo 3 - arm/disarm, Wi-Fi on/off")

    drone.arm()
    raspi.set_wifi("down")

    # Give 3 seconds to turn off Wi-Fi
    countdown(3)

    #raspi.check_internet('http://216.58.192.142')
    raspi.check_wifi()

    drone.disarm()
    raspi.set_wifi("up")

    # Give 6 seconds to connect to a Wi-Fi network
    countdown(6)
 
    #raspi.check_internet('https://bing.com')
    raspi.check_wifi()


# Read all messages and display the info
def demo4(drone, raspi):
    raspi.output("[INFO] Demo 4 - read all messages")
    drone.request_all_msgs(4)
    drone.read_all_messages()


# Request certain properties of message of the desired type
def demo5(drone, raspi):

    raspi.output("[INFO] Demo 5 - request properties")
    drone.request_all_msgs(4)
    print( drone.read_message("HEARTBEAT", "mode", "is_enabled") )
    countdown(2)
    print( drone.read_message("RC_CHANNELS", "chan1_raw", "chan3_raw", "chan17_raw") )
    countdown(2)
    print( drone.read_message("VFR_HUD", "airspeed", "climb", "alt", "groundspeed") )
    countdown(2)
    print( drone.read_message("ATTITUDE", "roll") )
    countdown(2)
    print( drone.read_message("SYS_STATUS", "battery_remaining", "current_battery") )
    countdown(2)


# Get some specific messages from the data stream
def demo6(drone, raspi):

    raspi.output("[INFO] Demo 6 - get specific messages")

    while(True):
        [voltage_battery] = drone.read_message("SYS_STATUS", "voltage_battery")
        
        raspi.output("[INFO] voltage_battery: {}".format(voltage_battery))

        [airspeed, climb, alt, groundspeed] = drone.read_message("VFR_HUD", "airspeed", "climb", "alt", "groundspeed")

        raspi.output("[INFO] airspeed: {}".format(airspeed))
        raspi.output("[INFO] climb: {}".format(climb))
        raspi.output("[INFO] alt: {}".format(alt))
        raspi.output("[INFO] groundspeed: {}".format(groundspeed))
        raspi.output("**********")
        
        time.sleep(1)


# Run catch_ashes uploaded script
def demo7(drone, raspi):

    raspi.output("[INFO] Demo 7 - call catch_ashes")
    from uploaded_scripts.catch_ashes import catch_ashes
    catch_ashes(drone, raspi)


def main():
    print("Main function " + get_datetime())
    drone = Vehicle()
    raspi = CompanionComputer()
    
    raspi.output("Waiting to connect...")

    drone.connect('/dev/ttyAMA0', 57600)
    
    print("Demo number?: ")
    print("1: Get/Set mode\n2: Move servo\n3: Arm/Disarm, Wi-Fi On/Off")
    print("4: read messages\n5: request properties\n6: read specific messages")
    print("7: call catch_ashes example")
    
    n = input()
    eval("demo" + n + "(drone, raspi)")


if __name__ == '__main__':
	main()
