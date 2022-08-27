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
    return x.strftime("%d/%m/%y %X")
  

class Vehicle:
    '''
    Represents a connection with the autopilot via MAVlink.
    Provides methods to receive and send information to the 
    autopilot (messages, flightmode, arming and more).
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
            #print("Waiting to connect...")
            self.master.wait_heartbeat()
            #print("[OK] Connected")
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

        #print("Current mode: " + self.get_mode())

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


    # Methods to handle different types of messages
    def handle_heartbeat(self, msg, req_properties):
        '''
        Read three fields of the heartbeat messages to display them
        Example:
            handle_heartbeat(msg)

        Output:
            Message HEARTBEAT
            Mode: Mode(0x00000000)
            Armed?: 0
            Enabled?: 0
        '''

        print("//We are in handle_heartbeat")
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


    def handle_rc_channels(self, msg, req_properties):
        '''
        Read RC channels 1 to 18 values and display them
        Example:
            handle_rc_channels(msg)

        Output:
            Message RC_CHANNELS
            chanl_raw: 0.00
            chan2 raw: 0.00
            chan3 raw: 0.00
            chan4 raw: 0.00
            chan5_raw: 0.00
            chan6_raw: 0.00
            chan7_raw: 0.00
            chan8_raw: 0.00 
            chan9 raw: 0.00
            chan10_raw: 0.00
            chanll raw: 0.00
            chan12 raw: 0.00
            chan13 raw: 0.00
            chan14 raw: 0.00
            chan15 raw: 0.00
            chan16 raw: 0.00
            chan17 raw: 0.00
            chan18_raw: 0.00
        '''

        print("//We are in handle_rc_channels")
        pprint.pprint(req_properties)

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

        ret = []
        for prop in req_properties:
            ret.append( eval("msg." + prop) )

        return ret
        

    def handle_hud(self, msg, req_properties):
        '''
        Read airspeed, groundspeed, heading, throttle, altitude and climb
        from a HUD message and display them
        Example:
            handle_hud(msg)
        
        Output:
            Message VFR_HUD
            Airspeed: 0.00
            GroundSpeed: 0.01
            Heading: 279.00
            Throttle: 0.00
            Alt: 0.00
            Climb: -0.01
        '''

        hud_data = (msg.airspeed, msg.groundspeed, msg.heading, 
                    msg.throttle, msg.alt, msg.climb)
        hud_vars = ("Airspeed", "GroundSpeed", "Heading", "Throttle", "Alt", "Climb")
        
        for i in range(0, len(hud_data)):
            print("{}: {:0.2f}".format(hud_vars[i], hud_data[i]))


        ret = []
        for prop in req_properties:
            ret.append( eval("msg." + prop) )

        return ret


    def handle_attitude(self, msg, req_properties):
        '''
        Read roll, pitch, yaw and its variation speeds: rollspeed, pitchspeed, yawspeed to display them
        Example:
            handle_attitude(msg)

        Output:
            Message ATTITUDE
            Roll: -0.01
            Pitch: -0.12
            Yaw: -1.16
            RollSpeed: -0.00
            PitchSpeed: -0.00
            YawSpeed: 0.00
        '''

        attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, 
                    msg.pitchspeed, msg.yawspeed)
        attitude_vars = ("Roll", "Pitch", "Yaw", "RollSpeed", "PitchSpeed", "YawSpeed")

        for i in range(0, len(attitude_data)):
            print("{}: {:0.2f}".format(attitude_vars[i], attitude_data[i]))
        
        ret = []
        for prop in req_properties:
            ret.append( eval("msg." + prop) )

        return ret


    def handle_sys_status(self, msg, req_properties):
        '''
        Read important fields of the system status: battery voltage, battery remaining, current battery and load to display them
        Example:
            handle_sys_status(msg)

        Output:
            Message SYS_STATUS
            Battery remaining: -1.00
            Current battery: -1.00
            Load: 500.00
            Voltage battery: 0.00

        '''
        sys_status_data = (msg.battery_remaining, msg.current_battery, msg.load, msg.voltage_battery)
        sys_status_vars = ("Battery remaining", "Current battery", "Load", "Voltage battery")
        
        for i in range(0, len(sys_status_data)):
            print("{}: {:0.2f}".format(sys_status_vars[i], sys_status_data[i]))

        ret = []
        for prop in req_properties:
            ret.append( eval("msg." + prop) )

        return ret


    def handle_global_position_int(self, msg, req_properties):
        '''
        Read alt, lat and lon and hdg from GPS positioning
        Example
            handle_global_position_int(msg)

        Output:
            Message SYS_STATUS
            Battery remaining: -1.00
            Current battery: -1.00
            Load: 500.00
            Voltage battery: 0.00

        '''
        global_position_data = (msg.alt, msg.hdg, msg.lat, msg.lon)
        global_position_vars = ("Altitude", "Heading", "Latitude", "Longitude")
        
        for i in range(0, len(global_position_data)):
            print("{}: {:0.2f}".format(global_position_vars[i], global_position_data[i]))
       
        ret = []
        for prop in req_properties:
            ret.append( eval("msg." + prop) )

        return ret


    def read_message(self, req_type, *req_properties): 
        
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

                # elif msg_type == "RC_CHANNELS" and req_type == "RC_CHANNELS":
                #     return self.handle_rc_channels(msg, req_properties)
                    
                # elif msg_type == "VFR_HUD" and req_type == "VFR_HUD":
                #     return self.handle_hud(msg, req_properties)

                # elif msg_type == "ATTITUDE" and req_type == "ATTITUDE":
                #     return self.handle_attitude(msg, req_properties)

                # elif msg_type == "SYS_STATUS" and req_type == "SYS_STATUS":
                #     return self.handle_sys_status(msg, req_properties)

                # elif msg_type == "GLOBAL_POSITION_INT" and req_type == "GLOBAL_POSITION_INT":
                #     return self.handle_global_position_int(msg, req_properties)
                
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
        Uses an especific handler method for each type of message
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.read_all_messages()

        Output
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
    drone.request_all_msgs(4)
    drone.read_all_messages()

# Request certain properties of message of the desired type
def demo4(drone, raspi):

    raspi.output("[INFO] Demo 4")
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


def main():
    print("Main function " + get_datetime())
    drone = Vehicle()
    raspi = CompanionComputer()

    drone.connect('/dev/ttyAMA0', 921600)
    raspi.output("[OK] Connected")
    
    n = input("Demo number?: ")
    
    eval("demo" + n + "(drone, raspi)")

if __name__ == '__main__':
	main()
