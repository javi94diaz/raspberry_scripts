'''
JAVIKIT v1.0

API for easy use of pymavlink methods from a companion computer

'''

import os, sys, time, datetime, re
from pymavlink import mavutil


def countdown(secs):
    '''
    # Show a countdown of some seconds onscreen
    # Example
        countdown(3)

    # Output
        Count 1
        Count 2
        Count 3

    '''
    print("Counting %s seconds" %secs)
    for i in range (1, secs+1):
        print("Count " + str(i))
        time.sleep(1)


def open_logfile(filename):
    ''' 
    # Open a new log file to keep track of activity
    # Example:
        open_logfile("my_log")
        log_file.write("This will be written in the log file\n")

    # Output (in "mylog.txt"):
        *** LOGFILE: mylog.txt - 08/15/22 10:31:59 ***
        This will be written in the log file

    '''

    file_name = filename + ".txt"
    try:
        log_file = open(file_name, "x")
    except:
        log_file = open(file_name, "a")

    curr_time = datetime.datetime.now()
    log_file.write("*** LOGFILE: " + file_name + " - " + curr_time.strftime("%x %X") + " ***\n")


# Connect to the autopilot
def connect(connection_string='/dev/ttyAMA0', baudrate=921600):
    master = mavutil.mavlink_connection(connection_string, baudrate)
    master.wait_heartbeat()
    print("Connected")
    return master


# Read current flight mode from heartbeat
def get_mode():
    pass


# Set a new flight mode
def set_mode():
    pass


def arm(master):
    '''
    # Arms the aircraft using the mavutil connection object
    # Example:
        arm(master)

    # Output:
        Waiting for the vehicle to arm...
        [OK] Armed succesfully

    '''
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

        print("Waiting for the vehicle to arm...")
        master.motors_armed_wait()

        if master.motors_armed():
            print('[OK] Armed succesfully')
    except:
        print('[FAIL] Arming failed')


def disarm(master):
    '''
    # Disarms the aircraft using the mavutil connection object
    # Example:
        disarm(master)

    # Output:
        Waiting for the vehicle to disarm...
        [OK] Disarmed succesfully
    '''
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)

        print("Waiting for the vehicle to disarm...")
        master.motors_disarmed_wait()

        if not master.motors_armed():
            print('[OK] Disarmed succesfully')
    except:
        print('[FAIL] Arming failed')


# Methods to handle different types of messages
def handle_heartbeat(msg):
	mode = mavutil.mode_string_v10(msg)
	is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
	is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

def handle_rc_channels(msg):
	channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, 
			msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)#,
			#msg.chan9_raw, msg.chan10_raw, msg.chan11_raw, msg.chan12_raw, 
			#msg.chan13_raw, msg.chan14_raw, msg.chan15_raw, msg.chan16_raw, 
			#msg.chan17_raw, msg.chan18_raw)
	
	the_keys = msg.to_dict().keys()
	#r = re.compile("^chan[0-2]*[0-9]_raw$") # regex for channels up to 18
	r = re.compile("^chan[1-8]{1}_raw$") # regex for channels 1-8
	new_keys = list(filter(r.match, the_keys))
	#print(new_keys)

	for i in new_keys:
		print("{}".format(i), end=' |')
		if i == list(new_keys)[-1]:
			print('')
	
	for item in channels:
		print("{}".format(item), end=9*' ' + '|')
		if channels.index(item) == -1:
			print('')

def handle_hud(msg):
	hud_data = (msg.airspeed, msg.groundspeed, msg.heading, 
				msg.throttle, msg.alt, msg.climb)
	print("Aspd\tGspd\tHead\tThro\tAlt\tClimb")
	print("%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data)

def handle_attitude(msg):
	attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, 
				msg.pitchspeed, msg.yawspeed)
	print("Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd")
	print("%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data)

def handle_sys_status(msg):
	sys_status_data = (msg.battery_remaining, msg.current_battery, msg.load, msg.voltage_battery)
	print ("")

# Read all messages and filter them by type
def read_messages(master):

    # Request all data with certain frequency (4 in this case)
	master.mav.request_data_stream_send(master.target_system, master.target_component, 
		mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

	while(True):

		# grab a mavlink message
		msg = master.recv_match(blocking=False)
		if not msg:
			#return
			print("No message")
		else:
			#pprint.pprint(msg.to_dict())
			#print(msg.to_dict().keys())

			# handle the message based on its type
			msg_type = msg.get_type()
			print("Message " + msg_type)
			
			if msg_type == "BAD_DATA":
				if mavutil.all_printable(msg.data):
					sys.stdout.write(msg.data)
					sys.stdout.flush()
			elif msg_type == "RC_CHANNELS": 
				handle_rc_channels(msg)
			elif msg_type == "HEARTBEAT":
				handle_heartbeat(msg)
			elif msg_type == "VFR_HUD":
				handle_hud(msg)
			elif msg_type == "ATTITUDE":
				handle_attitude(msg)
			elif msg_type == "SYS_STATUS":
				handle_sys_status(msg)

			print("\n*****************")
		time.sleep(0.1)


def set_wifi(command):
    '''
    # Turns ON/OFF the Wi-Fi network in a Linux companion computer

    command must be "up" or "down" to execute a valid console command
    '''

    if command == "up" or command == "down":
        cmd = "sudo ifconfig wlan0 " + command

        try:
            os.system(cmd)
            print("[OK] Wi-Fi turned " + command)
        except:
            print("[FAIL] Wi-Fi not set")
    else:
        print("Command not valid (must be 'up' or 'down')")


# Move a servo
