''' ******************************************************************

JAVIKIT v1.0

Python API for easy use of pymavlink methods from a companion computer

List of contents:
    modelist[]
    countdown()
    open_logfile()
    connect()
    get_mode()
    set_mode()
    arm()
    disarm()
    handle_heartbeat()
    handle_rc_channels()
    handle_hud()
    handle_attitude()
    handle_sys_status()
    read_messages()
    set_wifi()
    move_servo()
    get_datetime()
    request_msg()


****************************************************************** '''

import os, sys, time, datetime, re, pprint
from pymavlink import mavutil

# Chequiar la clase "mavmmaplog" que es un log file, para mi parte de post-flight

modelist = [
    'STABILIZE', 
    'ACRO', 
    'ALT_HOLD', 
    'AUTO', 
    'GUIDED', 
    'LOITER', 
    'RTL', 
    'CIRCLE', 
    'POSITION', 
    'LAND', 
    'OF_LOITER', 
    'DRIFT', 
    'SPORT', 
    'FLIP', 
    'AUTOTUNE', 
    'POSHOLD', 
    'BRAKE', 
    'THROW', 
    'AVOID_ADSB', 
    'GUIDED_NOGPS', 
    'SMART_RTL', 
    'FLOWHOLD', 
    'FOLLOW', 
    'ZIGZAG', 
    'SYSTEMID', 
    'AUTOROTATE', 
    'AUTO_RTL'
    ]


def countdown(seconds):
    '''
    # Show a countdown of some seconds onscreen
    # Example
        countdown(3)

    # Output
        Count 1
        Count 2
        Count 3

    '''
    print("Counting {} seconds".format(seconds))
    for i in range (1, seconds+1):
        print("Count " + str(i))
        time.sleep(1)


def open_logfile(filename):
    ''' 
    # Open a new log file to keep track of activity
    # Example:
        my_log = open_logfile("my_log")
        my_log.write("This will be written in the log file\n")

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
    log_file.write("\n*** LOGFILE: " + file_name + " - " + curr_time.strftime("%x %X") + " ***\n")

    return log_file

def connect(connection_string='/dev/ttyAMA0', baudrate=921600):
    '''
    # Connect to the autopilot
    # Returns a mavutil connection object to handle all methods in the vehicle
    # Example:
        master = connect('/dev/ttyAMA0', 921600)
    
    # Output:
        Waiting to connect...
        [OK] Connected
    '''
    try:
        master = mavutil.mavlink_connection(connection_string, baudrate)
        print("Waiting to connect...")
        master.wait_heartbeat()
        print("[OK] Connected")
        return master
    except:
        print("[FAIL] Connection not established")


def get_mode(master):
    '''
    # Read current flight mode
    # There are two ways, the attribute "flightmode", or using the 
    # heartbeat message to infer the current mode.
    # Example:
        get_mode(master)

    # Output:
        Current mode: 'STABILIZE'
    '''

    #print("Current mode directamente:")
    #print(master.flightmode)

    #print("Current mode con heartbeat:")
    mode_num = 0
    for i in range(0, 2):
        try:
            msg = master.recv_match(type="HEARTBEAT", blocking=True).to_dict()
            #pprint.pprint(msg['base_mode'])
            #pprint.pprint(msg['custom_mode'])
            if msg['custom_mode'] > mode_num:
                mode_num = msg['custom_mode']
            #print("*************************************************")
        except Exception as e:
            print(e)
        time.sleep(0.1)

    mode =  modelist[mode_num]
    #print("Current mode: " + mode)
    return mode

def set_mode(master, mode):
    '''
    # Set a new mode to the aircraft
    # First, it checks the current mode and then tries to change it
    # Example:
        set_mode(master, 'LOITER')
    
    # Output:
        Mode changed to 'LOITER'
        (check the GCS to see if it's actually changed)
    '''

    print("Current mode: " + get_mode(master))

    mode = mode.upper()
    
    # Check if mode is available
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        sys.exit(1)

    # Imprimimos la tabla hash de modos y su numero asociado
    # print("HASH TABLE - mode_mapping")
    # for i in modelist:
    #     mode_num = master.mode_mapping()[i]
    #     print(i + ": " +  str(mode_num))

    # Get mode ID
    mode_id = master.mode_mapping()[mode]
    #print("Mode id: " + str(mode_id))

    # Set new mode
    master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
    0,
    1, 
    mode_id, 0, 0, 0, 0, 0)

    while True:
        # Wait for ACK command
        # Would be good to add mechanism to avoid endlessly blocking
        # if the autopilot sends a NACK or never receives the message
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()
        #print (ack_msg)

        # Continue waiting if the acknowledged command is not `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

    print("New mode: " + get_mode(master))


def arm(master):
    '''
    # Arms the aircraft using the mavutil connection object
    # Example:
        master = connect('/dev/ttyAMA0', 921600)
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
        master = connect('/dev/ttyAMA0', 921600)
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

def request_msg(master, freq=4):
    '''
    # Request all types of messages with certain frequency (4 in this case)
    # Example:

    '''

    # Poner la alternativa con command_long MAV_DO_REQUEST_MSG o similar

    master.mav.request_data_stream_send(
        master.target_system, 
        master.target_component, 
		mavutil.mavlink.MAV_DATA_STREAM_ALL, 
        freq, 1)

# Read all messages and filter them by type
def read_messages(master):
    '''
    # Read all messages from mavlink stream
    # Uses an especific handler method for each type of message
    # Example:
        master = connect('/dev/ttyAMA0', 921600)
        read_messages(master)
    '''
    # Request all types of messages to the stream
    request_msg(master, 4)

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
    # command argument must be "up" or "down" to execute a valid console command
    # Example:
        set_wifi("up")

    # Output:
        [OK] Wi-Fi turned up
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


# Move servo to home position (ready to receive commands) = MIN pwm value
def move_servo(master, servo_n=10, microsec=1100):
    '''
    # Moves the desired servomotor servo_n to a position determined
    # by the microseconds given for the PWM.
    # The default MIN position is 1100 microseconds
    # The default TRIM position is 1500 microseconds
    # The default MAX position is 1900 microseconds
    # Example:
        master = connect('/dev/ttyAMA0', 921600)
        move_servo(master, 10, 1100) # Moves servo number 10 to MIN position
    '''

    try:

        # master.mav.command_long_send(
        #     master.target_system, 
        #     master.target_component,
        #     mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        #     0,            # first transmission of this command
        #     servo_n,  # servo instance
        #     microseconds, # PWM pulse-width
        #     0,0,0,0,0     # unused parameters
        # )

        master.set_servo(servo_n, microsec)
        
        print("[OK] Servo "  + str(servo_n) + " moved to " + str(microsec))
    except:
        print("[FAIL] Servo at home")


# Get current date and time
def get_datetime():
    x = datetime.datetime.now()
    return x.strftime("%x %X")


# # Send heartbeat from a MAVLink application (from the script running on Raspberry Pi)
# master.mav.heartbeat_send(
#     mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
#     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)



def main():
    print(get_datetime())
    master = connect('/dev/ttyAMA0', 921600)
    javilog = open_logfile("javilog")
    javilog.write("javikit 1")
    #read_messages(master)
    print("Initial mode: " + get_mode(master))
    set_mode(master, 'LOITER')
    move_servo(master, 10, 1100)


if __name__ == '__main__':
	main()