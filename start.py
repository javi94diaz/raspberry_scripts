# start.py
# Automatizes pre-flight checks

#NOTAS
# Hay un mensaje llamado MAV_CMD_PREFLIGHT_CALIBRATION, mirar de que va
# tambien el SYS_STATUS, que parece que tiene de todo lo que me hace falta
# tambien el MAV_STATE, y el MAV_MODE


from dronekit import connect
from pymavlink import mavutil
import os
import datetime

def request_msg():
    pass




x = datetime.datetime.now()
print(x.strftime("%x %X"))

# pymavlink connection
master = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)
print("Connected to mavlink")
master.wait_heartbeat()



# # Send heartbeat from a MAVLink application (from the script running on Raspberry Pi)
# master.mav.heartbeat_send(
#     mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
#     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)



# Open the log file for the raspberry
try:
    raspi_log = open("raspi_log.txt", "x")
except:
    raspi_log = open("raspi_log.txt", "a")

curr_time = datetime.datetime.now()
raspi_log.write("*** LOGFILE RASPBERRY PI 4 - " + curr_time.strftime("%x %X") + " ***\n")

# Flags to raise in each check
flag_check1 = False
flag_pilot = False
flag_armed = False
flag_wifi_off = False
flag_servo_home = False

# Leer un valor (modo, por ejemplo), verificar que esta dentro del rango
print("Checking mode...")

mode = "AUTO"

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    raspi_log.write("[FAIL] Mode not available\n")
else:
    
    mode_id = master.mode_mapping()[mode]

    # Set new mode
    master.mav.command_long_send(
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
    0, 
    1, 
    10, # 10 = AUTO
    0, 0, 0, 0, 0)

    #master.set_mode_send(mode_id)
    
    
    print("[OK] Mode " + str(mode) + " set")
    raspi_log.write("[OK] Mode " + str(mode) + " set\n")
    flag_check1 = True

# Requesting the pilot for confirmation of the manual checks
input("All checks passed, waiting for Pilot confirmation") # Press any key...
print("[OK] Pilot confirm")
raspi_log.write("[OK] Pilot confirm\n")
flag_pilot = True

# Arm the aircraft
print('Arming...')

try:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # Wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('[OK] Armed')
    raspi_log.write("[OK] Armed\n")
    flag_armed = True
except:
    print('[FAIL] Armed')
    raspi_log.write('[FAIL] Armed\n')

# Turn Wi-Fi off
print("Turning Wi-Fi off...")
cmd = "sudo ifconfig wlan0 down"

try:
    os.system(cmd)
    print("[OK] Wi-Fi down")
    raspi_log.write("[OK] Wi-Fi down\n")
    flag_wifi_off = True
except:
    print("[FAIL] Wi-Fi down")
    raspi_log.write("[FAIL] Wi-Fi down\n")

# Move servo to home position (ready to receive commands) = MIN pwm value
print("Moving servo to home position...")
servo_n = 10
microseconds = 1100

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

    master.set_servo(servo_n, microseconds)

    print("[OK] Servo at home")
    raspi_log.write("[OK] Servo at home\n")
    flag_servo_home = True
except:
    print("[FAIL] Servo at home")
    raspi_log.write("[FAIL] Servo at home\n")

print("Flags: ")
print(flag_check1) 
print(flag_pilot)
print(flag_armed)
print(flag_wifi_off)
print(flag_servo_home)

# Write down the start datetime in the log file
if flag_check1 and flag_pilot and flag_armed and flag_wifi_off and flag_servo_home:
    x = datetime.datetime.now()
    print("Start datetime " + x.strftime("%x %X"))
    raspi_log.write("Start datetime " + x.strftime("%x %X\n"))
else:
    print("[FAIL] Start")
    raspi_log.write("[FAIL] Start\n")

# Take off and execute the next script: flying_modes.py
# ...
# ...