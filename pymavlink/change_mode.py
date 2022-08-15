import sys
import time
from pymavlink import mavutil
import pprint

def countdown(secs):
    print("Counting %s seconds" %secs)
    for i in range (1, secs+1):
        print("Count " + str(i))
        time.sleep(1)

# Connection with the autopilot
master = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)
master.wait_heartbeat()

# # Send heartbeat from a MAVLink application (from the script running on Raspberry Pi)
# master.mav.heartbeat_send(
#     mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
#     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

#countdown(5)

# Ver el modo actual
print("Current mode directamente:")
print(master.flightmode)


print("Current mode con heartbeat:")
for i in range(0, 2):
    try:
        msg = master.recv_match(type="HEARTBEAT", blocking=True).to_dict()
        pprint.pprint(msg['base_mode'])
        pprint.pprint(msg['custom_mode'])
        print("*************************************************")
    except Exception as e:
        print(e)

    time.sleep(0.1)



# Choose a mode
#mode = 'LOITER'
mode = input("Introduce the new mode: ")

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)


# Imprimimos la tabla hash de modos y su numero asociado
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
    'AUTO_RTL']

print("HASH TABLE - mode_mapping")
for i in modelist:
    mode_num = master.mode_mapping()[i]
    print(i + ": " +  str(mode_num))

# Get mode ID
mode_id = master.mode_mapping()[mode]
print("Mode id: " + str(mode_id))

# Set new mode
master.mav.command_long_send(
   master.target_system, master.target_component,
   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
   0,
   1, 
   mode_id, 0, 0, 0, 0, 0)

# Probar este
# master.set_mode(mode_id)

# ESTE FUNCIONA BIEN CREO
# master.mav.set_mode_send(
#     master.target_system,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     mode_id)

#countdown(5)


while True:
    # Wait for ACK command
    # Would be good to add mechanism to avoid endlessly blocking
    # if the autopilot sends a NACK or never receives the message
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()
    print (ack_msg)

    # Continue waiting if the acknowledged command is not `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break




# Chequear el nuevo modo directamente
print("Mode changed to:")
print(master.flightmode)

# chequear nuevo modo con heartbeat
print("Nuevo mode con heartbeat:")
for i in range(0, 2):
    try:
        msg = master.recv_match(type="HEARTBEAT", blocking=True).to_dict()
        pprint.pprint(msg['base_mode'])
        pprint.pprint(msg['custom_mode'])
        print("*************************************************")
    except Exception as e:
        print(e)

    time.sleep(0.1)