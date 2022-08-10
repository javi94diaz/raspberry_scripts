# Test file for receiving and sending messages via mavlink

import time
import pprint # For pretty printing dictionaries
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)

print("connected, waiting for heartbeat...")
master.wait_heartbeat()
print("heartbeat received")

# Publishing a heartbet
for i in range (0,3):
    try:
        print(
            "Heartbeat from system (system %u component %u)" % (master.target_system, 
            master.target_component))
    except:
        print("Exception: No msg")
        pass
    time.sleep(0.1)

# Sending SYSTEM_TIME message
#master.mav.system_time_send(time_unix_usec, time_boot_ms)
#print("Sent system time message")

print("Messages dictionary")
pprint.pprint(master.messages)
#print(master.messages['MAV'])
#print(master.messages['HOME'])


try:
    print(master.time_since('HOME'))
    print(" seconds")
except:
    print('fail time')


try:
    print("Field")
    print(master.messages['HOME'].alt)
except:
    print('No GPS_RAW_INT message received')


print (master.mav.request_data_stream_send(
            master.target_system, 
            master.target_component, 
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1) )

