import pprint
import time
from pymavlink import mavutil


def wait_conn():
    msg = None
    while not msg:
        the_connection.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = the_connection.recv_match()
        time.sleep(0.5)


print("Start")
the_connection = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)
print("Connection 1")

# Send a ping to start connection and wait for any reply
wait_conn()
print("Connection 2")

#the_connection.mav.request_data_stream_send()

# Get some information
for i in range (0,10):
    try:

        msg= the_connection.recv_match(type='SERVO_OUTPUT_RAW').to_dict()
        pprint.pprint(msg)

    except:
        print("Exception: No msg")
        pass
    time.sleep(0.1)

try:
    the_connection.mav.request_data_stream_send(
        the_connection.target_system, 
        the_connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        args.rate, 1)

except:
    print("Fail")