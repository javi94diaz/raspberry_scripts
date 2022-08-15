from pymavlink import mavutil
import time

def countdown(secs):
    print("Counting %s seconds" %secs)
    for i in range (1, secs+1):
        print("Count " + str(i))
        time.sleep(1)


# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))

# Wait for the vehicle to send GPS_RAW_INT message
# time.sleep(10)
countdown(10)

try: 
    altitude=the_connection.messages['GPS_RAW_INT'].alt  # Note, you can access message fields as attributes!
    timestamp=the_connection.time_since('GPS_RAW_INT')
    print("Altitude is " + str(altitude))
    print("Time is " + str(timestamp))
except Exception as e:
    print(e)
    print('No GPS_RAW_INT message received')