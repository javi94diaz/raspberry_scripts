import pprint
import time
from pymavlink import mavutil

def countdown(secs):
    print("Counting %s seconds" %secs)
    for i in range (1, secs+1):
        print("Count " + str(i))
        time.sleep(1)


print("Start")
m = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)
print("Connected")

# Wait some seconds
countdown(3)

try:
    print("a")
    pprint.pprint(m.messages)
    print("b")
except Exception as ex:
    print("An exception occurred:")
    print(ex)


# Get information
for i in range(0, 30):
    try:
        msg = m.recv_match(blocking=True).to_dict()
        pprint.pprint(msg)
        print("*************************************************")
    except Exception as e:
        print(e)

    time.sleep(0.1)

try:
    print("c")
    print(m.messages)
    print("d")
except Exception as ex:
    print("An exception occurred:")
    print(ex)

print("e")
print(m.messages['MAV'].messages)