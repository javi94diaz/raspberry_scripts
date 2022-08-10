from dronekit import connect
import time

# Connect to the Vehicle
print("listeners.py")
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=921600)
print("ok connected")

# Mode listener callback
def mode_callback(self, attr_name, value):
    print("Mode changed to: ", value)

# Adding the listener callback
vehicle.add_attribute_listener('mode', mode_callback)

# Loop
i=0
while i <= 20:
    print("Waiting for MissionPlanner action [" + str(i) + "]")
    time.sleep(1)
    i+=1

# Wait 2s so callback can be notified before the observer is removed
time.sleep(2)
vehicle.remove_message_listener('mode', mode_callback)
