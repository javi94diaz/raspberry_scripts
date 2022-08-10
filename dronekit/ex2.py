from dronekit import connect
import time

# Connect to the Vehicle
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=921600)
print("ok connected")

# Printing almost all vehicle attributes
print("Autopilot Firmware version: %s" % vehicle.version)
print("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
print("Global Location: %s" % vehicle.location.global_frame)
print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print("Local Location: %s" % vehicle.location.local_frame)    #NED
print("Attitude: %s" % vehicle.attitude)
print("Velocity: %s" % vehicle.velocity)
print("GPS: %s" % vehicle.gps_0)
print("Groundspeed: %s" % vehicle.groundspeed)
print("Airspeed: %s" % vehicle.airspeed)
print("Gimbal status: %s" % vehicle.gimbal)
print("Battery: %s" % vehicle.battery)
print("EKF OK?: %s" % vehicle.ekf_ok)
print("Last Heartbeat: %s" % vehicle.last_heartbeat)
print("Rangefinder: %s" % vehicle.rangefinder)
print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
print("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
print("Heading: %s" % vehicle.heading)
print("Is Armable?: %s" % vehicle.is_armable)
print("System status: %s" % vehicle.system_status.state)
print("Mode: %s" % vehicle.mode.name)    # settable
print("Armed: %s" % vehicle.armed)    # settable

# Showing all parameters
print("\nPrint all parameters (iterate 'vehicle.parameters'):")
for key, value in vehicle.parameters.items():
    print(" Key:%s Value:%s" % (key,value))

# Listener for the servo10 position
@vehicle.parameters.on_attribute('SERVO10_FUNCTION')
def servo_callback(self, attr_name, value):
    print("CALLBACK: %s changed to: %s" % (attr_name, value))

# Changing servo positions
    # 0=Disabled
    # 134=SERVOn_MIN
    # 135=SERVOn_TRIM
    # 136=SERVOn_MAX
#func_list = [134,0,135,0,136,0]

# for i in func_list:
#     vehicle.parameters['SERVO10_FUNCTION'] = i
#     time.sleep(1)

for i in range (0,20):
    print("Waiting...")
    time.sleep(1)

vehicle.close()
print("Completed")