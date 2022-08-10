from dronekit import connect
from pymavlink import mavutil


# Connect to the Vehicle
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=921600)
print("ok connected")

msg = vehicle.message_factory.command_long_encode(
    0, 0,    # target_system, target_component
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
    0, #confirmation
    10,    # servo number
    1900,          # servo position between 1000 and 2000
    0, 0, 0, 0, 0)    # param 3 ~ 7 not used

# send command to vehicle
vehicle.send_mavlink(msg)