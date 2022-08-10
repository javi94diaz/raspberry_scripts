from pymavlink import mavutil

# Connection with the autopilot
master = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# Arm
master.arducopter_arm()
""" master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0) """

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Confirmation
if master.motors_armed():
    print("Confirmed")