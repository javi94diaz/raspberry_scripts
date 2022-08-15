from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# Disarm
master.arducopter_disarm()
""" master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
 """
# wait until disarming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to disarm")
master.motors_disarmed_wait()
print('Disarmed!')

# Confirmation
if not master.motors_armed():
    print("Confirmed")