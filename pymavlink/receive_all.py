from pymavlink import mavutil

# Connection with vehicle
vehicle = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)
vehicle.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))

# Receive all information from vehicle
while True:
    msg = vehicle.recv_match(blocking=True)
    print(msg)