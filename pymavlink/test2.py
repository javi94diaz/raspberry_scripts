# Test file to read parameters

from pymavlink import mavutil
import json
'''
# Connection with the autopilot
master = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

print(master.params)
print(master.messages)
#master.flightmode = "AUTO"
print(master.flightmode)
print(master.vehicle_type)
print(master.recv()) # sale de clase bytes

'''

data = {
    "president": {
        "name": "Zaphod Beeblebrox",
        "species": "Betelgeusian"
    }
}

with open("data_file.json", "w") as write_file:
    json.dump(data, write_file)

    