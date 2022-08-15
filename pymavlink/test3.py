from pymavlink import mavutil
import time

vehicle = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)
vehicle.wait_heartbeat() # receiving heartbeat from the vehicle
print("Heartbeat from system (system %u component %u)" %(vehicle.target_system, vehicle.target_system))

while True:
   gcs_msg = vehicle.recv_match(type='SERVO_OUTPUT_RAW')
   if gcs_msg == None:
      pass
   else:
      print(type(gcs_msg))
   time.sleep(0.1)

   # try: 
   #  altitude = vehicle.messages['GPS_RAW_INT'].alt  # Note, you can access message fields as attributes!
   #  timestamp = vehicle.time_since('GPS_RAW_INT')
   # except:
   #    print('No GPS_RAW_INT message received')
      
   # if gcs_msg and gcs_msg.get_type() != 'BAD_DATA':
   #       vehicle.mav.send(gcs_msg)