# start.py
# Automatizes pre-flight checks

#NOTAS
# Hay un mensaje llamado MAV_CMD_PREFLIGHT_CALIBRATION, mirar de que va
# tambien el SYS_STATUS, que parece que tiene de todo lo que me hace falta
# tambien el MAV_STATE, y el MAV_MODE


from dronekit import connect
from pymavlink import mavutil
import os

# pymavlink connection
master = mavutil.mavlink_connection('/dev/ttyAMA0', 921600)
print("Connected to mavlink")
master.wait_heartbeat()

# Open the log file for the raspberry
try:
    raspi_log = open("raspi_log.txt", "x")
except:
    raspi_log = open("raspi_log.txt", "a")

raspi_log.write("*** LOGFILE RASPBERRY PI 4 - DATETIME ***\n")

# Flags to raise in each check
flag_check1 = False
flag_pilot = False
flag_armed = False
flag_wifi_off = False
flag_servo_home = False

# Leer un valor (modo, por ejemplo), verificar que esta dentro del rango
print("Checking mode...")
#print(" Mode: %s" % vehicle.mode.name)    # settable
#curr_mode = vehicle.mode.name

curr_mode = "none"

if curr_mode == "STABILIZE":
    # Si todo está bien, mensaje de ok
    # y escribir en el fichero de logs la actividad hasta ahora
    print("Check 1: Mode [OK]")
    raspi_log.write("Check 1: Mode [OK]\n")
    flag_check1 = True
else:
    print("Check 1: Mode [FAIL]")
    raspi_log.write("Check 1: Mode [FAIL]\n")

# Pedir por teclado el ok del piloto
input("All checks passed, waiting for Pilot confirmation")

print("Pilot confirm [OK]")
raspi_log.write("Pilot confirm [OK]\n")
flag_pilot = True

# Armar el dron
print('Arming...')

try:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed [OK]')
    raspi_log.write("Armed [OK]\n")
    flag_armed = True
except:
        print('Armed [FAIL]')
        raspi_log.write('Armed [FAIL]\n')

# Apagar el Wi-Fi
print("Turning Wi-Fi off...")
cmd = "sudo ifconfig wlan0 down"

try:
    os.system(cmd)
    print("Wi-Fi down [OK]")
    raspi_log.write("Wi-Fi down [OK]\n")
    flag_wifi_off = True
except:
    print("Wi-Fi down [FAIL]")
    raspi_log.write("Wi-Fi down [FAIL]\n")

# Mover el servo a la posicion de reposo (listo para recibir orden) = MIN pwm value
print("Moving servo to home position...")

try:
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0, #confirmation
        10,    # servo number
        1100,          # servo position between 1000 and 2000
        0, 0, 0, 0, 0)    # param 3 ~ 7 not used

    # send command to vehicle
    vehicle.send_mavlink(msg)

    print("Servo at home [OK]")
    raspi_log.write("Servo at home [OK]\n")
    flag_servo_home = True
except:
    print("Servo at home [FAIL]")
    raspi_log.write("Servo at home [FAIL]\n")

# Anotar hora de comienzo en el log
if flag_check1 and flag_pilot and flag_armed and flag_wifi_off and flag_servo_home:
    print("Start datetime [dd/mm/yyyy hh:mm:ss]")
    raspi_log.write("Start datetime [dd/mm/yyyy hh:mm:ss]\n")
else:
    print("Start [FAIL]")
    raspi_log.write("Start [FAIL]\n")

# Comenzar temporizador
# (ver como se hace)

# Despegar y ejecutar siguiente script: flying_modes.py