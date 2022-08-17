''' ******************************************************************
    JAVIKIT v1.0
    Python API for easy use of pymavlink methods from a companion computer
****************************************************************** '''

import os, sys, time, datetime, re, pprint
import contextlib
from pymavlink import mavutil
from urllib.request import urlopen, URLError


# Chequiar la clase "mavmmaplog" que es un log file, para mi parte de post-flight
# Cambiar las descripciones y los ejemplos de uso de los metodos CUANDO LOS ACABE DE REFACTORIZAR
# Buscar javascript drag and drop box for a file, to upload it in the web to the raspi
# Hacer un metodo que imprima tanto por consola como en el logfile, que sino es un coñazo poner el print y el logfile.write todo el rato

# Para vuelos de long range, la GCS manda comandos al cubo, y desde la raspi solicitamos todos
# los mensajes y vamos revisando los que yo diga (x mensajes significan esto) que me pueden mandar desde la GCS o desde un 
# switch del mando para ejecutar una u otra funcion/script en la raspi

# Actions to do with the aircraft
# tomar fotos
# desplegar/replegar ash cathcer
# leer todos los mensajes y hacer un analisis del estado de la nave: temp motores, bateria etc, e inferir un estado de la maquina de estados de toda esa info
# y una vez sepamos el estado de la state machine, hacer una cosa u otra en respuesta: volver a casa, recalcular waypoints y reprogramar la mision, etc
# definir dos o tres estados basicos, con una respuesta cada uno, y listo
# bateria baja -> respuesta volver a casa, modo RTL
# temperatura alta en motor -> reprogramar un waypoint
# estado normal -> no hacer nada
# y asi como future work, seria expandir esta maquina de estados para hacerla mas compleja y darle mas inteligencia a la nave

def countdown(seconds):
    '''
    Show a countdown of some seconds onscreen
    Example
        countdown(3)

    Output
        Count 1
        Count 2
        Count 3

    '''
    print("Counting {} seconds".format(seconds))
    for i in range (1, seconds+1):
        print("Count " + str(i))
        time.sleep(1)


def get_datetime():
    '''
    Returns an string with current date and time
    using dd/mm/yy hh:mm:ss format
    Example:
        print(get_datetime())
    
    Output
        08/17/22 09:47:16

    '''
    x = datetime.datetime.now()
    return x.strftime("%d/%m/%y %X") # don't know why some of them are orange
  

class Vehicle:
    '''
    Represents a connection with the autopilot via MAVlink.
    Provides methods to receive and send information to the autopilot such as the messages, flightmode, servo position or arming and disarming the aircraft
    '''

    modelist = [
        'STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 
        'GUIDED', 'LOITER', 'RTL', 'CIRCLE', 
        'POSITION', 'LAND', 'OF_LOITER', 'DRIFT', 
        'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 
        'BRAKE', 'THROW', 'AVOID_ADSB', 'GUIDED_NOGPS', 
        'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG', 
        'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL'
        ]
    

    def __init__(self):
        self.mode = "default"
        self.master = None


    def connect(self, connection_string='/dev/ttyAMA0', baudrate=921600):
        '''
        Connect to the autopilot
        Returns a mavutil connection object to handle all methods in the vehicle
        Example:
            drone = Vehicle()
            drone.connect('/dev/ttyAMA0', 921600)
        
        Output:
            Waiting to connect...
            [OK] Connected
        '''
        try:
            self.master = mavutil.mavlink_connection(connection_string, baudrate)
            print("Waiting to connect...")
            self.master.wait_heartbeat()
            print("[OK] Connected")
        except:
            print("[ERROR] Connection not established")

    # Send heartbeat from a MAVLink application (from the script running on Raspberry Pi)
    def receive_heartbeat(self):
        '''
        Autopilot receives the heartbeat from the python script to prove the companion computer is alive
        Example:
            drone = Vehicle()
            drone.receive_heartbeat()

        Output:
            (none)
        '''
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 
            0, 0, 0)


    def get_mode(self):
        '''
        Read current flight mode
        There are two ways, the attribute "flightmode", or using the 
        heartbeat message to infer the current mode.
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.get_mode()

        Output:
            Current mode: 'STABILIZE'
        '''

        mode_num = 0
        for i in range(0, 2):
            try:
                msg = self.master.recv_match(type="HEARTBEAT", blocking=True).to_dict()
                if msg['custom_mode'] > mode_num:
                    mode_num = msg['custom_mode']
            except Exception as e:
                print(e)
            time.sleep(0.1)

        self.mode =  self.modelist[mode_num]
        return self.mode


    def set_mode(self, mode):
        '''
        Set a new mode to the aircraft
        First, it checks the current mode and then tries to change it
        The 'mode' parameter is not case sensitive
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            set_mode('LOITER')
        
        Output:
            Current mode: 'STABILIZE'
            Mode set to: 'LOITER'
        '''

        print("Current mode: " + self.get_mode())

        mode = mode.upper()
        
        # Check if given mode is valid
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit(1)

        # Get mode ID
        mode_id = self.master.mode_mapping()[mode]

        # Set new mode
        self.master.mav.command_long_send(
        self.master.target_system, self.master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
        0,
        1, 
        mode_id, 0, 0, 0, 0, 0)

        while True:
            # Wait for acknowledgement command
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()

            # Continue waiting if the acknowledged command is not `set_mode`
            if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                continue

            # Print the ACK result
            print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
            break

        print("Mode set to: " + self.get_mode())

        
    def arm(self):
        '''
        Arms the aircraft using the mavutil connection object
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.arm()

        Output:
            Waiting for the vehicle to arm...
            [OK] Armed succesfully

        '''
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0)

            print("Waiting for the vehicle to arm...")
            self.master.motors_armed_wait()

            if self.master.motors_armed():
                print('[OK] Armed succesfully')
        except:
            print('[ERROR] Arming failed')


    def disarm(self):
        '''
        Disarms the aircraft using the mavutil connection object
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.disarm()

        Output:
            Waiting for the vehicle to disarm...
            [OK] Disarmed succesfully
        '''
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0)

            print("Waiting for the vehicle to disarm...")
            self.master.motors_disarmed_wait()

            if not self.master.motors_armed():
                print('[OK] Disarmed succesfully')
        except:
            print('[ERROR] Disarming failed')


    def move_servo(self, servo_n=10, microsec=1100):
        '''
        Moves the desired servomotor servo_n to a position determined by the microseconds given for the PWM.
        The default MIN position is 1100 microseconds
        The default TRIM position is 1500 microseconds
        The default MAX position is 1900 microseconds
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.move_servo(10, 1100) # Moves servo number 10 to MIN position (1100 microseconds)
        
        Output:
            (the servo moves to desired position)
        '''

        try:

            # self.master.mav.commsand_long_send(
            #     self.master.target_system, 
            #     self.master.target_component,
            #     mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            #     0,            # first transmission of this command
            #     servo_n,  # servo instance
            #     microseconds, # PWM pulse-width
            #     0,0,0,0,0     # unused parameters
            # )

            self.master.set_servo(servo_n, microsec)
            
            print("[OK] Servo "  + str(servo_n) + " moved to " + str(microsec))
        except:
            print("[ERROR] Servo position not set")


    # Methods to handle different types of messages
    def handle_heartbeat(msg):
        mode = mavutil.mode_string_v10(msg)
        is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

    def handle_rc_channels(msg):
        channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, 
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)#,
                #msg.chan9_raw, msg.chan10_raw, msg.chan11_raw, msg.chan12_raw, 
                #msg.chan13_raw, msg.chan14_raw, msg.chan15_raw, msg.chan16_raw, 
                #msg.chan17_raw, msg.chan18_raw)
        
        the_keys = msg.to_dict().keys()
        #r = re.compile("^chan[0-2]*[0-9]_raw$") # regex for channels up to 18
        r = re.compile("^chan[1-8]{1}_raw$") # regex for channels 1-8
        new_keys = list(filter(r.match, the_keys))
        #print(new_keys)

        for i in new_keys:
            print("{}".format(i), end=' |')
            if i == list(new_keys)[-1]:
                print('')
        
        for item in channels:
            print("{}".format(item), end=9*' ' + '|')
            if channels.index(item) == -1:
                print('')

    def handle_hud(msg):
        hud_data = (msg.airspeed, msg.groundspeed, msg.heading, 
                    msg.throttle, msg.alt, msg.climb)
        print("Aspd\tGspd\tHead\tThro\tAlt\tClimb")
        print("%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data)

    def handle_attitude(msg):
        attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, 
                    msg.pitchspeed, msg.yawspeed)
        print("Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd")
        print("%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data)

    def handle_sys_status(msg):
        sys_status_data = (msg.battery_remaining, msg.current_battery, msg.load, msg.voltage_battery)
        print ("read sys_status")

    def request_msg(self, freq=4):
        '''
        Request all types of messages with given frequency (4 by default)
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            freq = 8
            drone.request_msg(freq)
        
        Output:
            (none)
        '''

        # Poner la alternativa con command_long MAV_DO_REQUEST_MSG o similar

        self.master.mav.request_data_stream_send(
            self.master.target_system, 
            self.master.target_component, 
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 
            freq, 1)


    def read_messages(self):
        '''
        Read all messages from mavlink stream
        Uses an especific handler method for each type of message
        Example:
            drone = Vehicle()
            # ... connect to the vehicle
            drone.read_messages()

        Output
            (See 'handle_[type_of_message]()' methods for each type of message)
        '''

        # First, request all types of messages to the stream
        freq = 4
        self.request_msg(freq)

        while(True):

            msg = self.master.recv_match(blocking=False)
            if not msg:
                #return
                print("No message")
            else:
                #pprint.pprint(msg.to_dict())
                #print(msg.to_dict().keys())

                # handle the message based on its type
                msg_type = msg.get_type()
                print("Message " + msg_type)

                if msg_type == "BAD_DATA":
                    if mavutil.all_printable(msg.data):
                        sys.stdout.write(msg.data)
                        sys.stdout.flush()
                elif msg_type == "RC_CHANNELS": 
                    self.handle_rc_channels(msg)
                elif msg_type == "HEARTBEAT":
                    self.handle_heartbeat(msg)
                elif msg_type == "VFR_HUD":
                    self.handle_hud(msg)
                elif msg_type == "ATTITUDE":
                    self.handle_attitude(msg)
                elif msg_type == "SYS_STATUS":
                    self.handle_sys_status(msg)

                print("\n*****************")
            time.sleep(0.1)



class CompanionComputer:

    def __init__(self):
        self.log_file = self.open_logfile("log")


    def open_logfile(self, filename):
        ''' 
        Open a new log file to keep track of activity
        Example:
            my_computer = CompanionComputer()
            my_log = my_computer.open_logfile("my_log")
            my_log.write("This will be written in the log file\n")

        Output (in "mylog.txt"):
            *** LOGFILE: mylog.txt - 08/15/22 10:31:59 ***
            This will be written in the log file
        '''

        file_name = filename + ".txt"
        try:
            log_file = open(file_name, "x")
        except:
            log_file = open(file_name, "a")

        curr_time = datetime.datetime.now()
        log_file.write("\n*** LOGFILE: " + file_name + " - " + curr_time.strftime("%d/%m/%y %X") + " ***\n")
        return log_file


    def output(self, msg):
        '''
        Print a message on terminal and in a log file
        Example:
            my_computer = CompanionComputer()
            my_log = my_computer.open_logfile("my_log")
            my_computer.output("[OK] This will be printed and logged")

        Output:
            (in console)        [OK] This will be printed and logged
            (in 'my_log.txt')   [OK] This will be printed and logged
        '''
        print(msg)
        self.log_file.write(msg + "\n")


    def set_wifi(self, command):
        '''
        Turns ON/OFF the Wi-Fi network in a Linux companion computer
        The 'command' argument must be "up" or "down" to perform a valid Linux command
        Example:
            my_computer = CompanionComputer()
            my_computer.set_wifi("up")

        Output:
            [OK] Wi-Fi turned up
            '''

        if command == "up" or command == "down":
            cmd = "sudo ifconfig wlan0 " + command

            try:
                os.system(cmd)
                print("[OK] Wi-Fi turned " + command)
            except:
                print("[ERROR] Wi-Fi not set")
        else:
            print("Command not valid (must be 'up' or 'down')")


    def check_internet(self, url):
        '''
        Check if internet connection is available opening a given url
        Example:
            my_computer = CompanionComputer()
            my_computer.check_internet('http://216.58.192.142'):
            
        
        Output:
            [OK] Internet available (in both console and log file)
        '''
        try:
            with contextlib.closing(urlopen(url, timeout=10)) as x:
                self.output("[INFO] Internet available")
                return True
        except URLError as err:
            self.output("[INFO] Internet not available")
            return False


def main():
    print(get_datetime())
    drone = Vehicle()
    raspi = CompanionComputer()

    drone.connect('/dev/ttyAMA0', 921600)
    raspi.output("[OK] Connected")
    
    #drone.read_messages()

    print("Initial mode: " + drone.get_mode())
    newmode = input("Introduce the new mode: ")
    newmode.upper()

    drone.set_mode(newmode)

    pos = int(input("Introduce the servo position: "))
    drone.move_servo(10, pos)
    raspi.output("[OK] Servo position set")
    
    drone.arm()    
    raspi.set_wifi("down")
    countdown(1) # Turning off Wi-Fi is almost immediate

    raspi.check_internet('http://216.58.192.142')

    drone.disarm()
    raspi.set_wifi("up")
    countdown(6) # It takes about 6 seconds to reconnect to a Wi-Fi network
 
    # Different url to avoid opening the first one twice
    raspi.check_internet('https://bing.com')


if __name__ == '__main__':
	main()