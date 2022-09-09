# Flask server for the website
from flask import Flask, request, render_template, jsonify, send_from_directory
from flask_cors import CORS
from javikit import *

app = Flask (__name__)
cors = CORS(app)
all_checks = False


# Main page
@app.route('/', methods= ['GET', 'POST'])
def get_message():
    print("Got request in main function")
    return render_template("index.html")


# 'Pre-flight checks' button response
@app.route('/pre_flight_checks')
def pre_flight_checks():
    '''
    Voltage battery in good range (depending of 2S, 3S, 4S, etc)
    Airspeed = 0 && groundspeed = 0
    Altitude < 10m
    GPS signal - satellites >= 3
    Change mode to AUTO
    '''

    raspi.output("[INFO] Executing pre-flight checks")

    ## Read information from autopilot
    # Battery
    [voltage_battery] = drone.read_message("SYS_STATUS", "voltage_battery")
    raspi.output("[PRE_FLIGHT] voltage_battery: {}".format(voltage_battery))

    # GPS signal
    [satellites_visible] = drone.read_message("GPS_RAW_INT", "satellites_visible")
    raspi.output("[PRE_FLIGHT] satellites_visible: {}".format(satellites_visible))

    # Airspeed, altitude, groundspeed
    [airspeed, alt, groundspeed] = drone.read_message("VFR_HUD", "airspeed", "alt", "groundspeed")
    raspi.output("[PRE_FLIGHT] airspeed: {}".format(airspeed))
    raspi.output("[PRE_FLIGHT] alt: {}".format(alt))
    raspi.output("[PRE_FLIGHT] groundspeed: {}".format(groundspeed))


    # LiPo battery (3S)
        # Minimum voltage = 9V
        # Maximum voltage = 12.6V
        # Nominal voltage = 11.1V

    min_voltage = 9000 # millivolts
    nom_voltage = 11100 # millivolts
    max_voltage = 12600 # millivolts
    alt_limit = 10 #meters
    min_satellites = 3

    ## Determine if the checks are correct
    all_checks = False
    check1 = False
    check2 = False
    check3 = False
    check4 = False
    check5 = False

    if voltage_battery >= nom_voltage and voltage_battery <= max_voltage:
        raspi.output("[OK] Preflight voltage_battery correct")
        check1 = True
    else:
        raspi.output("[ERROR] Preflight voltage_battery incorrect")

    if airspeed == 0:
        raspi.output("[OK] Preflight airspeed correct")
        check2 = True
    else:
        raspi.output("[ERROR] Preflight airspeed incorrect")

    if groundspeed == 0:
        raspi.output("[OK] Preflight groundspeed correct")
        check3 = True
    else:
        raspi.output("[ERROR] Preflight groundspeed incorrect")
        
    if alt <= alt_limit: 
        raspi.output("[OK] Preflight altitude correct")
        check4 = True
    else:
        raspi.output("[ERROR] Preflight altitude incorrect")
        
    if satellites_visible >= min_satellites:
        raspi.output("[OK] Preflight satellites_visible correct")
        check5 = True
    else:
        raspi.output("[ERROR] Preflight satellites_visible incorrect")

    if check1 and check2 and check3 and check4 and check5:
        raspi.output("[OK] All pre-flight checks correct")
        all_checks = True
        # Changing the mode before flying
        raspi.output("Current mode: " + drone.get_mode())
        drone.set_mode('AUTO')
    else:
        raspi.output("[ERROR] Some pre-flight check is incorrect")

    print("all_checks: {}".format(all_checks))

    return {'all_checks': all_checks}


# 'Arm and execute script' button response
@app.route('/arm')
def arm():

    print("[ARM] all_checks: {}".format(all_checks))

    if all_checks:
        
        # Arm the drone
        drone.arm()
        raspi.output('[OK] Armed succesfully')
        
        # Switch off the Wi-Fi interface
        raspi.set_wifi("down")
        countdown(2)
        
        # Execute the uploaded script
        from uploaded_scripts.catch_ashes import catch_ashes
        catch_ashes(drone, raspi)
        ret = True
    else:
        raspi.output('[ERROR] Not allowed to arm')
        ret = False

    return {'ret': ret}
    

# 'Disarm and Wi-Fi ON' button response
@app.route('/disarm')
def disarm():

    drone.disarm()
    raspi.output('[OK] Disarmed succesfully')
    raspi.set_wifi("up")
    countdown(6) # Wait for some seconds to connect to a network
    raspi.check_wifi()
    return "Nothing"


# 'Upload' button response
@app.route('/upload_static_file', methods=['POST'])
def upload_static_file():
    print("Got request in static files")
    print(request.files)
    f = request.files['static_file']
    f.save("uploaded_scripts/" + f.filename)
    resp = {"success": True, "response": "file saved!"}
    return jsonify(resp), 200

# 'Plot data' button response (sends the data.json file back to the client) 
@app.route('/request_data')
def patata():
    print("request_data")
    response = send_from_directory(directory='saved_data', filename='data.json')
    return response


if __name__ == '__main__':
    drone = Vehicle()
    raspi = CompanionComputer()
    
    connection_string = '/dev/ttyAMA0'
    connection_string2 = '/dev/ttyAMA1' 
    baud_rate = 57600
    baud_rate2 = 921600

    raspi.output("[INFO] Waiting to connect to autopilot...")
    drone.connect(connection_string, baud_rate2)
    raspi.output("[OK] Connected")
    drone.request_all_msgs(4)

    app.run(debug=True, host='0.0.0.0')
    