# Flask server for the website
from flask import Flask, request, render_template, jsonify, send_from_directory
from flask_cors import CORS
from javikit import *

app = Flask (__name__)
cors = CORS(app)


# @app.route('/')
# def index():
#     return render_template('index.html')


@app.route('/', methods= ['GET', 'POST'])
def get_message():
    # if request.method == "GET":
    print("Got request in main function")
    return render_template("index.html")


@app.route('/upload_static_file', methods=['POST'])
def upload_static_file():
    print("Got request in static files")
    print(request.files)
    f = request.files['static_file']
    f.save("uploaded_scripts/" + f.filename)
    resp = {"success": True, "response": "file saved!"}
    return jsonify(resp), 200


@app.route('/request_data')
def patata():
    print("request_data")
    response = send_from_directory(directory='saved_data', filename='data.json')
    return response


@app.route('/pre_flight_checks')
def pre_flight_checks():

    ## From the checklist
    # battery level
    # connection with GCS
    # telemetry link
    # airspeed
    # servo check, arm servos (for surfaces), check controls 

    print("pre flight checks button pressed")
    print("Current mode: " + drone.get_mode())
    drone.set_mode('STABILIZE')

    [voltage_battery] = drone.read_message("SYS_STATUS", "voltage_battery")
    print("\nPREFLIGHT voltage_battery: {}".format(voltage_battery))

    print("airspeed, climb, alt, groundspeed:")
    print( drone.read_message("VFR_HUD", "airspeed", "climb", "alt", "groundspeed") )

    #print( drone.read_message("HEARTBEAT", "mode", "is_enabled") )

    return "Nothing"


@app.route('/arm')
def arm():
    drone.arm()
    raspi.output('[OK] Armed succesfully')
    raspi.set_wifi("down")
    countdown(2)
    from uploaded_scripts.catch_ashes import catch_ashes
    catch_ashes()
    return "Nothing"
    


@app.route('/disarm')
def disarm():

    drone.disarm()
    raspi.output('[OK] Disarmed succesfully')
    raspi.set_wifi("up")
    countdown(6)
    raspi.check_wifi()
    return "Nothing"



if __name__ == '__main__':
    drone = Vehicle()
    raspi = CompanionComputer()
    drone.connect('/dev/ttyAMA0', 921600)
    raspi.output("[OK] Connected")
    drone.request_all_msgs(4)
    app.run(debug=True, host='0.0.0.0')
    