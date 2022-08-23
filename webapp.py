# Flask server for the website
from flask import Flask, request, render_template, jsonify
from flask_cors import CORS
from pymavlink import mavutil
import os
from javikit import *

app = Flask (__name__)
#CORS(app)


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


@app.route('/pre_flight_checks')
def pre_flight_checks():
    
    print("pre flight checks button pressed")
    print("Current mode: " + drone.get_mode())
    drone.set_mode('STABILIZE')

    voltage_battery = drone.read_next_msg("SYS_STATUS", "voltage_battery")
    print("voltage_battery: ", end='')
    print(voltage_battery)

    return "Nothing"


@app.route('/arm')
def arm():
    drone.arm()
    raspi.output('[OK] Armed succesfully')
    raspi.set_wifi("down")
    countdown(2)
    #os.system("catch_ashes.py")
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
    