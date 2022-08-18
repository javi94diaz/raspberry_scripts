# Flask server for the website
from flask import Flask, render_template
from pymavlink import mavutil
import os
from javikit import *

app = Flask (__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/hello')
def hello():
    return "hello page"

@app.route('/arm')
def arm():
    drone.arm()
    raspi.set_wifi("down")
    countdown(2)
    raspi.check_wifi()

    return "Nothing"

#background process happening without any refreshing
@app.route('/disarm')
def disarm():

    drone.disarm()
    raspi.set_wifi("up")
    countdown(6)
    raspi.check_wifi()

    return "Nothing"


if __name__ == '__main__':
    drone = Vehicle()
    raspi = CompanionComputer()
    drone.connect('/dev/ttyAMA0', 921600)
    raspi.output("[OK] Connected")
    app.run(debug=True, host='0.0.0.0')
    