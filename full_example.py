##### START
# Execute webapp.py
# Open the web page in "localhost:5000"
# Upload a new script with actions (using the state machine) - do_stuff.py
# Push button "Preflight checks"
    # (Uploads mission)
    # Checks mode
    # Checks GPS signal
    # Checks battery level
    # Waits to human confirmation
# Human pushes button "Pilot OK"
    # Arm the drone
    # Wi-Fi OFF
    # Write everything to the log file
    # Start executing the uploaded script: do_stuff.py
    # Start executing the script record_data.py

# Take off

# The script do_stuff.py is running, performing the actions programmed
# The script record_data.py is collecting information to show after landing

# Land
# Disarm the drone
# Wi-Fi ON
# Execute webapp.py
# Open the web page in "localhost:5000"
# Script record_data.py has saved the data in files (battery level, voltage, altitude and others)
# Script show_data.py reads that files and prints graphics on the web page
# Button "Download Log files" gets the log files from the Autopilot
##### END