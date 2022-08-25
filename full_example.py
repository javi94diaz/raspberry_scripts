##### START #####

# Execute webapp.py - DONE
# Open the web page in "localhost:5000" - DONE
# Upload a new script with actions: do_stuff.py - DONE
# Push button "Preflight checks"
    # Checks mode - DONE
    # Checks GPS signal - DONE
    # Checks battery level - DONE
    # Waits to human confirmation??? (manual checks such as setting the mission, connecting batteries, etc)
# Human pushes button "Pilot OK" or button "Arm and Wi-Fi OFF"
    # Arm the drone - DONE
    # Wi-Fi OFF - DONE
    # Write everything to the log file - DONE
    # Start executing the uploaded script: do_stuff.py - DONE
    # Start executing the script record_data.py 

# Take off - 
#   mensaje MAV_CMD_NAV_TAKEOFF y MAV_CMD_NAV_VTOL_TAKEOFF

# The script do_stuff.py is running, performing the actions programmed - DONE
# The script record_data.py is collecting information to show after landing

# Land
#   mensaje MAV_CMD_NAV_VTOL_LAND

# Disarm the drone - DONE
# Wi-Fi ON - DONE
# Execute webapp.py - DONE
# Open the web page in "localhost:5000" - DONE
# Script record_data.py has saved the data in files (battery level, voltage, altitude and others)
# Script show_data.py reads that files and prints graphics on the web page
# Button "Download Log files" gets the log files from the Autopilot

##### END #####