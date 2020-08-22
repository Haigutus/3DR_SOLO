#from dronekit import connect, VehicleMode
from drone_functions import *
import random
import atexit
import datetime
import aniso8601
from uuid import uuid4
import pandas
import copy

@atexit.register
def end_flight():

    # Return to launch
    print("INFO - Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

    # Write log
    log = pandas.DataFrame(log_list)
    log.to_csv("{}.csv".format(flight_UUID))
    print("INFO - Flight log written for flight: {}".format(flight_UUID))

    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()

    # Shut down simulator if it was started.
    if connection_to != "solo":
        sitl.stop()

### INIATE LOGGING ###

#Callback to log telemetery
log_list = []
telemetry_dict = {}
telemetry_dict["timestamp"] = datetime.datetime.utcnow()
flight_UUID = str(uuid4())
telemetry_dict["flight_UUID"] = flight_UUID
update_interval = aniso8601.parse_duration("PT1S")

def telemetry_callback(self, key, value):

    telemetry_dict[key] = value

    # get current time
    time_now_utc = datetime.datetime.utcnow()

    # get previous timestamp
    previous_timestamp = telemetry_dict["timestamp"]

    # Only record whole set of data when interval has passed
    if time_now_utc - previous_timestamp >= update_interval:

        # set current timestamp
        telemetry_dict["timestamp"] = time_now_utc

        # Lets make a copy by value of dict
        copy_dict = {}
        copy_dict.update(telemetry_dict)
        log_list.append(copy_dict)

## Run the script in Solo or SITL ##
#Make connection_to="solo" to run the script locally from a host PC, communicating with Solo
#To communicate with Solo, connect host PC to controller's wifi network
#Else the script is run in SITL environment


### SETTINGS ###

area_diameter_m = 50 #
height_m = 15
speed_ms = 10

#connection_to = "SITL"
connection_to = "solo"

### SETTINGS END ###

### SET UP CONNECTION ###
if connection_to == "solo":
    print('Connecting to Solo on: udpin:0.0.0.0:14550')
    vehicle = connect('udpin:0.0.0.0:14550', wait_ready=True)
    #vehicle.wait_ready(True)

else:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
    print('Connecting to SITL vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)

### SET UP CONNECTION END ###


#Add observer for the vehicle's telemetry
vehicle.add_attribute_listener('*', telemetry_callback)

### FLIGHT PROCESS ###

#Arm the vehicle and fly to target altitude (in meters)
arm_and_takeoff(height_m, vehicle)

# Get initial position
start_location = vehicle.location.global_relative_frame

# Hover around initial position
while True:
    d_north = random.randrange(area_diameter_m / 2 * -1, area_diameter_m / 2)
    d_east = random.randrange(area_diameter_m/ 2 * -1, area_diameter_m / 2)
    next_target = get_location_metres(start_location, d_north, d_east)

    distance_from_home = get_distance_metres(start_location, next_target)
    print("Going to distance from home [m]: {:.2f}".format(distance_from_home))

    goto(next_target, vehicle, speed=speed_ms)


