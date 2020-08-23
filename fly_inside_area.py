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
    log.to_csv("{}_{}.csv".format(connection_to, flight_UUID))
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

area_size_m = 50
max_height_m = 35 # Drone will also take off to this altitude
min_height_m = 15

direction_deg = 10
direction_m = 500

speed_ms = 20



connection_to = "sitl"
#connection_to = "solo"

### SETTINGS END ###

### SET UP CONNECTION ###
if connection_to == "solo":
    connection_string = "udpin:0.0.0.0:14550"

else:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to {} vehicle on: {}'.format(connection_to, connection_string))
vehicle = connect(connection_string, wait_ready=True)

#Add observer for the vehicle's telemetry
vehicle.add_attribute_listener('*', telemetry_callback)

### SET UP CONNECTION END ###

### FLIGHT PROCESS ###

#Arm the vehicle and fly to target altitude (in meters)
arm_and_takeoff(max_height_m, vehicle)

# Get the home location
home_location = vehicle.location.global_relative_frame

# Get the start location, around what to start the flight process
d_north = math.cos(math.radians(direction_deg)) * direction_m
d_east  = math.sin(math.radians(direction_deg)) * direction_m
start_location = get_location_metres(home_location, d_north, d_east)

# Hover around initial position
while vehicle.mode.name == "GUIDED": #Stop action if we are no longer in guided mode.

    # Get random delta in meters from starting point within area
    positive_half_area_size = int(area_size_m / 2)
    negative_half_area_size = positive_half_area_size * -1

    d_north = random.randrange(negative_half_area_size, positive_half_area_size)
    d_east = random.randrange(negative_half_area_size, positive_half_area_size)

    # Get new latitude and longitude from deltas
    next_target = get_location_metres(start_location, d_north, d_east)

    # Set random altitude
    next_target.alt = random.randrange(min_height_m, max_height_m)

    distance_from_home = get_distance_metres(start_location, next_target)
    print("Going to distance from start [m]: {:.2f}".format(distance_from_home))

    goto(next_target, vehicle, speed=speed_ms)

print("INFO - Exiting, drone mode changed from GUIDED -> {}".format(vehicle.mode.name))
end_flight()