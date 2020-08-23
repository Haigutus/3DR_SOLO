#Import modules
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import dronekit_sitl
import time
import math
import copy
import pymavlink
from pymavlink import mavutil
from gmplot import gmplot
import datetime

#Functions

def angle_of_line(lat1, lon1, lat2, lon2):
# Determines the angle of line relative to vertical line from south to north
 
    d_lon = (lon1 - lon2)
    S = math.cos(math.radians(lat2)) * math.sin(math.radians(d_lon))
    C = (math.cos(math.radians(lat1))*math.sin(math.radians(lat2)))-(math.sin(math.radians(lat1))*math.cos(math.radians(lat2))*math.cos(math.radians(d_lon)))
    at = math.atan2(S,C)
    #deg = math.degrees(at)
    #angle_of_line = 180 - deg #Angle of line relative to vertical line from south to north
    angle_of_line = math.degrees(at)

    return angle_of_line


def arm_and_takeoff(aTargetAltitude, vehicle):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    new_lat = original_location.lat + (dLat * 180/math.pi)
    new_lon = original_location.lon + (dLon * 180/math.pi)

    if type(original_location) is LocationGlobal:
        target_location = LocationGlobal(new_lat, new_lon, original_location.alt)

    elif type(original_location) is LocationGlobalRelative:
        target_location = LocationGlobalRelative(new_lat, new_lon, original_location.alt)

    else:
        raise Exception("Invalid Location object passed")

    return target_location

def get_location_degrees_distance(original_location, degrees, distance):
    "Returns new location defined by angle in degrees and distance in meters"

    # Find delta North and East in meters
    d_north = math.cos(math.radians(degrees)) * distance
    d_east = math.sin(math.radians(degrees)) * distance

    # Return new location object
    return get_location_metres(original_location, d_north, d_east)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    d_lat = aLocation2.lat - aLocation1.lat
    d_lon = aLocation2.lon - aLocation1.lon
    d_alt = aLocation2.alt - aLocation1.alt

    distance_2D = math.sqrt((d_lat**2) + (d_lon**2)) * 1.113195e5
    distance_3D = math.sqrt(distance_2D**2 + d_alt**2)


    return distance_3D

def circular_mapping(start_point, end_point, step, radius, number_of_pictures, vehicle):
    #tulevikus v6iks m6elda, et step tuleks automaatselt arvutada kaameranurgast

    location_list=[]

    #Create copy of initial start point (i.e. target location)
    target_location = copy.deepcopy(start_point)

    location_list.append({
    'icon': 'http://maps.google.com/mapfiles/ms/icons/blue-dot.png',
    'lat': target_location.lat,
    'lng': target_location.lon,
    'infobox': "<b>Objekt</b>"
    })

    #Get current location
    current_location = vehicle.location.global_relative_frame

    location_list.append({
    'icon': 'http://maps.google.com/mapfiles/ms/icons/green-dot.png',
    'lat': current_location.lat,
    'lng': current_location.lon,
    'infobox': "<b>Alguspunkt</b>"
    })

    #Make sure drone stops some distance before the target location/object
    if current_location.lat <= target_location.lat:
        start_point = get_location_metres(target_location, -radius, 0)
    else:
        start_point = get_location_metres(target_location, radius, 0)

    print("1. start_point for gimbal is:")
    print(start_point)

    initial_angle = int(math.degrees(math.atan2(start_point.lon - target_location.lon ,start_point.lat - target_location.lat)))

    print ("Current angle: " + str(initial_angle))

    #Define the degree step at which pictures will be taken
    step_degree = 360/number_of_pictures

    #For loop for performing circular mapping at different altitudes
    for n, altitude_step in enumerate(range(target_location.alt, end_point.alt+step, step)):

        #Fly to the the target location/object
        start_point.alt = altitude_step
        target_location.alt = altitude_step

        print("Flying to target altitude")
        location_list.append({
        'icon': 'http://maps.google.com/mapfiles/ms/icons/red-dot.png',
        'lat': start_point.lat,
        'lng': start_point.lon,
        'infobox': "<b>Korguspunk nr {} seatud korgus {}</b>".format(n,altitude_step)
        })
        goto(start_point, vehicle)

        #Take picture - to be implemented

        #For for making a circle around the object
        for x, angle in enumerate(range(initial_angle,initial_angle+360,step_degree)):

            print ("Current angle: " + str(angle))

            start_point = get_location_metres(target_location, radius * math.cos(math.radians(angle)), radius * math.sin(math.radians(angle)))
            #startpoint.lat = radius * math.cos(math.radians(angle)) + startpoint.lat
            #startpoint.lon = radius * math.sin(math.radians(angle)) + startpoint.lon
            print("start_point for gimbal is:")
            location_list.append({
            'icon': 'http://maps.google.com/mapfiles/ms/icons/red-dot.png',
            'lat': start_point.lat,
            'lng': start_point.lon,
            'infobox': "<b>Punkt nr {}_{} seatud korgus</b> {} nurk {}".format(n,x,altitude_step, angle)
            })
            print(start_point)
            print("Flying to a position")
            goto(start_point, vehicle)

            #Point gimbal in the region of interest (ROI)
            vehicle.gimbal.target_location(target_location)

            #Take picture - to be implemented
            take_a_pic(vehicle)

        print("All positions completed for the altitude")
    print (location_list)

def goto(target_location, vehicle, speed=None):

    current_location = vehicle.location.global_relative_frame

    target_distance=get_distance_metres(current_location, target_location)
    print("Current: {}".format(current_location))
    print("Target:  {}".format(target_location))

    vehicle.simple_goto(target_location, airspeed=speed)

    list_of_distances = [target_distance]

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        time.sleep(1)

        current_location   = vehicle.location.global_relative_frame
        remaining_distance = get_distance_metres(current_location, target_location)

        print("Distance to target [m]: ", round(remaining_distance, 2))
        print("Air Speed        [m/s]: ", round(vehicle.airspeed, 2))
        print("Battery level      [%]: ", vehicle.battery.level, '\n')

        list_of_distances.append(remaining_distance)

        if abs(list_of_distances[-1] - list_of_distances[-2]) < 0.5:
            print ("Sending go to command")
            vehicle.simple_goto(target_location, airspeed=speed)
            

        if remaining_distance < 2:
        #if remainingDistance<=targetDistance*0.05 or : #Just below target, in case of undershoot.
            print("Reached target")
            #current_location = vehicle.location.global_relative_frame
            #print("Current: {}".format(current_location))
            #print("Target:  {}".format(target_location))
            break


def take_a_pic(vehicle):

    MAVLINK_GIMBAL_SYSTEM_ID = 1
    MAVLINK_GIMBAL_COMPONENT_ID = 154

    #Switch camera mode to photo
    msg = vehicle.message_factory.gopro_set_request_encode(MAVLINK_GIMBAL_SYSTEM_ID, MAVLINK_GIMBAL_COMPONENT_ID, mavutil.mavlink.GOPRO_COMMAND_CAPTURE_MODE, (1, 0, 0, 0))
    vehicle.send_mavlink(msg)
    vehicle.flush()

    time.sleep(1)

    #Take a picture
    msg = vehicle.message_factory.gopro_set_request_encode(MAVLINK_GIMBAL_SYSTEM_ID, MAVLINK_GIMBAL_COMPONENT_ID, mavutil.mavlink.GOPRO_COMMAND_SHUTTER, (1, 0, 0, 0))
    vehicle.send_mavlink(msg)
    vehicle.flush()


def pic_location_map(data):

    #Generate empty lists
    lats_A = []
    lons_A = []
    lats_B = []
    lons_B = []
    lats_tower = []
    lons_tower = []
    lats_offset = []
    lons_offset = []

    #Add data of interest to lists
    for i in data[1:-1]:

        check = i.get("offset_A")
        if check != None:
            lats_A.append(check.lat)
            lons_A.append(check.lon)

        check = i.get("offset_A2")
        if check != None:
            lats_A.append(check.lat)
            lons_A.append(check.lon)

        check = i.get("offset_B")
        if check != None:
            lats_B.append(check.lat)
            lons_B.append(check.lon)

        check = i.get("offset_B2")
        if check != None:
            lats_B.append(check.lat)
            lons_B.append(check.lon)

        check = i.get("lat")
        if check != None:
            lats_tower.append(check)

        check = i.get("lng")
        if check != None:
            lons_tower.append(check)


        check = i.get("offset_line")
        if check != None:
            lats_offset.append(check.lat)
            lons_offset.append(check.lon)

        check = i.get("offset_line2")
        if check != None:
            lats_offset.append(check.lat)
            lons_offset.append(check.lon)

    # Convert lists to tuples
    lats_tuple_A = tuple(lats_A)
    lons_tuple_A = tuple(lons_A)

    lats_tuple_B = tuple(lats_B)
    lons_tuple_B = tuple(lons_B)

    lats_tuple_tower = tuple(lats_tower)
    lons_tuple_tower = tuple(lons_tower)

    lats_tuple_offset = tuple(lats_offset)
    lons_tuple_offset = tuple(lons_offset)

    # Place map
    gmap = gmplot.GoogleMapPlotter(data[0]["lat"], data[0]["lng"], 16)

    # Place scatter data
    gmap.scatter(lats_tuple_A, lons_tuple_A, '#800080', size=10, marker=False)
    gmap.scatter(lats_tuple_B, lons_tuple_B, '#ff0000', size=10, marker=False)
    gmap.scatter(lats_tuple_tower, lons_tuple_tower, '#db9c24', size=7, marker=False)
    gmap.scatter(lats_tuple_offset, lons_tuple_offset, '#db9c24', size=10, marker=False)

    # Draw map
    gmap.draw("my_map.html")


def telemetry(vehicle, print_telemetry=False, post_url=None):
    """Input - DroneKit Vehicle.vehicle object, print_telemetry=False, post_url=None
       Output - print out all telemetry and/or sends it to a webservice """
    
    telemetry_dict = {
        "autopilot_firmware_version": vehicle.version,
        "autopilot_supports_ftp": vehicle.capabilities.ftp,
        #"global_location": vehicle.location.global_frame,
        "global_location_relative_altitude": vehicle.location.global_relative_frame,
        "local_location": vehicle.location.local_frame,  # distance form starting position
        #"location_latitude":vehicle.location.global_relative_frame.lat,
        #"location_longitude": vehicle.location.global_relative_frame.lon,
        #"location_altitude_relative": vehicle.location.global_relative_frame.alt,
        #"location_altitude_absolute": vehicle.location.global_frame.alt,
        "attitude": vehicle.attitude,
        #"velocity": vehicle.velocity,
        "GPS_fix": vehicle.gps_0.fix,
        "GPS_sattelites": vehicle.gps_0.num_sat,
        "groundspeed": vehicle.groundspeed,
        "airspeed": vehicle.airspeed,
        "gimbal status": vehicle.gimbal,
        "battery": vehicle.battery,
        "EKF": vehicle.ekf_ok,
        "last_heartbeat": vehicle.last_heartbeat,
        "rangefinder": vehicle.rangefinder,
        #"rangefinder_distance": vehicle.rangefinder.distance,
        #"rangefinder_voltage": vehicle.rangefinder.voltage,
        "heading": vehicle.heading,
        "armable": vehicle.is_armable,
        "system_status": vehicle.system_status.state,
        "mode": vehicle.mode.name,  # settable
        "armed": vehicle.armed,  # settable
        "timestamp": datetime.datetime.now().isoformat()
    }

    if print_telemetry == True:
        for key in telemetry_dict:
            print("{} -> {}".format(key, telemetry_dict[key]))

    if post_url:
        import requests
        response = requests.post(post_url, json=telemetry_dict)
        print("Telemetry sent with http code: {}".format(response.status_code))

    return telemetry_dict


def change_alt(vehicle, d_alt=0):
    current_location = vehicle.location.global_relative_frame

    target_location = current_location
    target_location.alt = current_location.alt + d_alt    

    goto(target_location, vehicle)

    return target_location

def change_location(vehicle, d_lat, d_lon):

    current_location = vehicle.location.global_relative_frame

    target_location = get_location_metres(current_location, d_lat, d_lon)

    goto(target_location, vehicle)

    return target_location











