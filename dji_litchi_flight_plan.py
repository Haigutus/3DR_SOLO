# 2020-08-22 Kristjan - initial script to generate random flight plan for import to litchi app https://flylitchi.com/hub (used for dji drones)

import pandas
import random
import math
from uuid import uuid4

### FUNCTIONS ###

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
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location["latitude"] / 180))

    # Create new container for new point
    new_location = {}
    new_location.update(original_location)

    # New position in decimal degrees
    new_location["latitude"]  = original_location["latitude"]  + (dLat * 180 / math.pi)
    new_location["longitude"] = original_location["longitude"] + (dLon * 180 / math.pi)

    return new_location

def get_location_degrees_distance(original_location, degrees, distance):
    "Returns new location defined by angle in degrees and distance in meters"

    # Find delta North and East in meters
    d_north = math.cos(math.radians(degrees)) * distance
    d_east = math.sin(math.radians(degrees)) * distance

    # Return new location object
    return get_location_metres(original_location, d_north, d_east)

### SETTINGS ###

area_size_m = 50
max_height_m = 35
min_height_m = 15

direction_deg = 340
direction_m   = 100

flight_type = "random"
#flight_type = "box"

number_of_points = 99 # Maximum number of points is restricted to 99


# Set initial point and use names, used in litchi csv format
home_location = {"latitude":     59.4330543607982,
                 "longitude":    24.7639391004639,
                 "altitude(m)":  max_height_m}

# Get the start location, around what to start the flight process
start_location = get_location_degrees_distance(home_location, direction_deg, direction_m)

# Create list to keep all generated points
flight_plan = [home_location, start_location]

while len(flight_plan) < number_of_points:

    positive_half_area_size = int(area_size_m / 2)
    negative_half_area_size = positive_half_area_size * -1

    if flight_type == "random":
        # Get random delta in meters from starting point within area

        d_north  = random.randrange(negative_half_area_size, positive_half_area_size)
        d_east   = random.randrange(negative_half_area_size, positive_half_area_size)

        # Get random altitude
        altitude = random.randrange(min_height_m, max_height_m)

        # Get new latitude and longitude from deltas
        new_point = get_location_metres(start_location, d_north, d_east)

        # Set new height
        new_point["altitude(m)"] = altitude

        # Add new waypoint to route
        flight_plan.append(new_point)

    elif flight_type == "box":
        # right at max altitude
        right_hi_point = get_location_degrees_distance(start_location, direction_deg + 90, positive_half_area_size)

        # right at min altitude
        right_low_point = dict(right_hi_point)
        right_low_point["altitude(m)"] = min_height_m

        # left at max altitude
        left_hi_point = get_location_degrees_distance(start_location, direction_deg - 90, positive_half_area_size)

        # left at min altitude
        left_low_point = dict(left_hi_point)
        left_low_point["altitude(m)"] = min_height_m

        # Add generated point to route
        flight_plan.extend([right_hi_point, right_low_point, left_low_point, left_hi_point])

    else:
        print("No flight type available with name: {}".format(flight_type))
        quit()


# Write flight plan to CSV, import at https://flylitchi.com/hub

file_name = "litch_{}_{}_{}_{}.csv".format(direction_m, direction_deg, flight_type, uuid4())

column_order = ["latitude",	"longitude", "altitude(m)"]

flight_plan_data = pandas.DataFrame(flight_plan[:99])[column_order] # Only 99 points allowed
flight_plan_data.to_csv(file_name, index=False)

print("Flight plan generated {}, import at https://flylitchi.com/hub".format(file_name))
