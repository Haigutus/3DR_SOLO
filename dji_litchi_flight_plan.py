# 2020-08-22 Kristjan - initial script to generate random flight plan for import to litchi app https://flylitchi.com/hub (used for dji drones)

import pandas
import random
import math
from uuid import uuid4

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

area_size_m = 50
max_height_m = 35
min_height_m = 15

nuber_of_points = 99 # Maxmimum number of points


# Set initial point and use names, used in litchi csv format
initial_point = {"latitude":     59.4330543607982,
                "longitude":    24.7639391004639,
                "altitude(m)":  30}

# Create list to keep all generated points
flight_plan = [initial_point]

while len(flight_plan) < nuber_of_points:

    # Get random delta in meters from starting point within area
    positive_half_area_size = int(area_size_m / 2)
    negative_half_area_size = positive_half_area_size * -1

    d_north  = random.randrange(negative_half_area_size, positive_half_area_size)
    d_east   = random.randrange(negative_half_area_size, positive_half_area_size)

    # Get random altitude
    altitude = random.randrange(min_height_m, max_height_m)

    # Get new latitude and longitude from deltas
    new_point = get_location_metres(initial_point, d_north, d_east)

    # Set new height
    new_point["altitude(m)"] = altitude

    # Add new waypoint to route
    flight_plan.append(new_point)

# Write flight plan to CSV, import at https://flylitchi.com/hub

file_name = "litch_flight_plan_{}.csv".format(uuid4())

column_order = ["latitude",	"longitude", "altitude(m)"]

flight_plan_data = pandas.DataFrame(flight_plan)[column_order]
flight_plan_data.to_csv(file_name, index=False)

print("Flight plan generated {}, import at https://flylitchi.com/hub".format(file_name))
