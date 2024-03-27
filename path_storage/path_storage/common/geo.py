from numpy import sin, cos, arccos, pi, round
import math

# from haversine import haversine


def rad2deg(radians):
    degrees = radians * 180 / pi
    return degrees


def deg2rad(degrees):
    radians = degrees * pi / 180
    return radians


def getDistanceBetweenPoints(latitude1, longitude1, latitude2, longitude2, unit="m"):

    theta = longitude1 - longitude2

    distance = (
        60
        * 1.1515
        * rad2deg(
            arccos(
                (sin(deg2rad(latitude1)) * sin(deg2rad(latitude2)))
                + (
                    cos(deg2rad(latitude1))
                    * cos(deg2rad(latitude2))
                    * cos(deg2rad(theta))
                )
            )
        )
    )

    if math.isnan(distance):
        distance = 0.0

    if unit == "m":
        # print(
        #     "position : "
        #     + str(latitude1)
        #     + ","
        #     + str(longitude1)
        #     + " : "
        #     + str(latitude2)
        #     + ","
        #     + str(longitude2)
        #     + " / distance : "
        #     + str(round(distance * 1000 * 1.609344, 2))
        # )
        return round(distance * 1000 * 1.609344, 2)
    elif unit == "km":
        return round(distance * 1.609344, 2)

    print("unkwon distance unit : " + unit)
    return 0
