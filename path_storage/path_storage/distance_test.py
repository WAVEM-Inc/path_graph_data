import math
import pyproj
from math import radians, sin, cos, sqrt, atan2


def haversine_distance(lat1, lon1, lat2, lon2):
    # 지구 반지름 (km 단위)
    R = 6371.0

    # 경위도를 라디안 단위로 변환
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    lat2 = radians(lat2)
    lon2 = radians(lon2)

    # Haversine 공식
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    # 두 지점 사이의 거리 계산
    distance = R * c

    return distance


def convert_latlon_to_utm(latitude, longitude):
    # 경위도 좌표계를 UTM 좌표계로 변환하는 객체 생성
    utm_converter = pyproj.Proj(proj="utm", zone=52, ellps="WGS84")

    # 경위도를 UTM 좌표로 변환
    utm_x, utm_y = utm_converter(longitude, latitude)

    return utm_x, utm_y


# 두 점 사이의 거리를 계산하는 함수
def distanceBetween(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# 두 좌표를 연결한 직선과 외부 점 사이의 거리를 계산하는 함수
def distance_from_line(x1, y1, x2, y2, x0, y0):

    # 직선의 방정식 이용
    numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denominator = math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
    # 거리 계산
    distance = numerator / denominator

    return distance


# 두 점을 연결하는 직선에 수직이고 시작 좌표를 지나는 직선과 외부 점 사이의 거리를 계산하는 함수
def distance_from_perpendicular_line(x1, y1, x2, y2, x0, y0):
    # (y2 - y1) 또는 (x2 - x1) 가 0 인 경우는 별도 계산 해야함

    # 시작 점을 지나는 직선의 기울기는
    perpendicular_m = (x2 - x1) / (y2 - y1) * -1
    # 시작 점을 지나는 직선의 방정식은 y - y1 = perpendicular_m * (x - x1)
    # 이 직선과 외부 점 사이의 거리를 계산
    distance = abs(
        (perpendicular_m * x0 - y0 + y1 - perpendicular_m * x1)
        / math.sqrt(1 + perpendicular_m * perpendicular_m)
    )

    return distance


def main():

    # a = math.sqrt(2)

    # b = (a / 2.0) * 3
    # print(str(b))

    # x1, y1 = map(float, input("Enter  first point (x1 y1): ").split())
    # x2, y2 = map(float, input("Enter  second point (x2 y2): ").split())
    # x0, y0 = map(float, input("Enter external point (x0 y0): ").split())

    # dist = distance_from_perpendicular_line(x1, y1, x2, y2, x0, y0)
    # # dist = distance_from_line(x1, y1, x2, y2, x0, y0)
    # print("Distance :", dist)

    lat1 = float(input("Enter latitude (decimal degrees): "))
    lon1 = float(input("Enter longitude (decimal degrees): "))
    utm_x1, utm_y1 = convert_latlon_to_utm(lat1, lon1)

    lat2 = float(input("Enter latitude (decimal degrees): "))
    lon2 = float(input("Enter longitude (decimal degrees): "))
    utm_x2, utm_y2 = convert_latlon_to_utm(lat2, lon2)

    distance = distanceBetween(utm_x1, utm_y1, utm_x2, utm_y2)

    print("distance utm : ", distance)

    # 두 지점 사이의 거리 계산
    distance = haversine_distance(lat1, lon1, lat2, lon2)
    print("Distance harversine :", distance * 1000, "m")


if __name__ == "__main__":
    main()
