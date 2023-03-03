import math
import pyproj

def ecef_to_lla(pos_tuple):

    x = pos_tuple[0]
    y = pos_tuple[1]
    z = pos_tuple[2]

    ecef = pyproj.Proj(proj = 'geocent', ellps = 'WGS84', datum = 'WGS84')
    lla = pyproj.Proj(proj = 'latlong', ellps = 'WGS84', datum = 'WGS84')
    lon, lat, alt = pyproj.transform(ecef, lla, x, y, z, radians=True)

    return (lat, lon, alt)