import pyproj

g = pyproj.Geod(ellps='clrk66')

p1 = ()

az12, az21, dist = g.inv( 77.0365, 38.8977, 77.0502, 38.8893)

print(dist)