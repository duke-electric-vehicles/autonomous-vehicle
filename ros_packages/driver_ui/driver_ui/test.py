import folium
from folium.plugins import MarkerCluster
import pandas as pd
chapel_coords = [36.0019, 78.9403]
my_map = folium.Map(location = chapel_coords, zoom_start = 13)
folium.Marker(chapel_coords, popup = 'chapel').add_to(my_map)


#Display the map
my_map