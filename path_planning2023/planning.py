import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from matplotlib.cm import viridis
from matplotlib.colors import Normalize

# 1. Conversion from Geographic to Cartesian
def geo_to_cartesian(lon, lat, alt):
    R = 6371000  # Earth radius in meters
    lon, lat = np.radians(lon), np.radians(lat)
    x = (R + alt) * np.cos(lat) * np.cos(lon)
    y = (R + alt) * np.cos(lat) * np.sin(lon)
    z = alt
    return x, y, z

# Compute Curvature
def compute_curvature(x, y):
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**(3/2)
    return curvature

# Load data
data = pd.read_csv('sem_2023_us.csv')

# Convert to Cartesian
cartesian_points_data = [geo_to_cartesian(lon, lat, alt) for lon, lat, alt in zip(data['Longitude'], data['Latitude'], data['Metres above sea level'])]
x_data, y_data, z_data = zip(*cartesian_points_data)

# Path Interpolation
t_data = np.linspace(0, 1, len(x_data))
fx_data = interp1d(t_data, x_data, kind='cubic')
fy_data = interp1d(t_data, y_data, kind='cubic')
t_new_data = np.linspace(0, 1, 10*len(x_data))
x_new_data = fx_data(t_new_data)
y_new_data = fy_data(t_new_data)

# Speed Profile
conversion_factor = 3.281  # m/s to mph
max_speed_mps = 20 / conversion_factor
max_speed_mph = max_speed_mps * conversion_factor
curvatures_data = compute_curvature(x_new_data, y_new_data)
curvature_threshold = np.percentile(np.abs(curvatures_data), 95)  # Threshold based on 90th percentile of curvature
dynamic_factor = (1 - np.clip(np.abs(curvatures_data) / curvature_threshold, 0, 1))  # Linear reduction based on curvature
min_speed_mph = 12  # For example, set minimum speed to 12 mph
dynamic_speed_profile_mph = min_speed_mph + dynamic_factor * (max_speed_mph - min_speed_mph)

# Compute the moving average for the dynamic speed profile
window_size = int(0.05 * len(dynamic_speed_profile_mph))  # 5% of the data length
smoothed_speed_profile_mph = np.convolve(dynamic_speed_profile_mph, np.ones(window_size)/window_size, mode='valid')

# Adjust the data for visualization
adjusted_x_new_data = x_new_data[int(window_size/2):-int(window_size/2)]
adjusted_x_new_data = adjusted_x_new_data[:len(smoothed_speed_profile_mph)]
adjusted_y_new_data = y_new_data[int(window_size/2):-int(window_size/2)]
adjusted_y_new_data = adjusted_y_new_data[:len(smoothed_speed_profile_mph)]
adjusted_t_new_data = t_new_data[int(window_size/2):-int(window_size/2)]
adjusted_t_new_data = adjusted_t_new_data[:len(smoothed_speed_profile_mph)]

# Define normalization for the color map based on hardcoded speed range
norm = Normalize(vmin=16, vmax=18)

# Trim the smoothed_speed_profile_mph to match the adjusted x and y data lengths
smoothed_speed_profile_mph = smoothed_speed_profile_mph[:len(adjusted_x_new_data)]

# Visualization with Smoothed Speed Profile (Corrected Again)
fig, axs = plt.subplots(2, 1, figsize=(12, 12))

# Race Track Path
axs[0].plot(adjusted_x_new_data, adjusted_y_new_data, label="Interpolated Path", color='lightgray', zorder=0)
axs[0].scatter(x_data, y_data, color='red', marker='o', s=5, label="Original Points", zorder=1)
sc = axs[0].scatter(adjusted_x_new_data, adjusted_y_new_data, c=smoothed_speed_profile_mph, cmap="viridis", s=40, norm=norm, label="Optimal Speed", zorder=2)
cbar = fig.colorbar(sc, ax=axs[0], orientation="vertical")
cbar.set_label("Speed (mph)", rotation=270, labelpad=20)
axs[0].legend()
axs[0].set_title("Race Track Path with Smoothed Speed Profile and Hardcoded Color Range")
axs[0].set_xlabel("X (meters)")
axs[0].set_ylabel("Y (meters)")
axs[0].axis('equal')

# Speed Profile
axs[1].plot(adjusted_t_new_data, smoothed_speed_profile_mph, label="Smoothed Speed Profile", color="blue")
axs[1].legend()
axs[1].set_title("Smoothed Speed Profile along Path")
axs[1].set_xlabel("Normalized Path Parameter")
axs[1].set_ylabel("Speed (mph)")

plt.tight_layout()
plt.show()
