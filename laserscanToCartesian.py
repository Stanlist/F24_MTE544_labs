import numpy as np
from utilities import FileReader
import matplotlib.pyplot as plt

def polar_to_cartesian(ranges, azimuths):
    x = ranges * np.cos(azimuths)
    y = ranges * np.sin(azimuths)
    return x, y

# Example usage

azimuth = 0.0 # Total azimuth angle assuming first range is at angle 0
azimuth_incr = 0.008714509196579456

headers, values=FileReader("laser_content_circle.csv").read_file()

basic_ranges = values[0][0]

basic_azimuths = []
for i in range(len(basic_ranges)):
    basic_azimuths.append(azimuth)
    azimuth += azimuth_incr

trimmed_ranges = []
trimmed_azimuths = []
for i in range(len(basic_ranges)):
    if not basic_ranges[i] == "inf":
        trimmed_ranges.append(basic_ranges[i])
        trimmed_azimuths.append(basic_azimuths[i])

ranges = np.array(trimmed_ranges)  # Replace with your LiDAR range data
azimuths = np.array(trimmed_azimuths)  # Replace with your azimuth angles
x, y = polar_to_cartesian(ranges, azimuths)
plt.scatter(x, y, marker='o', label='Laser Scan Points')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title("First Range Set of Laser Scan for Circle Movement")
plt.legend()
plt.show()
