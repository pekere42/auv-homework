import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from haversine import haversine, inverse_haversine, Unit
from scipy.optimize import least_squares
import folium


def error_function(pinger, points, distances):
    lat_p, lon_p, depth_p = pinger

    errors = []
    for (lat, lon, depth), distance in zip(points, distances):
        # calcualte the distance between the pinger and the point using haversine
        surface_distance = haversine((lat, lon), (lat_p, lon_p), unit=Unit.METERS)

        # calculate the total distance between the pinger and the point using pythagorean
        total_distance = np.sqrt(surface_distance**2 + (depth - depth_p) ** 2)

        errors.append(total_distance - distance)

    return errors


df = pd.read_csv("data.csv")
# fix the distance data
df["dist"] = df["dist"] * 2

points = df[["lat", "lon", "depth"]].to_numpy()
distances = df["dist"].to_numpy()
print(points)

# use least squares to find the pinger location
initial_guess = np.array([0.0, 0.0, 0.0])
result = least_squares(error_function, initial_guess, args=(points, distances))

lat_p, lon_p, depth_p = result.x
print(result.x)

fig, ax = plt.subplots()
ax.set_aspect("equal")


# Plot initial measurement points and their circles
for _, row in df.iterrows():
    lat, lon, dist = row["lat"], row["lon"], row["dist"]
    ax.plot(lat, lon, "bo")  # Measurement point

    # Generate a circle around the measurement point using inverse_haversine
    for i in range(0, 360, 10):
        new_point = inverse_haversine((lat, lon), dist, i, unit=Unit.METERS)
        ax.plot(new_point[0], new_point[1], "ro", markersize=2)

ax.plot(lat_p, lon_p, "go", label="Estimated Pinger Location")

m = folium.Map([42.4369032878788, 18.587494378787877], zoom_start=12)

# draw the sensors and their circles on the map
for _, row in df.iterrows():
    lat, lon, dist = row["lat"], row["lon"], row["dist"]
    folium.Marker(
        location=[lat, lon],
        icon=folium.Icon(color="blue"),
    ).add_to(m)

    folium.Circle(
        location=[lat, lon],
        radius=dist,
        color="blue",
        fill=False,
    ).add_to(m)

# draw the pinger on the map
folium.Marker(
    location=[lat_p, lon_p],
    icon=folium.Icon(color="green"),
).add_to(m)

m.save(outfile="map.html")

plt.xlabel("latitude")
plt.ylabel("longitude")
plt.grid(True)
plt.legend()
plt.show()
