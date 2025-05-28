# %%
import os 
import polars as pl
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
import re 

def import_data(path) -> pl.DataFrame: 

    h = [
        "time_s",
        "time_ns",
        "frame",
        "marker1_name",
        "marker1_x",
        "marker1_y",
        "marker1_z",
        "marker1_visible",
        "marker2_name",
        "marker2_x",
        "marker2_y",
        "marker2_z",
        "marker2_visible",
        "marker3_name",
        "marker3_x",
        "marker3_y",
        "marker3_z",
        "marker3_visible",
        "rb_name", 
        "rb_id", 
        "rb_parentId", 
        "rb_x", 
        "rb_y", 
        "rb_z", 
        "rb_q1",
        "rb_q2",
        "rb_q3", 
        "rb_q4",
        "rb_trackingValid",
        "rb_meanError"
    ]

    data = pd.read_csv(path, names=h, index_col=False, dtype={"time_ns": str})
    data = pl.from_dataframe(data)
    return data

# %%
data_sources = {name: None for name in ["rosbag_front_extremes", "rosbag_left_extremes", "rosbag_right_extremes", "rosbag_net_extremes"]}
for source in data_sources.keys():
    data_sources[source] = import_data(f"data/{source}/data.csv") 


# %%
fig = plt.figure()
ax = fig.add_subplot(projection="3d")

for name, data in data_sources.items(): 
    data: pl.DataFrame = data.filter(pl.col("rb_trackingValid")).gather_every(15)

    if name == "rosbag_right_extremes":
        bad_data = data.filter(pl.col("rb_z") > 3.5)
        print(bad_data.sort("frame"))
        data = data.filter((pl.col("frame") <= 65000)).filter(pl.col("rb_y") < -3)
        print(data.filter((pl.col("rb_y") > -3.5) & (pl.col("rb_x") < 0)).sort("frame"))
    if name == "rosbag_front_extremes":
        data = data.filter(pl.col("rb_x") < -3.7)
    if name == "rosbag_left_extremes":
        data = data.filter(pl.col("rb_y") >= 4)
    if name == "rosbag_net_extremes":
        data = data.filter(pl.col("rb_x") >= 2.8)
    label_name = re.search("[a-z]+_([a-z]+)", name).group(1)
    label_name = label_name + " side"
    label_name = label_name.title()
    ax.scatter(data.select("rb_x").to_numpy(), data.select("rb_y").to_numpy(), data.select("rb_z").to_numpy(), label=label_name)


safe_vertices = np.array([
    [-4, -3.35],
    [2.3, -2.82],
    [2.6, 4],
    [-4.1, 3.89]
])

safe_height_range = [0, 3.6]

safe_volume_lower_corners = np.append(safe_vertices, safe_height_range[0]*np.zeros((4, 1)), 1)
safe_volume_upper_corners = np.append(safe_vertices, safe_height_range[1]*np.ones((4, 1)), 1)

safe_volume = np.concatenate([safe_volume_lower_corners, safe_volume_upper_corners], 0)

# Shrink each vertex 10% towards the origin
shrink_factor = 0.9
safe_volume = safe_volume * shrink_factor


ax.scatter(safe_volume[:, 0], safe_volume[:, 1], safe_volume[:, 2], marker="*", alpha=1, label="Safe Vertices")

# Define the vertices of the cube
vertices = safe_volume

# Define the faces of the cube using the indices of the vertices
faces = [
    [vertices[i] for i in [0, 1, 5, 4]],  # Bottom face
    [vertices[i] for i in [2, 3, 7, 6]],  # Top face
    [vertices[i] for i in [0, 1, 2, 3]],  # Front face
    [vertices[i] for i in [4, 5, 6, 7]],  # Back face
    [vertices[i] for i in [0, 3, 7, 4]],  # Left face
    [vertices[i] for i in [1, 2, 6, 5]],  # Right face
]

# Add the faces to the plot
ax.add_collection3d(Poly3DCollection(faces, alpha=0.3, edgecolor="k"))

# Compute the convex hull of the safe volume
hull = ConvexHull(safe_volume)

# Calculate the volume of the convex hull
safe_volume_value = hull.volume

# Print the computed volume
print(f"Safe volume: {safe_volume_value:.2f} cubic meters")


ax.legend()
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")

plt.tight_layout()
plt.savefig("plot.png")
plt.show()

# %% Export Safe Volume to a csv

df = pd.DataFrame(safe_volume, columns=["x", "y", "z"])

df.to_csv("geofence.csv")
