# %%
import pandas as pd
import polars as pl
import scipy.spatial
import matplotlib.pyplot as plt

import argparse

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

parser = argparse.ArgumentParser()
parser.add_argument("filename", help="The csv file exported from `ros2 topic echo --csv <topicname>`")

args = parser.parse_args()
data = pd.read_csv(args.filename, names=h, index_col=False)
data = pl.from_dataframe(data)

# %%
validPoints = data.filter(pl.col("rb_trackingValid"))

validPoints_xyz = validPoints.select([
    "rb_x", 
    "rb_y",
    "rb_z"
])

validPoints_xyz
hull = scipy.spatial.ConvexHull(validPoints_xyz)
fig = plt.figure()
ax = fig.add_subplot(projection="3d")

# ax.scatter(validPoints_xyz["rb_x"], validPoints_xyz["rb_y"], validPoints_xyz["rb_z"])

for simplex in hull.simplices:
    ax.plot(
        validPoints_xyz[simplex, 0], 
        validPoints_xyz[simplex, 1], 
        validPoints_xyz[simplex, 2], 
        'k-'
    )



plt.show()