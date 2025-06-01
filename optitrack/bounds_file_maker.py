#!/usr/bin/env python3
"""
Compute plane equations for an extruded prism defined by a 2D polygon (safe_vertices)
from z = 0 to z = 3.6. Each plane is given as [n_x, n_y, n_z, d], with the normal
pointing outward.
"""

import numpy as np
import json

def compute_extruded_planes(vertices, z_min=0.0, z_max=3.6):
    """
    Given a list of 2D vertices (Nx2 array) defining a closed polygon (in order),
    compute the six planes (four sides + top + bottom) of the prism extruded between
    z = z_min and z = z_max. Each plane is returned as [n_x, n_y, n_z, d], where
      n = (n_x, n_y, n_z) is a unit normal pointing inward,
      and the plane equation is n·[x, y, z] = d.
    """
    verts = np.asarray(vertices, dtype=float)
    if verts.ndim != 2 or verts.shape[1] != 2:
        raise ValueError("vertices must be an (N x 2) array of [x, y] coordinates.")
    N = verts.shape[0]
    if N < 3:
        raise ValueError("At least 3 vertices are required to define a polygon.")

    # 1) Compute centroid of the polygon in XY
    centroid = np.mean(verts, axis=0)

    planes = []

    # 2) For each edge, compute the inward normal in the XY plane
    for i in range(N):
        v_curr = verts[i]
        v_next = verts[(i + 1) % N]
        edge_vec = v_next - v_curr  # [dx, dy]

        # A perpendicular (candidate) normal in XY: [dy, -dx]
        n_xy = np.array([edge_vec[1], -edge_vec[0]])

        # Normalize to unit length
        norm = np.linalg.norm(n_xy)
        if norm == 0:
            raise ValueError(f"Edge {i} has zero length.")
        n_xy /= norm

        # Compute d = n_x * x0 + n_y * y0  (for z=irrelevant)
        d = n_xy.dot(v_curr)

        # Ensure the normal points inward: the centroid should satisfy n·centroid <= d
        if n_xy.dot(centroid) >= d:
            # flip
            n_xy = -n_xy
            d = -d

        # The side plane has n_z = 0
        plane = [n_xy[0], n_xy[1], 0.0, d]
        planes.append(plane)

    # 3) Top plane: z = z_max → normal [0, 0,  1], so d = 1·z_max
    planes.append([0.0, 0.0, -1.0, z_max])

    # 4) Bottom plane: z = z_min → normal [0, 0, -1], so d = (-1)·z_min = 0 if z_min=0
    planes.append([0.0, 0.0, 1.0, -z_min])

    return planes


if __name__ == "__main__":
    # Define the base polygon vertices
    safe_vertices = np.array([
        [2.6, -2.3],
        [-2.6, -2.3],
        [ -2.1, 2.9 ],
        [3.8,  1.67]
    ])

    # Compute planes
    planes = compute_extruded_planes(safe_vertices, z_min=0.0, z_max=3.6)

    with open("bounds.json", "w") as f:
        json.dump(planes, f, indent=4)

    # Print them in the requested format
    print("planes = [")
    for p in planes:
        # Format each plane as [n_x, n_y, n_z, d]
        print("    [{:.4f}, {:.4f}, {:.4f}, {:.4f}],".format(p[0], p[1], p[2], p[3]))
    print("]")
