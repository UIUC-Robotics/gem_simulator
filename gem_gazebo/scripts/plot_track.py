#!/usr/bin/env python3
"""
plot_track.py - Plot the x,y waypoints from track.csv on a 2D figure.

Usage:
    python3 plot_track.py /path/to/track.csv
"""

import sys
import csv
import matplotlib.pyplot as plt

csv_path = sys.argv[1] if len(sys.argv) > 1 else "track.csv"

x_vals, y_vals = [], []
with open(csv_path) as f:
    for row in csv.reader(f):
        if not row:
            continue
        x_vals.append(float(row[0]))
        y_vals.append(float(row[1]))

plt.figure(figsize=(8, 8))
plt.plot(x_vals, y_vals, "-", linewidth=1, color="tab:blue", label="track")
plt.scatter(x_vals[0], y_vals[0], color="green", zorder=5, label="start")
plt.scatter(x_vals[-1], y_vals[-1], color="red", zorder=5, label="end")
plt.axis("equal")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title(f"Track waypoints ({len(x_vals)} points)")
plt.legend()
plt.grid(True)
plt.show()
