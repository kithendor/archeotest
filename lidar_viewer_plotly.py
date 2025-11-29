import csv
import numpy as np
import plotly.graph_objects as go

# Load LiDAR scan data
points = []
with open("scan_points.csv") as f:
    next(f)  # skip header
    for row in csv.reader(f):
        points.append((float(row[0]), float(row[1]), float(row[2])))

points = np.array(points)
x, y, z = points[:,0], points[:,1], points[:,2]

# Create 3D scatter visualization
fig = go.Figure(data=[go.Scatter3d(
    x=x, y=y, z=z,
    mode='markers',
    marker=dict(
        size=3,
        color=z,
        colorscale='Viridis',
        opacity=0.9
    )
)])

fig.update_layout(
    title="3D LiDAR Scan Visualization",
    width=950,
    height=700,
    scene=dict(
        xaxis_title="X (cm)",
        yaxis_title="Y (cm)",
        zaxis_title="Z (cm)"
    )
)

fig.show()
