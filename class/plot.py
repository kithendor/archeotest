import pandas as pd
import plotly.express as px

# ==============================
# ΦΟΡΤΩΣΗ ΔΕΔΟΜΕΝΩΝ
# ==============================

# Το CSV πρέπει να έχει στήλες: x, y, z
df = pd.read_csv("scan_points.csv")

print("Φορτώθηκαν", len(df), "σημεία")


# ==============================
# 2D ΚΑΤΟΨΗ (TOP-DOWN HEATMAP)
# ==============================

fig2d = px.scatter(
    df,
    x="x",
    y="y",
    color="z",
    color_continuous_scale="Viridis",
    title="2D Κάτοψη χώρου (Top View)",
)

fig2d.update_layout(
    xaxis_title="X (cm)",
    yaxis_title="Y (cm)",
    width=900,
    height=600
)

fig2d.show()


# ==============================
# 3D ΑΠΕΙΚΟΝΙΣΗ ΣΗΜΕΙΩΝ (POINT CLOUD)
# ==============================

fig3d = px.scatter_3d(
    df,
    x="x",
    y="y",
    z="z",
    color="z",
    color_continuous_scale="Viridis",
    title="3D Απεικόνιση Σάρωσης",
)

fig3d.update_layout(
    scene=dict(
        xaxis_title="X (cm)",
        yaxis_title="Y (cm)",
        zaxis_title="Height Z (cm)"
    ),
    width=900,
    height=700
)

fig3d.show()
