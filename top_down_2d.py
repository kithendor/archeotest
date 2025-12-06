import numpy as np
import plotly.express as px

# ---- LOAD POINTS FROM PREVIOUS SCAN ----
# Αν είναι στο ίδιο script, απλά χρησιμοποίησε xs, ys, zs
# Εδώ απλά υποθέτουμε ότι τα έχεις ήδη σε λίστες:
# xs, ys, zs = [...]

# Μετατροπή σε numpy arrays
xs_arr = np.array(xs)
ys_arr = np.array(ys)
zs_arr = np.array(zs)

# ---- 1. FIT GROUND PLANE ----
A = np.c_[xs_arr, ys_arr, np.ones_like(xs_arr)]
coeffs, _, _, _ = np.linalg.lstsq(A, zs_arr, rcond=None)
a, b, c = coeffs

# υπολογισμός επίπεδου εδάφους
plane = a * xs_arr + b * ys_arr + c

# ισοπέδωση εδάφους (ground flattening)
z_flat = zs_arr - plane

print("Ground flattened.")
print(f"Z range after flattening: {z_flat.min():.2f} .. {z_flat.max():.2f}")

# ---- 2. CREATE TOP-DOWN 2D MAP ----
fig = px.scatter(
    x=xs_arr,
    y=ys_arr,
    color=z_flat,
    color_continuous_scale="Viridis",
    width=700,
    height=700,
    title="Top-Down 2D LiDAR Map (Flattened)"
)

# καλύτερη αισθητική
fig.update_traces(marker=dict(size=5))
fig.update_layout(
    xaxis_title="X (cm)",
    yaxis_title="Y (cm)",
    yaxis=dict(scaleanchor="x", scaleratio=1)  # ΤΕΛΕΙΟ: σωστή αναλογία χαρτογράφησης
)

fig.show()
