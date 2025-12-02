import numpy as np
import matplotlib.pyplot as plt

g = 9.81

# ----- Crane constants -----
boom_mass    = 8000.0      # kg
payload_mass = 500.0       # kg
base_mass    = 20000.0     # kg

alpha = 0.5    # boom CG at mid-length
beta  = 1.0    # payload at tip
d_base = 4.0   # base CG behind pivot (m)

# ----- Extension sweep -----
L_min = 20.0
L_max = 60.0
k     = 5.0     # step size

extensions = np.arange(L_min, L_max + 1e-9, k)

# Angle sweep (radians)
angles = np.linspace(0, 1.5, 600)   # 0° to ~86°

plt.figure(figsize=(8,8))

for L in extensions:
    stable_tip_points = []

    for θ in angles:
        # Boom CG
        x_boom_cg = alpha * L * np.cos(θ)
        boom_lever = -x_boom_cg

        # Payload
        x_payload = beta * L * np.cos(θ)
        payload_lever = -x_payload

        # Restoring moment
        base_lever = d_base

        # Moments
        boom_moment    = boom_mass    * g * boom_lever
        payload_moment = payload_mass * g * payload_lever
        base_moment    = base_mass    * g * base_lever

        net = boom_moment + payload_moment + base_moment

        # print(boom_moment, payload_moment, base_moment)

        if net > 0:  # stable
            x_tip = L * np.cos(θ)
            y_tip = L * np.sin(θ)
            stable_tip_points.append((x_tip, y_tip))

    if len(stable_tip_points) > 0:
        pts = np.array(stable_tip_points)
        plt.plot(pts[:,0], pts[:,1], label=f"L = {L:.0f} m")

# ----- Plot formatting -----
plt.axhline(0, color='black', linewidth=1)
plt.axvline(0, color='black', linewidth=1)

plt.title("Stability Frontier Across Boom Extensions")
plt.xlabel("Boom Tip X (m)")
plt.ylabel("Boom Tip Y (m)")
plt.axis('equal')
plt.grid(True)
plt.legend(title="Boom Extension")
plt.show()
