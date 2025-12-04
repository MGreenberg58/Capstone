import numpy as np
import matplotlib.pyplot as plt

g = 9.81

# -------------------------------------------------------------------
# Compute stability frontier by sweeping L for each θ
# Keep BOTH stable and unstable points
# -------------------------------------------------------------------
def compute_frontier_with_stability(
    boom_mass, payload_mass, base_mass,
    alpha, beta, d_base,
    L_min=15.0, L_max=80.0, L_step=3,
    θ_range=np.linspace(0, 1.5, 100),
    θ_offset=0.0
):
    stable_pts = []
    unstable_pts = []
    frontier_pts = []
    frontier_lengths = []
    frontier_thetas = []   # NEW

    for θ in θ_range:
        max_stable_L = None
        L = L_min

        while L <= L_max:
            x_boom_cg = alpha * L * np.cos(θ)
            x_payload = beta * L * np.cos(θ)

            boom_moment    = boom_mass    * g * (-x_boom_cg)
            payload_moment = payload_mass * g * (-x_payload)
            base_moment    = base_mass    * g * d_base

            net = boom_moment + payload_moment + base_moment

            θ_total = θ + θ_offset
            x_tip = L * np.cos(θ_total)
            y_tip = L * np.sin(θ_total)

            if net > 0:
                stable_pts.append((x_tip, y_tip, L))
                max_stable_L = L
            else:
                unstable_pts.append((x_tip, y_tip, L))

            L += L_step

        if max_stable_L is not None:
            θ_total = θ + θ_offset
            x_f = max_stable_L * np.cos(θ_total)
            y_f = max_stable_L * np.sin(θ_total)
            frontier_pts.append((x_f, y_f))
            frontier_lengths.append(max_stable_L)
            frontier_thetas.append(θ)   # NEW

    return (
        np.array(stable_pts),
        np.array(unstable_pts),
        np.array(frontier_pts),
        np.array(frontier_lengths),
        np.array(frontier_thetas)
    )

# -------------------------------------------------------------------
# Apply offsets (absolute or percentage)
# -------------------------------------------------------------------
def apply_offsets(
    boom_mass, payload_mass, base_mass,
    boom_offset=0.0, payload_offset=0.0, base_offset=0.0,
    boom_pct=0.0, payload_pct=0.0, base_pct=0.0,
    θ_offset=0.0, L_min_offset=0.0, L_max_offset=0.0,
    L_min_pct=0.0, L_max_pct=0.0
):
    """
    Returns updated masses, θ_offset, and L range after applying offsets
    pct = fraction (0.1 = +10%)
    """
    boom_m    = boom_mass    * (1 + boom_pct)    + boom_offset
    payload_m = payload_mass * (1 + payload_pct) + payload_offset
    base_m    = base_mass    * (1 + base_pct)    + base_offset

    L_min_new = L_min_offset + L_min_pct * 1.0  # we can add fraction later in main
    L_max_new = L_max_offset + L_max_pct * 1.0  # placeholder

    return boom_m, payload_m, base_m, θ_offset, L_min_new, L_max_new

# -------------------------------------------------------------------
# Interpolate the frontier curve
# -------------------------------------------------------------------
def interpolate_frontier(pts, num=500):
    pts = pts[np.argsort(pts[:,0])]
    X = pts[:,0]
    Y = pts[:,1]

    Xi = np.linspace(X.min(), X.max(), num)
    Yi = np.interp(Xi, X, Y)
    return Xi, Yi


# -------------------------------------------------------------------
# Main
# -------------------------------------------------------------------
if __name__ == "__main__":

    boom_mass    = 8000.0
    payload_mass = 500.0
    base_mass    = 20000.0

    alpha = 0.5
    beta  = 1.0
    d_base = 4.0

    

    stable_pts, unstable_pts, frontier_pts, frontier_lengths, frontier_thetas = compute_frontier_with_stability(
            boom_mass, payload_mass, base_mass,
            alpha, beta, d_base,
            L_min=20.0, L_max=80.0, L_step=.5,
            θ_range=np.linspace(0, 1.5, 600)
        )

    Xi, Yi = interpolate_frontier(frontier_pts)

    # ------------------ PLOT -------------------
    plt.figure(figsize=(10,9))

    # Normalize L values for colormap
    Ls_stable = stable_pts[:,2]
    Ls_unstable = unstable_pts[:,2]

    # Stable filled circles
    scatter = plt.scatter(
        stable_pts[:,0], stable_pts[:,1],
        c=Ls_stable,
        cmap="viridis",
        s=30,
        linewidth=0.3,
        label="Stable"
    )

    cmap = plt.get_cmap("viridis")
    norm = plt.Normalize(min(Ls_stable.min(), Ls_unstable.min()), max(Ls_stable.max(), Ls_unstable.max()))

    # Unstable hollow circles
    plt.scatter(
        unstable_pts[:,0], unstable_pts[:,1],
        facecolors=cmap(norm(Ls_unstable)),
        cmap="viridis",
        s=30,
        linewidth=0.3,
        edgecolor='red',
        label="Unstable"
    )

    # # Frontier curve (smooth)
    # plt.plot(Xi, Yi, color="yellow", linewidth=2.2,
    #          label="Stability Frontier (Max L per θ)")

    # Formatting
    plt.axhline(0, color='black', linewidth=1)
    plt.axvline(0, color='black', linewidth=1)

    plt.title("Stability Frontier Colored by Boom Length")
    plt.xlabel("Boom Tip X (m)")
    plt.ylabel("Boom Tip Y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    # Colorbar for boom length
    cbar = plt.colorbar(scatter)
    cbar.set_label("Boom Length (m)")

    plt.figure(figsize=(8,6))
    plt.plot(frontier_lengths, frontier_thetas, '-o', markersize=4, color='blue')
    plt.title("Max Stable Boom Length vs Angle")
    plt.xlabel("Stable Boom Length (m)")
    plt.ylabel("Max Boom angle (rad)")
    plt.grid(True)
    plt.show()

