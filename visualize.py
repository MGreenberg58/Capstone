import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, BoundaryNorm, TwoSlopeNorm
from scipy.interpolate import interp1d

g = 9.81

def compute_frontier_with_stability(
    boom_mass, payload_mass, base_mass,
    alpha, beta, d_base,
    L_min=15.0, L_max=80.0, L_step=3,
    θ_range=np.linspace(0, 1.5, 100)):
    stable_pts = []
    unstable_pts = []
    frontier_pts = []
    frontier_lengths = []
    frontier_thetas = []

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

            θ_total = θ
            x_tip = L * np.cos(θ_total)
            y_tip = L * np.sin(θ_total)

            if net > 0:
                stable_pts.append((x_tip, y_tip, L))
                max_stable_L = L
            else:
                unstable_pts.append((x_tip, y_tip, L))

            L += L_step

        if max_stable_L is not None:
            θ_total = θ
            x_f = max_stable_L * np.cos(θ_total)
            y_f = max_stable_L * np.sin(θ_total)
            frontier_pts.append((x_f, y_f))
            frontier_lengths.append(max_stable_L)
            frontier_thetas.append(θ)   

    return (
        np.array(stable_pts),
        np.array(unstable_pts),
        np.array(frontier_pts),
        np.array(frontier_lengths),
        np.array(frontier_thetas)
    )


if __name__ == "__main__":

    boom_mass    = 8000.0
    payload_mass = 500.0
    base_mass    = 20000.0

    alpha = 0.5
    beta  = 1.0
    d_base = 4.0

    boom_m_adj = boom_mass * (1 + 0.0) + 0.0
    payload_m_adj = payload_mass * (1 + 0.0) + 0.0
    base_m_adj = base_mass * (1 + 0.0) + 0.0

    min_len = 10 * (1 + 0.0) + 0.0
    max_len = 60 * (1 + 0.0) + 0.0

    ang_range = np.linspace(-0.02, np.deg2rad(90) * (1 + 0.00) , 200) + 0.0

    stable_pts, unstable_pts, frontier_pts, frontier_lengths, frontier_thetas = compute_frontier_with_stability(
        boom_m_adj, payload_m_adj, base_m_adj,
        alpha, beta, d_base,
        L_min=min_len, L_max=max_len, L_step=.2,
        θ_range=ang_range,
    )

    # Rainbow stability envelope plot
    plt.figure(figsize=(10,9))
    Ls_stable = stable_pts[:,2]
    Ls_unstable = unstable_pts[:,2]

    # stable
    scatter = plt.scatter(
        stable_pts[:,0], stable_pts[:,1],
        c=Ls_stable,
        cmap="viridis",
        s=30,
        linewidth=0.1,
        label="Stable"
    )
    cmap = plt.get_cmap("viridis")
    norm = plt.Normalize(min(Ls_stable.min(), Ls_unstable.min()), max(Ls_stable.max(), Ls_unstable.max()))

    # unstable
    plt.scatter(
        unstable_pts[:,0], unstable_pts[:,1],
        facecolors=cmap(norm(Ls_unstable)),
        cmap="viridis",
        s=30,
        linewidth=0.2,
        edgecolor='red',
        label="Unstable"
    )

    plt.axhline(0, color='black', linewidth=1)
    plt.axvline(0, color='black', linewidth=1)
    plt.title("Stability Frontier Colored by Boom Length")
    plt.xlabel("Boom Tip X (m)")
    plt.ylabel("Boom Tip Y (m)")
    plt.axis("equal")
    plt.grid(True)

    legend = plt.legend(markerscale=2)
    for handle in legend.legend_handles:
        if hasattr(handle, "set_linewidths"):
            handle.set_linewidths([1.5])

    cbar = plt.colorbar(scatter)
    cbar.set_label("Boom Length (m)")

    # Non-rainbow stability
    plt.figure(figsize=(10,6))
    f = interp1d(frontier_thetas, frontier_lengths, kind='linear', bounds_error=False, fill_value=np.nan)
    θ_fine = np.linspace(frontier_thetas.min(), frontier_thetas.max(), 500)
    L_frontier = f(θ_fine)

    plt.fill_between(np.rad2deg(θ_fine), 0, L_frontier, color='lightgreen', alpha=0.5, label='Stable Region')
    plt.plot(np.rad2deg(θ_fine), L_frontier, color='green', linewidth=2.2, label='Stability Frontier')

    unstable_thetas = np.array([pt[2] for pt in unstable_pts])
    unstable_angles = np.arctan2([pt[1] for pt in unstable_pts], [pt[0] for pt in unstable_pts])
    plt.scatter(np.rad2deg(unstable_angles), unstable_thetas, color='red', s=10, alpha=0.4, label='Unstable Points')

    plt.title("Crane Stability Chart: Boom Angle vs Max Stable Boom Length")
    plt.xlabel("Boom Angle θ (rad)")
    plt.ylabel("Max Stable Boom Length (m)")
    plt.grid(True)
    plt.legend()

    # Shift comparison plot
    plt.figure(figsize=(8, 6))

    plt.plot(np.rad2deg(frontier_thetas), frontier_lengths,'-o', markersize=3, color='red')
    plt.plot(np.rad2deg(frontier_thetas + 0.03), frontier_lengths,'-o', markersize=3, color='green')
    plt.plot(np.rad2deg(frontier_thetas * 1.01), frontier_lengths,'-o', markersize=3, color='blue')

    plt.title("Stability Frontier: Boom Angle vs Boom Length")
    plt.xlabel("Boom Angle θ (deg)")
    plt.ylabel("Max Stable Boom Length (m)")
    plt.grid(True)


    # red/blue frontier delta L plot
    plt.figure(figsize=(10,9))
    plt.scatter(stable_pts[:,0], stable_pts[:,1], c='lightgray', s=20, label='Stable', alpha=0.5)

    f = interp1d(frontier_thetas, frontier_lengths, kind='linear', bounds_error=False, fill_value=np.nan)

    delta_theta = -np.deg2rad(1)
    L_shifted = f(frontier_thetas + delta_theta)
    delta_L = L_shifted - frontier_lengths
    valid = ~np.isnan(delta_L)

    plt.scatter(stable_pts[:,0], stable_pts[:,1], c='lightgray', s=20, alpha=0.5, label='Stable')
    Xf = frontier_pts[:,0]
    Yf = frontier_pts[:,1]

    scatter = plt.scatter(Xf, Yf, c=delta_L, cmap='coolwarm_r', s=40, edgecolor='black')
    plt.colorbar(scatter, label='ΔBoom Length to be stable(m) for θ - 1 deg')

    plt.title("Stability Frontier Colored by ΔL needed after angle shift")
    plt.xlabel("Boom Tip X (m)")
    plt.ylabel("Boom Tip Y (m)")
    plt.axis("equal")
    plt.grid(True)


    # Delta L plot
    plt.figure(figsize=(8,6))
    plt.plot(frontier_lengths[valid], delta_L[valid], '-o', markersize=4, color='red')
    plt.title("Change in Boom Length in order to be stable if Boom Angle is mismeasured")
    plt.xlabel("Boom Length (m)")
    plt.ylabel("ΔBoom Length to be stable(m) for θ - 1 deg")
    plt.grid(True)


    # Delta Theta plot
    plt.figure(figsize=(8,6))
    f_theta_vs_L = interp1d(frontier_lengths, frontier_thetas, kind='linear', bounds_error=False, fill_value=np.nan)
    L_shifted_theta = frontier_lengths + 0.1
    delta_theta_needed = f_theta_vs_L(L_shifted_theta) - frontier_thetas
    valid_theta = ~np.isnan(delta_theta_needed)

    plt.plot(frontier_lengths[valid_theta], np.rad2deg(delta_theta_needed[valid_theta]), '-o', markersize=4, color='blue')
    plt.title("Change in Boom Angle to remain stable if boom length is mismeasured")
    plt.xlabel("Boom Length (m)")
    plt.ylabel("Δθ (deg) needed for stability")
    plt.grid(True)


    # Rainbow Delta L
    plt.figure(figsize=(10,9))
    f = interp1d(frontier_thetas, frontier_lengths, kind='linear', bounds_error=False, fill_value=np.nan)

    delta_theta = -0.03
    stable_angles = np.arctan2(stable_pts[:,1], stable_pts[:,0])
    L_shifted = f(stable_angles + delta_theta)
    delta_L = L_shifted - stable_pts[:,2]  
    valid = ~np.isnan(delta_L)

    vmin = delta_L[valid].min()  # min value (red)
    vcenter = 1              # value that maps to yellow
    vmax = delta_L[valid].max()  # max value (green)
    norm = TwoSlopeNorm(vmin=vmin, vcenter=vcenter, vmax=vmax)

    scatter = plt.scatter(
        stable_pts[valid,0],
        stable_pts[valid,1],
        c=delta_L[valid],
        cmap='RdYlGn',
        norm=norm,
        s=40)
    
    num_mid_ticks = 3
    mid_ticks = np.linspace(vmin, vmax, num_mid_ticks + 2)
    plt.colorbar(scatter, label='ΔBoom Length to be at stability frontier for θ - 1 deg', ticks=mid_ticks)

    plt.title("Stable Points Colored by ΔL for 1 deg angle decrease")
    plt.xlabel("Boom Tip X (m)")
    plt.ylabel("Boom Tip Y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    # Rainbow delta theta
    plt.figure(figsize=(10,9))
    f = interp1d(frontier_lengths, frontier_thetas, kind='linear', bounds_error=False, fill_value=np.nan)
    L_shifted = stable_pts[:,2] + 0.1
    delta_theta_needed = f(L_shifted) - np.arctan2(stable_pts[:,1], stable_pts[:,0])
    valid = ~np.isnan(delta_theta_needed)

    vmin = delta_theta_needed[valid].min()
    vcenter = -0.1
    vmax = delta_theta_needed[valid].max()

    norm = TwoSlopeNorm(vmin=vmin, vcenter=vcenter, vmax=vmax)

    scatter = plt.scatter(
        stable_pts[valid,0],
        stable_pts[valid,1],
        c=np.rad2deg(delta_theta_needed[valid]),
        cmap='RdYlGn_r',
        norm=norm,
        s=40
    )

    num_mid_ticks = 3
    mid_ticks = np.linspace(vmin, vmax, num_mid_ticks + 2) 
    plt.colorbar(scatter, label='Δθ (rad) needed to be at stability frontier boom length + 0.1m', ticks=mid_ticks)

    plt.axis("equal")
    plt.grid(True)
    plt.title("Stable Points Colored by Δθ for 0.1m boom length increase")
    plt.xlabel("Boom Tip X (m)")
    plt.ylabel("Boom Tip Y (m)")

    # Rainbow delta mass
    plt.figure(figsize=(10,9))
    stable_pts2, unstable_pts2, frontier_pts2, frontier_lengths2, frontier_thetas2 = compute_frontier_with_stability(
        boom_mass, payload_mass * 1.05, base_mass,
        alpha, beta, d_base,
        min_len, max_len, 0.2,
        ang_range
    )

    f_heavier = interp1d(frontier_thetas, frontier_lengths2, kind='linear', bounds_error=False, fill_value=np.nan)

    stable_angles = np.arctan2(stable_pts[:,1], stable_pts[:,0])
    L_heavier = f_heavier(stable_angles)
    delta_L = L_heavier - stable_pts[:,2]
    valid = ~np.isnan(delta_L)

    vmin = delta_L[valid].min()
    vcenter = 2   
    vmax = delta_L[valid].max()

    norm = TwoSlopeNorm(vmin=vmin, vcenter=vcenter, vmax=vmax)

    scatter = plt.scatter(
        stable_pts[valid,0],
        stable_pts[valid,1],
        c=delta_L[valid],
        cmap='RdYlGn',
        norm=norm,
        s=40
    )

    plt.colorbar(scatter, label='ΔBoom Length needed for +5% payload weight')

    plt.title("Stable Points Colored by ΔL for +5% Payload Mass")
    plt.xlabel("Boom Tip X (m)")
    plt.ylabel("Boom Tip Y (m)")
    plt.axis("equal")
    plt.grid(True)

    plt.show()

