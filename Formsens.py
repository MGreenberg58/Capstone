import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from scipy.optimize import minimize
from scipy.stats import norm
from scipy.interpolate import interp1d

# Gravity
g = 9.81

def compute_frontier(
    boom_mass, payload_mass, base_mass,
    alpha_geom, beta_geom, d_base,
    L_min, L_max, L_step,
    theta_range
):
    frontier_lengths = []
    frontier_thetas = []

    for θ in theta_range:
        max_stable_L = None
        L = L_min

        while L <= L_max:

            x_boom_cg = alpha_geom * L * np.cos(θ)
            x_payload = beta_geom  * L * np.cos(θ)

            boom_moment    = boom_mass    * g * (-x_boom_cg)
            payload_moment = payload_mass * g * (-x_payload)
            base_moment    = base_mass    * g * d_base

            net = boom_moment + payload_moment + base_moment

            if net > 0:
                max_stable_L = L
            else:
                break

            L += L_step

        if max_stable_L is not None:
            frontier_lengths.append(max_stable_L)
            frontier_thetas.append(θ)

    return np.array(frontier_lengths), np.array(frontier_thetas)

def tipping_margin(x, alpha_geom, beta_geom, d_base):
    boom_mass, payload_mass, base_mass, L, theta = x

    x_boom_cg = alpha_geom * L * np.cos(theta)
    x_payload = beta_geom  * L * np.cos(theta)

    boom_moment    = boom_mass    * g * (-x_boom_cg)
    payload_moment = payload_mass * g * (-x_payload)
    base_moment    = base_mass    * g * d_base

    return -(boom_moment + payload_moment + base_moment)

def u_to_x(u, means, stds):
    return means + stds * u

def grad_g_x(x, alpha_geom, beta_geom, d_base):
    boom_mass, payload_mass, base_mass, L, theta = x

    common = g * np.cos(theta)
    mass_combo = alpha_geom * boom_mass + beta_geom * payload_mass

    dg_dboom = g * alpha_geom * L * np.cos(theta)
    dg_dpayload = g * beta_geom * L * np.cos(theta)
    dg_dbase = -g * d_base
    dg_dL = g * np.cos(theta) * mass_combo
    dg_dtheta = -g * L * np.sin(theta) * mass_combo

    return np.array([
        dg_dboom,
        dg_dpayload,
        dg_dbase,
        dg_dL,
        dg_dtheta
    ])

def form_analysis(means, stds, alpha_geom, beta_geom, d_base, u,
                  tol=1e-6, max_iter=100):


    for _ in range(max_iter):

        x = means + stds * u
        g_val = tipping_margin(x, alpha_geom, beta_geom, d_base)

        grad_x = grad_g_x(x, alpha_geom, beta_geom, d_base)
        grad_u = grad_x * stds

        norm_grad = np.linalg.norm(grad_u)

        if norm_grad == 0:
            raise RuntimeError("Zero gradient encountered")

        # HL-RF update
        u_new = ((grad_u @ u - g_val) / norm_grad**2) * grad_u

        if np.linalg.norm(u_new - u) < tol:
            u = u_new
            break

        u = u_new
    else:
        raise RuntimeError("HL-RF did not converge")

    beta = np.linalg.norm(u)
    Pf = norm.cdf(-beta)

    return beta, Pf, u


def importance_factors(u_star):
    beta = np.linalg.norm(u_star)
    return u_star / beta

def pf_sensitivity(alpha_vec, stds, beta):
    phi = norm.pdf(beta)
    return phi * alpha_vec / stds

def beta_sensitivity(alpha_vec, stds):
    return alpha_vec / stds


if __name__ == "__main__":

    boom_mass    = 8000.0
    payload_mass = 500.0
    base_mass    = 20000.0

    alpha_geom = 0.5
    beta_geom  = 1.0
    d_base     = 4.0

    means = np.array([
        boom_mass,
        payload_mass,
        base_mass,
        19.0,
        np.deg2rad(20)
    ])

    stds = np.array([
        0.02 * boom_mass,
        0.05 * payload_mass,
        0.01 * base_mass,
        0.05,
        np.deg2rad(0.5)
    ])

    names = ["boom_mass", "payload_mass", "base_mass", "L", "theta"]

    theta_range = np.linspace(0, np.deg2rad(90), 250)

    frontier_lengths, frontier_thetas = compute_frontier(
        boom_mass, payload_mass, base_mass,
        alpha_geom, beta_geom, d_base,
        L_min=10,
        L_max=60,
        L_step=0.05,
        theta_range=theta_range
    )

    beta_list = []
    Pf_list = []
    theta_valid = []
    L_valid = []

    pf_sens_hist = {name: [] for name in names}
    beta_sens_hist = {name: [] for name in names}
    u_prev = None

    for Lf, θf in zip(frontier_lengths, frontier_thetas):

        means_local = np.array([
            means[0],
            means[1],
            means[2],
            Lf,
            θf
        ])

        if u_prev is None:
            u0 = np.zeros_like(means_local)
        else:
            u0 = u_prev
        
        try:
            beta, Pf, u_star = form_analysis(
                means_local, stds,
                alpha_geom, beta_geom, d_base, u0
            )
        except RuntimeError:
            continue

        alpha_vec = importance_factors(u_star)

        pf_sens = pf_sensitivity(alpha_vec, stds, beta)
        beta_sens = beta_sensitivity(alpha_vec, stds)

        beta_list.append(beta)
        Pf_list.append(Pf)
        theta_valid.append(θf)
        L_valid.append(Lf)

        for name, s_pf, s_beta in zip(names, pf_sens, beta_sens):
            pf_sens_hist[name].append(s_pf)
            beta_sens_hist[name].append(s_beta)

        u_prev = u_star

    theta_valid = np.array(theta_valid)
    L_valid = np.array(L_valid)

    for k in pf_sens_hist:
        pf_sens_hist[k] = np.array(pf_sens_hist[k])
        beta_sens_hist[k] = np.array(beta_sens_hist[k])

    x_frontier = L_valid * np.cos(theta_valid)
    y_frontier = L_valid * np.sin(theta_valid)

    for name in names:

        # Pf Sense
        plt.figure(figsize=(8,7))
        sens_vals = pf_sens_hist[name]

        vmin = sens_vals.min()
        vmax = sens_vals.max()

        if vmin < 0 and vmax > 0:
            norm_color = TwoSlopeNorm(
                vmin=vmin,
                vcenter=0,
                vmax=vmax
            )
        else:
            norm_color = None 

        scatter = plt.scatter(
            x_frontier,
            y_frontier,
            c=sens_vals,
            cmap="RdYlGn_r",
            norm=norm_color,
            s=60
        )

        plt.colorbar(scatter, label=f"∂Pf / ∂μ_{name}")
        plt.title(f"Sensitivity of Pf to {name}")
        plt.xlabel("Boom Tip X (m)")
        plt.ylabel("Boom Tip Y (m)")
        plt.axis("equal")
        plt.grid(True)

        # Beta Sens
        plt.figure(figsize=(8,7))
        sens_vals = beta_sens_hist[name]

        vmin = sens_vals.min()
        vmax = sens_vals.max()
        
        if vmin < 0 and vmax > 0:
            norm_color = TwoSlopeNorm(
                vmin=vmin,
                vcenter=0,
                vmax=vmax
            )
        else:
            norm_color = None 

        scatter = plt.scatter(
            x_frontier,
            y_frontier,
            c=sens_vals,
            cmap="RdYlGn_r",
            norm=norm_color,
            s=60
        )

        plt.colorbar(scatter, label=f"∂β / ∂μ_{name}")
        plt.title(f"Sensitivity of Reliability Index β to {name}")
        plt.xlabel("Boom Tip X (m)")
        plt.ylabel("Boom Tip Y (m)")
        plt.axis("equal")
        plt.grid(True)

  
    # plt.figure(figsize=(8,7))
    # plt.plot(np.rad2deg(theta_valid), pf_sens_hist["L"])
    # plt.plot(theta_valid, beta_sens_hist)

    plt.show()

