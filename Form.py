import numpy as np
from scipy.stats import norm
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def compute_frontier_with_stability(
    boom_mass, payload_mass, base_mass,
    alpha, beta, d_base,
    L_min=10.0, L_max=60.0, L_step=3,
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

"""
FORM + Importance Sampling for Crane Tip-Over Reliability

Angle convention:
  theta = 0 rad   -> boom horizontal
  theta = pi/2    -> boom vertical
  Positive theta  -> counter-clockwise

Failure condition:
  g(x) <= 0  => tip-over
"""

g = 9.81


def tipping_margin(x, alpha, beta, d_base):
    """
    x = [boom_mass, payload_mass, base_mass, L, theta]
    Returns stabilizing moment (N·m)
    """
    boom_mass, payload_mass, base_mass, L, theta = x

    x_boom_cg = alpha * L * np.cos(theta)
    x_payload = beta  * L * np.cos(theta)

    boom_moment    = boom_mass    * g * (-x_boom_cg)
    payload_moment = payload_mass * g * (-x_payload)
    base_moment    = base_mass    * g * d_base

    return -(boom_moment + payload_moment + base_moment)


def u_to_x(u, means, stds):
    """Map standard normal u -> physical x"""
    return means + stds * u


def x_to_u(x, means, stds):
    """Map physical x -> standard normal u"""
    return (x - means) / stds


def form_analysis(means, stds, alpha, beta, d_base):
    """
    Returns:
      beta_form  : reliability index
      Pf_form    : FORM probability estimate
      u_star     : MPP in standard normal space
      x_star     : MPP in physical space
    """

    means = np.array(means)
    stds  = np.array(stds)

    def objective(u):
        return np.linalg.norm(u)

    def constraint(u):
        x = u_to_x(u, means, stds)
        return tipping_margin(x, alpha, beta, d_base)

    cons = {"type": "eq", "fun": constraint}

    u0 = np.zeros_like(means)

    res = minimize(
        objective,
        u0,
        method="SLSQP",
        constraints=cons,
        options={"ftol": 1e-9, "disp": False}
    )

    if not res.success:
        raise RuntimeError("FORM optimization failed")

    u_star = res.x
    beta_form = np.linalg.norm(u_star)
    Pf_form = norm.cdf(-beta_form)
    x_star = u_to_x(u_star, means, stds)

    return beta_form, Pf_form, u_star, x_star

def importance_sampling(
    means, stds,
    alpha, beta, d_base,
    u_star,
    N=200000
):
    """
    Importance sampling centered at FORM MPP (numerically stable)
    """

    dim = len(means)
    rng = np.random.default_rng()

    # Sample from N(u*, I)
    u_samples = rng.normal(size=(N, dim)) + u_star
    x_samples = means + stds * u_samples

    # Evaluate limit state
    g_vals = np.array([
        tipping_margin(x, alpha, beta, d_base)
        for x in x_samples
    ])

    # Indicator of failure
    I = (g_vals <= 0).astype(float)

    # Stable likelihood ratio
    exponent = -np.dot(u_samples, u_star) + 0.5 * np.dot(u_star, u_star)
    weights = np.exp(exponent)

    Pf_IS = np.mean(I * weights)

    return Pf_IS

def importance_factors(u_star):
    beta = np.linalg.norm(u_star)
    alpha = u_star / beta
    return alpha

def mean_sensitivities(alpha_vec, stds, beta):
    phi = norm.pdf(beta)
    dPf_dmu = phi * alpha_vec / stds
    return dPf_dmu

def importance_sampling_form(
    means, stds,
    alpha_geom, beta_form, u_star,
    alpha, beta, d_base,
    N=100000,
    c=3.0):

    dim = len(means)
    rng = np.random.default_rng()

    # Covariance aligned with failure direction
    Sigma = np.eye(dim) + (c - 1) * np.outer(alpha_geom, alpha_geom)
    L = np.linalg.cholesky(Sigma)

    z = rng.normal(size=(N, dim))
    u_samples = u_star + z @ L.T
    x_samples = means + stds * u_samples

    g_vals = np.array([
        tipping_margin(x, alpha, beta, d_base)
        for x in x_samples
    ])

    I = (g_vals <= 0).astype(float)

    # Likelihood ratio
    inv_Sigma = np.linalg.inv(Sigma)
    log_w = (
        -0.5 * np.sum(u_samples**2, axis=1)
        +0.5 * np.sum((u_samples - u_star) @ inv_Sigma * (u_samples - u_star), axis=1)
    )

    weights = np.exp(log_w)
    Pf = np.mean(I * weights)

    return Pf


if __name__ == "__main__":

    # Mean values
    means = np.array([
        8000.0,          # boom_mass
        500.0,           # payload_mass
        20000.0,         # base_mass
        19.0,            # L (m)
        np.deg2rad(20)   # theta (rad)
    ])

    stds = np.array([
        0.02 * means[0],
        0.05 * means[1],
        0.01 * means[2],
        0.05,
        np.deg2rad(0.5)
    ])

    alpha = 0.5
    beta = 1.0
    d_base = 4.0


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

    ang_range = np.linspace(-0.02, np.deg2rad(90) * (1 + 0.00) , 500) + 0.0

    stable_pts, unstable_pts, frontier_pts, frontier_lengths, frontier_thetas = compute_frontier_with_stability(
        boom_m_adj, payload_m_adj, base_m_adj,
        alpha, beta, d_base,
        L_min=min_len, L_max=max_len, L_step=1,
        θ_range=ang_range,
    )

    print("Running FORM analysis...")
    beta_form, Pf_form, u_star, x_star = form_analysis(
        means, stds,
        alpha, beta, d_base
    )

    print(f"Reliability index beta = {beta_form:.3f}")
    print(f"FORM probability Pf  = {Pf_form:.3e}")
    print("MPP in physical space:")
    print(f"  boom_mass    = {x_star[0]:.2f} kg")
    print(f"  payload_mass = {x_star[1]:.2f} kg")
    print(f"  base_mass    = {x_star[2]:.2f} kg")
    print(f"  L            = {x_star[3]:.2f} m")
    print(f"  theta        = {np.rad2deg(x_star[4]):.2f} deg")

    print("\nRunning Importance Sampling...")
    Pf_IS = importance_sampling(
        means, stds,
        alpha, beta, d_base,
        u_star,
        N=100000
    )

    print(f"Importance Sampling Pf = {Pf_IS:.3e}")

    alpha_vec = importance_factors(u_star)

    names = ["boom_mass", "payload_mass", "base_mass", "L", "theta"]

    print("\nFORM Importance Factors:")
    for name, a in zip(names, alpha_vec):
        print(f"  {name:12s}: {a:+.3f}")

    sens_mu = mean_sensitivities(alpha_vec, stds, beta_form)

    print("\nSensitivity of Pf to mean values:")
    for name, s in zip(names, sens_mu):
        print(f"  dPf/dmu {name:12s}: {s:+.3e}")


    from collections import defaultdict

    names = ["boom_mass", "payload_mass", "base_mass", "L", "theta"]

    alpha_history = defaultdict(list)
    beta_history = []
    Pf_history = []
    theta_valid = []
    for Lf, θf in zip(frontier_lengths, frontier_thetas):

        means_local = np.array([
            means[0],
            means[1],
            means[2],
            Lf,
            θf
        ])

        stds_local = np.array([
            stds[0],
            stds[1],
            stds[2],
            stds[3],
            stds[4]
        ])

        try:
            β, Pf, u_star, _ = form_analysis(
            means_local, stds,
            alpha, beta, d_base)
        except RuntimeError:
            # Skip points where FORM fails
            continue


        α_vec = importance_factors(u_star)


        beta_history.append(β)
        Pf_history.append(Pf)
        theta_valid.append(θf)


        for name, a in zip(names, α_vec):
            alpha_history[name].append(a)

    theta_valid = np.array(theta_valid)
    beta_history = np.array(beta_history)
    for k in alpha_history:
        alpha_history[k] = np.array(alpha_history[k])
    Pf_history = np.array(Pf_history)

    plt.figure(figsize=(10,6))

    for name in names:
        plt.plot(
            np.rad2deg(theta_valid),
            alpha_history[name],
            label=name
        )

    plt.axhline(0, color="k", lw=0.8)
    plt.xlabel("Boom Angle θ (deg)")
    plt.ylabel("FORM Importance Factor α")
    plt.title("Uncertainty Importance Along Deterministic Stability Frontier")
    plt.legend()
    plt.grid(True)
    plt.show()

    plt.figure()
    plt.plot(np.rad2deg(theta_valid), beta_history)
    plt.xlabel("Boom Angle θ (deg)")
    plt.ylabel("Reliability Index β")
    plt.title("Reliability Index Along Stability Frontier")
    plt.grid(True)
    plt.show()
