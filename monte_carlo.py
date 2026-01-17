import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

g = 9.81

def tipping_margin(
    boom_mass, payload_mass, base_mass,
    alpha, beta, d_base,
    L, theta
):
    """
    Returns net stabilizing moment.
    Positive => stable
    Negative => tip-over
    """
    x_boom_cg = alpha * L * np.cos(theta)
    x_payload = beta  * L * np.cos(theta)

    boom_moment    = boom_mass    * g * (-x_boom_cg)
    payload_moment = payload_mass * g * (-x_payload)
    base_moment    = base_mass    * g * d_base

    return boom_moment + payload_moment + base_moment

def tip_probability_monte_carlo(
    L, theta,
    means,
    stds,
    alpha, beta, d_base,
    N=10000,
    seed=None
):
    """
    Monte Carlo estimate of P(tip-over) at a single (L, theta).
    """
    rng = np.random.default_rng(seed)

    boom_mass    = rng.normal(means["boom_mass"],    stds["boom_mass"],    N)
    payload_mass = rng.normal(means["payload_mass"], stds["payload_mass"], N)
    base_mass    = rng.normal(means["base_mass"],    stds["base_mass"],    N)

    theta_mc = rng.normal(theta, stds["theta"], N)
    L_mc     = rng.normal(L,     stds["L"],     N)

    margin = tipping_margin(
        boom_mass, payload_mass, base_mass,
        alpha, beta, d_base,
        L_mc, theta_mc
    )

    return np.mean(margin <= 0)

def compute_probability_field(
    L_vals, theta_vals,
    means, stds,
    alpha, beta, d_base,
    N=10000
):
    P = np.zeros((len(theta_vals), len(L_vals)))

    for i, theta in enumerate(theta_vals):
        for j, L in enumerate(L_vals):
            P[i, j] = tip_probability_monte_carlo(
                L, theta,
                means, stds,
                alpha, beta, d_base,
                N=N
            )

    return P

def extract_probabilistic_frontier(
    L_vals, theta_vals,
    P,
    p_crit=1e-6
):
    frontier_L = []
    frontier_theta = []

    for i, theta in enumerate(theta_vals):
        valid = np.where(P[i, :] <= p_crit)[0]
        if len(valid) > 0:
            j = valid.max()
            frontier_L.append(L_vals[j])
            frontier_theta.append(theta)

    return np.array(frontier_L), np.array(frontier_theta)

def probability_sensitivity(
    L, theta,
    means, stds,
    alpha, beta, d_base,
    var_name,
    eps_fraction=0.05,
    N=10000
):
    """
    Sensitivity of tip probability with respect to:
    - mass means (boom_mass, payload_mass, base_mass)
    - operating variables (theta, L)
    """

    base_p = tip_probability_monte_carlo(
        L, theta,
        means, stds,
        alpha, beta, d_base,
        N=N
    )

    if var_name in means:
        means_perturbed = means.copy()
        means_perturbed[var_name] *= (1 + eps_fraction)

        perturbed_p = tip_probability_monte_carlo(
            L, theta,
            means_perturbed, stds,
            alpha, beta, d_base,
            N=N
        )

        return (perturbed_p - base_p) / (means[var_name] * eps_fraction)

    elif var_name == "theta":
        delta = eps_fraction * max(abs(theta), 1e-3)

        perturbed_p = tip_probability_monte_carlo(
            L, theta + delta,
            means, stds,
            alpha, beta, d_base,
            N=N
        )

        return (perturbed_p - base_p) / delta

    elif var_name == "L":
        delta = eps_fraction * max(abs(L), 1e-2)

        perturbed_p = tip_probability_monte_carlo(
            L + delta, theta,
            means, stds,
            alpha, beta, d_base,
            N=N
        )

        return (perturbed_p - base_p) / delta

    else:
        raise ValueError(f"Unknown variable '{var_name}'")
    
def margin_sensitivity(
    L, theta,
    means,
    alpha, beta, d_base,
    var_name,
    eps_fraction=1e-3
):
    """
    Sensitivity of stability margin g wrt a variable.
    Units:
      - mass:   N·m / kg
      - theta:  N·m / rad
      - L:      N·m / m
    """

    g0 = tipping_margin(
        means["boom_mass"],
        means["payload_mass"],
        means["base_mass"],
        alpha, beta, d_base,
        L, theta
    )

    if var_name in means:
        means_p = means.copy()
        means_p[var_name] *= (1 + eps_fraction)

        g1 = tipping_margin(
            means_p["boom_mass"],
            means_p["payload_mass"],
            means_p["base_mass"],
            alpha, beta, d_base,
            L, theta
        )

        return (g1 - g0) / (means[var_name] * eps_fraction)

    elif var_name == "theta":
        delta = eps_fraction * max(abs(theta), 1e-3)

        g1 = tipping_margin(
            means["boom_mass"],
            means["payload_mass"],
            means["base_mass"],
            alpha, beta, d_base,
            L, theta + delta
        )

        return (g1 - g0) / delta

    elif var_name == "L":
        delta = eps_fraction * max(abs(L), 1e-2)

        g1 = tipping_margin(
            means["boom_mass"],
            means["payload_mass"],
            means["base_mass"],
            alpha, beta, d_base,
            L + delta, theta
        )

        return (g1 - g0) / delta

    else:
        raise ValueError(f"Unknown variable '{var_name}'")

if __name__ == "__main__":

    means = {
        "boom_mass":    8000.0,
        "payload_mass": 500.0,
        "base_mass":    20000.0,
    }

    # Look at these later for exact vals
    stds = {
        "boom_mass":    0.02 * means["boom_mass"], 
        "payload_mass": 0.05 * means["payload_mass"], 
        "base_mass":    0.01 * means["base_mass"],     
        "theta":        np.deg2rad(0.5),               
        "L":            0.05,                           
    }

    alpha = 0.5
    beta = 1.0
    d_base = 4.0

    L_vals = np.linspace(10, 60, 50)
    theta_vals = np.linspace(0, np.deg2rad(90), 90)

    print("Computing probability field...")
    P = compute_probability_field(
        L_vals, theta_vals,
        means, stds,
        alpha, beta, d_base,
        N=5000
    )

    # probability heatmap
    plt.figure(figsize=(9, 6))
    plt.contourf(
        np.rad2deg(theta_vals),
        L_vals,
        P.T,
        levels=30,
        cmap="inferno"
    )
    plt.colorbar(label="Probability of Tip-Over")
    plt.xlabel("Boom Angle (deg)")
    plt.ylabel("Boom Length (m)")
    plt.title("Monte Carlo Tip-Over Probability")
    plt.grid(True)

    L_frontier, theta_frontier = extract_probabilistic_frontier(
        L_vals, theta_vals, P, p_crit=1e-6
    )

    plt.plot(
        np.rad2deg(theta_frontier),
        L_frontier,
        "cyan",
        linewidth=2,
        label="1e-6 Stability Frontier"
    )
    plt.legend()
    plt.show()

    # Example sensitivity evaluation
    L_test = 60.0
    theta_test = np.deg2rad(45)

    for var in ["payload_mass", "theta", "L"]:
        s = probability_sensitivity(
            L_test, theta_test,
            means, stds,
            alpha, beta, d_base,
            var,
            eps_fraction=0.05,
            N=10000
        )
        print(f"Sensitivity dP/d({var}) = {s:.3e}")

    for var in ["payload_mass", "boom_mass", "base_mass", "theta", "L"]:
        s = margin_sensitivity(
            L_test, theta_test,
            means,
            alpha, beta, d_base,
            var,
            eps_fraction=1e-4
        )
        print(f"dg/d({var}) = {s:.3e}")
