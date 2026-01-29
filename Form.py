import numpy as np
from scipy.stats import norm
from scipy.optimize import minimize

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
        10.0,            # L (m)
        np.deg2rad(10)   # theta (rad)
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
