import numpy as np
from collections import deque
from typing import Optional, Union

class SensorNoiseModel:
    def __init__(self,
                 dt: float,
                 sigma: Union[float, np.ndarray] = 0.0,
                 cov: Optional[np.ndarray] = None,
                 bias_sigma: float = 0.0,
                 bias_tau: Optional[float] = None,
                 bias_is_random_walk: bool = False,
                 quantization: Optional[float] = None,
                 delay: float = 0.0,
                 dim: int = 1,
                 seed: Optional[int] = None):
   
        self.dt = float(dt)
        self.dim = int(dim)

        # Noise generator
        self._rng = np.random.default_rng(seed)

        # Covariance / sigma handling
        if cov is not None:
            cov = np.array(cov, dtype=float)
            assert cov.shape == (dim, dim), "cov must be dim x dim"
            self.cov = cov
            # We'll use multivariate normal draws from this covariance
        else:
            # sigma may be scalar or vector
            if np.isscalar(sigma):
                self.sigma = float(sigma) * np.ones(dim)
            else:
                self.sigma = np.array(sigma, dtype=float)
                assert self.sigma.shape == (dim,)
            self.cov = None

        # Bias parameters
        self.bias_is_random_walk = bool(bias_is_random_walk)
        self.bias_sigma = float(bias_sigma)
        self.bias_tau = float(bias_tau) if bias_tau is not None else None

        # Initialize bias state vector
        self.bias = np.zeros(dim, dtype=float)

        # Precompute Gauss-Markov coefficient phi = exp(-dt/tau)
        if (not self.bias_is_random_walk) and (self.bias_tau is not None) and (self.bias_tau > 0):
            self._phi = np.exp(-self.dt / self.bias_tau)
            # continuous->discrete process noise variance scaling:
            # Var(w_discrete) = (1 - phi^2) * Var(b_inf)
            # But we accept bias_sigma as the std of the driving noise per discrete step (user-defined).
        else:
            self._phi = None

        # Quantization
        self.quantization = None if (quantization is None or quantization == 0) else float(quantization)

        # Delay implementation: ring buffer of (timestamp, measurement_without_delay)
        self.delay = max(0.0, float(delay))
        self._buffer = deque()  # store tuples (t, value as np.array)
        self._current_time = 0.0

    def _sample_white(self):
        if self.cov is not None:
            return self._rng.multivariate_normal(mean=np.zeros(self.dim), cov=self.cov)
        else:
            return self._rng.normal(loc=0.0, scale=self.sigma, size=self.dim)

    def _update_bias(self):
        if self.bias_sigma == 0:
            return

        if self.bias_is_random_walk:
            # Random walk: b <- b + N(0, (bias_sigma^2) * dt)  (bias_sigma given as std per sqrt(s))
            step_std = self.bias_sigma * np.sqrt(self.dt)
            self.bias += self._rng.normal(0.0, step_std, size=self.dim)
        else:
            # Gauss-Markov: b_{k+1} = phi * b_k + w_b, w_b ~ N(0, q_b)
            # We let bias_sigma act as std of w_b per discrete step (user-provided).
            if self._phi is None:
                # If no tau provided, default to slow random walk-ish (small alpha)
                alpha = 1e-3
                self.bias = (1 - alpha) * self.bias + self._rng.normal(0.0, self.bias_sigma, size=self.dim)
            else:
                w = self._rng.normal(0.0, self.bias_sigma, size=self.dim)
                self.bias = self._phi * self.bias + w

    def _apply_quantization(self, x: np.ndarray) -> np.ndarray:
        if self.quantization is None:
            return x
        # Quantize per-element
        q = self.quantization
        return (np.round(x / q) * q)

    def _push_buffer(self, t: float, value: np.ndarray):
        # Append new ideal+noise (before delay) measurement to buffer
        self._buffer.append((t, value.copy()))

        # Keep buffer size reasonable: remove older entries beyond needed window.
        # We only need to keep samples older than current_time - (delay + dt)
        keep_older_than = t - (self.delay + 2.0 * self.dt)
        while self._buffer and (self._buffer[0][0] < keep_older_than):
            self._buffer.popleft()

    def _pop_delayed(self, t: float) -> np.ndarray:
        if self.delay <= 0.0:
            # No delay: return latest value
            if not self._buffer:
                raise RuntimeError("Buffer empty when no delay.")
            return self._buffer[-1][1].copy()

        target_time = t - self.delay
        # If buffer empty or target_time earlier than oldest, return oldest (clamped)
        if not self._buffer:
            raise RuntimeError("Buffer empty when delay requested.")

        # If target_time <= oldest timestamp, return oldest (clamp)
        if target_time <= self._buffer[0][0]:
            return self._buffer[0][1].copy()

        # If target_time >= newest timestamp, return newest (clamp)
        if target_time >= self._buffer[-1][0]:
            return self._buffer[-1][1].copy()

        # Otherwise find two samples surrounding target_time and linearly interpolate
        # Buffer is small so linear search is fine
        for i in range(len(self._buffer)-1):
            t_i, v_i = self._buffer[i]
            t_j, v_j = self._buffer[i+1]
            if t_i <= target_time <= t_j:
                if t_j == t_i:
                    return v_i.copy()
                alpha = (target_time - t_i) / (t_j - t_i)
                return ((1 - alpha) * v_i + alpha * v_j).copy()

        # Fallback
        return self._buffer[-1][1].copy()

    def measure(self, true_value: Union[float, np.ndarray], t: Optional[float] = None) -> np.ndarray:
        if t is None:
            t = self._current_time

        x = np.array(true_value, dtype=float).reshape(self.dim)

        # 1) Update internal clock and bias state (bias is a stateful process)
        # We assume measure() is called at exactly dt intervals; the user must ensure this.
        # Update bias using dt
        self._update_bias()

        # 2) Add bias to ideal measurement
        meas_with_bias = x + self.bias

        # 3) Add white noise
        wn = self._sample_white()
        meas_noisy = meas_with_bias + wn

        # 4) Quantize (applied before adding to the delay buffer)
        meas_q = self._apply_quantization(meas_noisy)

        # 5) Push into buffer with timestamp t (this is the 'sensor produced' time)
        self._push_buffer(t, meas_q)

        # 6) Return the delayed value (interpolated if fractional)
        out = self._pop_delayed(t)

        # Advance internal time if caller did not supply explicit t progression
        self._current_time = t + self.dt

        return out

    def reset_bias(self, bias: Optional[Union[float, np.ndarray]] = None):
        if bias is None:
            self.bias = np.zeros(self.dim)
        else:
            b = np.array(bias, dtype=float).reshape(self.dim)
            self.bias = b.copy()

    def set_seed(self, seed: Optional[int]):
        self._rng = np.random.default_rng(seed)
