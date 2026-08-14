from dataclasses import dataclass
import numpy as np
from scipy.signal import bilinear, lfilter


@dataclass
class DrydenConfig:
    """Configuration parameters for the Dryden wind model."""
    mean_velocity: float = 2.5  # Mean wind velocity in m/s
    sigma_w: float = 0.6
    length_scale_Lw: float = 30.0  # Length scale for the wind model in meters
    nominal_airspeed_V: float = 5.0  # Nominal wind speed in m/s


class DrydenWindGenerator:
    def __init__(self, config: DrydenConfig = DrydenConfig(), dt: float = 0.01):
        self.config = config
        self.dt = dt
        self._build_filter()


    def _build_filter(self):
        sigma, L_w, V = self.config.sigma_w, self.config.length_scale_Lw, self.config.nominal_airspeed_V
        K = sigma * np.sqrt(L_w / (np.pi * V))
        T = L_w / V
        nums_s = [K * np.sqrt(3) * T, K]
        dens_s = [T**2, 2 * T, 1.0]
        self.b_disc, self.a_disc = bilinear(nums_s, dens_s, fs=1.0 /self.dt)

    def generate_velocity_series(self, num_steps: int, seed: int = None) -> np.ndarray:
        if seed is not None:
            np.random.seed(seed)

        white_noise = np.random.normal(0.0, 1.0 / np.sqrt(self.dt), num_steps)
        turbulence = lfilter(self.b_disc, self.a_disc, white_noise)
        return turbulence + self.config.mean_velocity

    
