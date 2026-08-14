import math
from typing import Optional
from .base_sensor import BaseSensor


class AngleSensor(BaseSensor):
    """Angle sensor that can wrap angles and apply a low-pass filter (LPF)."""
    def __init__(
            self,
            bias: float = 0.0,
            noise_std: float = 0.0,
            seed: Optional[int] = None,
            wrap: bool = True,
            lpf_alpha: float = 0.0,
    ):
        super().__init__(name="AngleSensor", noise_std=noise_std, bias=bias, seed=seed)
        self.wrap = wrap
        self.lpf_alpha = lpf_alpha


    def wrap_angle(self, a: float) -> float:
        """Wrap angle to [-pi, pi] [-180, 180] degrees."""
        return (a + math.pi) % (2 * math.pi) - math.pi

    def apply_filter(self, value: float) -> float:
        if self.wrap:
            value = self.wrap_angle(value)
        if self.lpf_alpha and self.last_reading is not None:
            # EMA in angle space; when wrapping use sin/cos if needed for stability
            return self.last_reading + self.lpf_alpha * (value - self.last_reading)
        return value