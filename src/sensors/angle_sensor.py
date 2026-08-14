import math
from dataclasses import dataclass

from base_sensor import BaseSensor

@dataclass
class AngleSensor(BaseSensor):
    """Angle sensor that can wrap angles and apply a low-pass filter (LPF)."""
    wrap: bool = True
    lpf_alpha: float = 0.0  # 0=no filter, in (0,1] for EMA

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