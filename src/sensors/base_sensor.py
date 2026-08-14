from dataclasses import dataclass
import random
from typing import Optional

@dataclass
class BaseSensor:
    def __init__(
            self,
            name: str,
            bias: float = 0.0,
            noise_std: float = 0.0,
            seed: Optional[int] = None,
    ):
        self.name = name
        self.bias = bias
        self.noise_std = noise_std
        self.seed = seed

        self._rng = random.Random(self.seed)
        self.last_reading = None
        self.last_time = None

    def reset(self):
        """
        Reset the sensor's internal state, if applicable.
        """
        self._rng = random.Random(self.seed)
        self.last_reading = None
        self.last_time = None

    def calibrate(self, offset: float):
        """
        Calibrate the sensor with a new bias offset.
        """
        self.bias = float(offset)

    def set_noise(self, std: float):
        """
        Set the standard deviation of the sensor's noise.
        """
        self.noise_std = float(std)

    def apply_filter(self, value: float) -> float:
        """
        Apply a filter to the sensor reading, if applicable.
        Can be overridden in subclasses to implement specific filtering techniques.
        """
        return value 

    def read(self, true_value: float, timestamp: Optional[float] = None) -> float:
        """
        Simulate a sensor reading based on the true value, adding bias and noise.
        Optionally, a timestamp can be provided to track when the reading was taken.
        """
        noise = self._rng.gauss(0.0, self.noise_std) if self.noise_std > 0.0 else 0.0
        meas = float(true_value) + self.bias + noise
        meas = self.apply_filter(meas)
        self.last_reading = meas
        self.last_time = timestamp
        return meas