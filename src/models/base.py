from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict

@dataclass
class State:
    time: float = 0.0
    position: float = 0.0
    velocity: float = 0.0
    angle: float = 0.0
    angular_velocity: float = 0.0


class BaseModel:
    def __init__(
        self,
        name: str,
        mass: float = 1.0,
        gravity: float = 9.81,
        *,
        dt: float = 0.01,
    ):
        self.name = name
        self.mass = mass
        self.gravity = gravity
        self.dt = dt

        # State of the model
        self.time: float = 0.0
        self.state = State()

        #Other optional metadat that may be provided and useful for the model
        self.meta: Dict[str, Any] = {}


    @property
    def weight(self) -> float:
        """Calculate the weight of the model based on mass and gravity."""
        return self.mass * self.gravity
    

    def reset(self, state: State) -> None:
        """Reset the state of the model to its initial or new set conditions."""
        self.state = state


    def clamp(self, value: float, low: float, high: float) -> float:
        """Clamp a value between a low and high range."""
        return max(low, min(value, high))

    
    def step(self, dt: float = None, *args, **kwargs):
        """ Update the state of the model based on its dynamics.
            This method should be overridden by subclasses to implement specific dynamics.
        """
        if dt is None:
            dt = self.dt
        raise NotImplementedError(f"The step method must be implemented by subclasses of {self.__class__.__name__}.")

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name={self.name}, mass={self.mass})"