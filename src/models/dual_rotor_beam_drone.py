from dataclasses import dataclass
from typing import Optional
from .base import BaseModel
import numpy as np

@dataclass(slots=True)
class ControlMix:
    left_motor_thrust: float
    right_motor_thrust: float
    net_torque: float

@dataclass(slots=True)
class WindMix:
    left_drag: float
    right_drag: float
    wind_torque: float


class DualRotorBeamDrone(BaseModel):
    def __init__(
            self,
            mass: float = 1.0,
            length: float = 0.5,
            motor_base_thrust: float = 0.0,
            max_motor_thrust: float = 15.0,
            min_motor_thrust: float = 0.0,
            gravity: float = 9.81,
            width: float = 0.01,
            drag_coefficient: float = 1.1,
            air_density: float = 1.225,
    ):
        super().__init__(name="DualRotorBeamDrone", mass=mass, gravity=gravity)

        self.length = length
        self.motor_base_thrust = motor_base_thrust
        self.max_motor_thrust = max_motor_thrust
        self.min_motor_thrust = min_motor_thrust
        self.angular_damping = 0.0

        self.arms_length = self.length / 2.0
        # TODO: Write physics module for inertia calculation
        self.inertia = (1 / 12) * self.mass * (self.length ** 2)
        self.surface_area = self.length * width
        self.drag_coefficient = drag_coefficient
        self.air_density = air_density

    def _clamp_thrust(self, thrust: float) -> float:
        return self.clamp(thrust, self.min_motor_thrust, self.max_motor_thrust)

    def compute_motor_thrusts(self, pid_output: float) -> ControlMix:
        left_demand = self.motor_base_thrust + pid_output
        right_demand = self.motor_base_thrust - pid_output

        left_thrust = self._clamp_thrust(left_demand)
        right_thrust = self._clamp_thrust(right_demand)

        # TODO: WRITE PHYSICS MODULE FOR TORQUE CALCULATION
        net_torque = (left_thrust - right_thrust) * self.arms_length

        return ControlMix(left_motor_thrust=left_thrust, right_motor_thrust=right_thrust, net_torque=net_torque)

    def angular_acceleration(self, pid_output: float, angular_velocity: float = None, wind_torque: float = 0.0) -> float:
        if angular_velocity is None:
            angular_velocity = self.state.angular_velocity

        control_mix = self.compute_motor_thrusts(pid_output)
        damping_torque = self.angular_damping * angular_velocity
        total_torque = ( control_mix.net_torque + wind_torque) - damping_torque
        return total_torque / self.inertia

    def compute_wind_torque(self, v_wind_left: float, v_wind_right: float, air_density: Optional[float] = None) -> float:
        rho = air_density if air_density is not None else self.air_density
        cd = self.drag_coefficient
        area = self.surface_area

        f_left = 0.5 * rho * cd  * area * (v_wind_left ** 2) * np.sign(v_wind_left)
        f_right = 0.5 * rho * cd * area * (v_wind_right ** 2) * np.sign(v_wind_right)

        wind_torque = (f_left - f_right) * self.arms_length
        return WindMix(left_drag=f_left, right_drag=f_right, wind_torque=wind_torque)

    def step(
            self, 
            pid_output: float, 
            v_wind_left: float = 0.0, 
            v_wind_right: float = 0.0, 
            angle: float = None, 
            angular_velocity: float = None, 
            dt: float = None): 
        if dt is None:
            dt = self.dt

        if angular_velocity is None:
            angular_velocity = self.state.angular_velocity

        if angle is None:
            angle = self.state.angle

        control_mix = self.compute_motor_thrusts(pid_output)
        wind_mix = self.compute_wind_torque(v_wind_left, v_wind_right)


        angular_acc = self.angular_acceleration(
            pid_output = pid_output,
            angular_velocity = angular_velocity,
            wind_torque = wind_mix.wind_torque
            )

        next_angular_velocity = angular_velocity + angular_acc * dt
        next_angle = angle + next_angular_velocity * dt

        self.time += dt
        self.state.time = self.time
        self.state.angle = next_angle
        self.state.angular_velocity = next_angular_velocity

        return next_angle, next_angular_velocity, angular_acc, control_mix, wind_mix