"""Dual-rotor suspended beam drone configuration and dynamics model."""

from __future__ import annotations

from dataclasses import dataclass

from .model import Model

import random


@dataclass(slots=True)
class ControlMix:
    left_motor_thrust: float
    right_motor_thrust: float
    net_torque: float


class DualRotorBeamDroneConfig(Model):
    def __init__(
        self,
        mass: float = 1.0,
        length: float = 0.5,
        motor_base_thrust: float = 0.0,
        max_thrust: float = 15.0,
        min_thrust: float = 0.0,
        gravity: float = 9.81,
        angular_damping: float = 0.0,
    ):
        super().__init__("DualRotorBeamDrone")
        self.mass = mass
        self.length = length
        self.motor_base_thrust = motor_base_thrust
        self.max_thrust = max_thrust
        self.min_thrust = min_thrust
        self.gravity = gravity
        self.angular_damping = angular_damping

        self.arms_length = length / 2.0
        self.inertia = (1.0 / 12.0) * mass * (length**2)

    @property
    def base_motor_thrust_per_motor(self) -> float:
        return self.motor_base_thrust


class DualRotorBeamDroneModel:
    def __init__(
        self,
        config: DualRotorBeamDroneConfig | None = None,
        *,
        length: float | None = None,
        mass: float | None = None,
        motor_base_thrust: float | None = None,
        max_motor_thrust: float | None = None,
        min_motor_thrust: float = 0.0,
        angular_damping: float = 0.0,
    ):
        if config is None:
            if length is None or mass is None or motor_base_thrust is None or max_motor_thrust is None:
                raise ValueError(
                    "Provide either a DualRotorBeamDroneConfig or length, mass, motor_base_thrust, and max_motor_thrust."
                )

            config = DualRotorBeamDroneConfig(
                mass=mass,
                length=length,
                motor_base_thrust=motor_base_thrust,
                max_thrust=max_motor_thrust,
                min_thrust=min_motor_thrust,
                angular_damping=angular_damping,
            )

        self.config = config
        self.length = config.length
        self.mass = config.mass
        self.motor_base_thrust = config.motor_base_thrust
        self.max_motor_thrust = config.max_thrust
        self.min_motor_thrust = config.min_thrust
        self.angular_damping = config.angular_damping
        self.arms_length = config.arms_length
        self.inertia = config.inertia

    def _clamp_thrust(self, thrust: float) -> float:
        return max(min(thrust, self.max_motor_thrust), self.min_motor_thrust)

    def calculate_motor_thrusts(self, pid_output: float) -> ControlMix:
        left_demand = self.motor_base_thrust + pid_output
        right_demand = self.motor_base_thrust - pid_output

        left_motor_thrust = self._clamp_thrust(left_demand)
        right_motor_thrust = self._clamp_thrust(right_demand)
        net_torque = (left_motor_thrust - right_motor_thrust) * self.arms_length

        return ControlMix(
            left_motor_thrust=left_motor_thrust,
            right_motor_thrust=right_motor_thrust,
            net_torque=net_torque,
        )

    def calculate_net_torque_with_pid(self, pid_output: float) -> float:
        return self.calculate_motor_thrusts(pid_output).net_torque

    def angular_acceleration(self, pid_output: float, angular_velocity: float = 0.0) -> float:
        control = self.calculate_motor_thrusts(pid_output)
        damping_torque = self.angular_damping * angular_velocity
        return (control.net_torque - damping_torque) / self.inertia

    def step(
        self,
        angle: float,
        angular_velocity: float,
        pid_output: float,
        dt: float,
    ) -> tuple[float, float, float, ControlMix]:
        control = self.calculate_motor_thrusts(pid_output)
        angular_acc = self.angular_acceleration(pid_output, angular_velocity)

        next_angular_velocity = angular_velocity + angular_acc * dt
        next_angle = angle + next_angular_velocity * dt

        return next_angle, next_angular_velocity, angular_acc, control

    def get_current_angle(self, angle: float) -> float:
        return angle + random.gauss(0, 0.001)  # Add small noise to simulate sensor reading
