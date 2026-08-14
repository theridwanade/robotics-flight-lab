from models.base import BaseModel
from dataclasses import dataclass

@dataclass(slots=True)
class ControlMix:
    left_motor_thrust: float
    right_motor_thrust: float
    net_torque: float



class DualRotorBeamDrone(BaseModel):
    def __init__(
            self,
            mass: float = 1.0,
            length: float = 0.5,
            motor_base_thrust: float = 0.0,
            max_motor_thrust: float = 15.0,
            min_motor_thrust: float = 0.0,
            gravity: float = 9.81,
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

    def angular_acceleration(self, pid_output: float, angular_velocity: float = None) -> float:
        if angular_velocity is None:
            angular_velocity = self.state.angular_velocity
        control_mix = self.compute_motor_thrusts(pid_output)
        damping_torque = self.angular_damping * angular_velocity
        return (control_mix.net_torque - damping_torque) / self.inertia

    def step(self, angle: float, angular_velocity: float, pid_output: float, dt: float = None): 
        if dt is None:
            dt = self.dt

        control_mix = self.compute_motor_thrusts(pid_output)
        angular_acc = self.angular_acceleration(pid_output, angular_velocity)

        next_angular_velocity = angular_velocity + angular_acc * dt
        next_angle = angle + next_angular_velocity * dt

        self.time += dt
        self.state.time = self.time
        self.state.angle = next_angle
        self.state.angular_velocity = next_angular_velocity

        return next_angle, next_angular_velocity, angular_acc, control_mix