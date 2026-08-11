class DualRotorBeamDroneModel:
    def __init__(self, length: float, mass: float, motor_base_thrust: float, max_motor_thrust: float):
        self.length = length
        self.mass = mass
        self.motor_base_thrust = motor_base_thrust
        self.max_motor_thrust = max_motor_thrust
        self.arms_length = length / 2  # Each arm is half the total length
        self.inertia = (1 / 12) * mass * (length ** 2)  # Moment of inertia for a uniform beam

    def calculate_net_torque_with_pid(self, pid_output: float, ):
        """Calculate the net torque on the beam based on PID output."""
        # Mix motor thrusts based on PID output to generate torque
        left_demand = self.motor_base_thrust - pid_output
        right_demand = self.motor_base_thrust + pid_output

        # Clamp thrust between 0 and max_motor_thrust
        left_motor_thrust = max(min(left_demand, self.max_motor_thrust), 0.0)
        right_motor_thrust = max(min(right_demand, self.max_motor_thrust), 0.0)

        # Calculate net torque
        net_torque = (left_motor_thrust - right_motor_thrust) * self.arms_length
        return net_torque
        