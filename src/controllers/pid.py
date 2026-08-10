class PIDController:
    def __init__(self, max_integral_error, dt=0.01, kp=0.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_integral_error = max_integral_error
        self.integral_error = 0
        self.previous_error = 0

    def compute(self, setpoint, measured_value) -> float:
        """Calculate the PID output based on the setpoint and measured value."""
        error = setpoint - measured_value

        self.integral_error += error * self.dt
        self.integral_error = max(min(self.integral_error, -self.max_integral_error), self.max_integral_error)

        derivative_error = (error - self.previous_error) / self.dt
        self.previous_error = error

        output = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * derivative_error)
        return output