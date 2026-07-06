class PIDController:
    def __init__(self, dt: float, max_integral_error: float, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_integral_error = max_integral_error

        self.integral_error = 0.0
        self.previous_error = 0.0

    def update(self, target: float, measurement: float) -> float:
        error = target - measurement

        self.integral_error += error * self.dt
        self.integral_error = self.clamp(self.integral_error, -self.max_integral_error, self.max_integral_error)

        derivative_error = (error - self.previous_error) / self.dt
        self.previous_error = error

        return (self.kp * error) + (self.ki * self.integral_error) + (self.kd * derivative_error)
    
    @staticmethod
    def clamp(self, value: float, max_value: float, min_value: float = 0.0) -> float:
        return max(min(value, max_value), min_value)