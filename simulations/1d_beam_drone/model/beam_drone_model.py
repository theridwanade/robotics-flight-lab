class DualRotorBeamDroneModel:
    def __init__(self, length: float, mass: float):
        self.length = length
        self.mass = mass
        self.arms_length = length / 2  # Each arm is half the total length
        self.inertia = (1 / 12) * mass * (length ** 2)  # Moment of inertia for a uniform beam
