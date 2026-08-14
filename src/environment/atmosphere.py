from dataclasses import dataclass

@dataclass
class Atmosphere:
    """Atmosphere model for simulating environmental conditions."""
    density: float = 1.225  # kg/m^3 at sea level
    temperature: float = 288.15  # Kelvin at sea level
