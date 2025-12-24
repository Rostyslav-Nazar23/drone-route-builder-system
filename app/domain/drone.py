"""Drone domain model."""
from dataclasses import dataclass
from typing import Optional


@dataclass
class Drone:
    """Represents a drone with its capabilities and constraints."""
    name: str
    max_speed: float  # m/s
    max_altitude: float  # meters
    min_altitude: float  # meters
    battery_capacity: float  # Wh (Watt-hours)
    power_consumption: float  # W (Watts) at cruise speed
    max_flight_time: Optional[float] = None  # seconds, calculated if None
    max_range: Optional[float] = None  # meters, calculated if None
    turn_radius: float = 50.0  # meters, minimum turning radius
    climb_rate: float = 5.0  # m/s, maximum vertical speed
    descent_rate: float = 5.0  # m/s, maximum vertical descent speed
    
    def __post_init__(self):
        """Calculate derived parameters if not provided."""
        if self.max_flight_time is None:
            # Calculate from battery capacity and power consumption
            if self.power_consumption > 0:
                self.max_flight_time = (self.battery_capacity / self.power_consumption) * 3600  # seconds
            else:
                self.max_flight_time = 3600  # default 1 hour
        
        if self.max_range is None:
            # Calculate from max flight time and speed
            self.max_range = self.max_speed * self.max_flight_time
        
        # Validate constraints
        if self.min_altitude >= self.max_altitude:
            raise ValueError(f"min_altitude ({self.min_altitude}) must be less than max_altitude ({self.max_altitude})")
        if self.max_speed <= 0:
            raise ValueError(f"max_speed must be positive, got {self.max_speed}")
        if self.battery_capacity <= 0:
            raise ValueError(f"battery_capacity must be positive, got {self.battery_capacity}")
    
    def can_reach(self, distance: float, altitude_change: float = 0.0) -> bool:
        """Check if drone can reach a point given distance and altitude change."""
        # Check range
        if distance > self.max_range:
            return False
        
        # Check altitude constraints
        # This is a simplified check - actual route planning will be more complex
        return True
    
    def estimate_flight_time(self, distance: float, altitude_change: float = 0.0) -> float:
        """Estimate flight time for given distance and altitude change."""
        horizontal_time = distance / self.max_speed if self.max_speed > 0 else 0
        vertical_time = abs(altitude_change) / self.climb_rate if altitude_change != 0 else 0
        return max(horizontal_time, vertical_time)
    
    def estimate_energy_consumption(self, distance: float, altitude_change: float = 0.0) -> float:
        """Estimate energy consumption in Wh."""
        flight_time = self.estimate_flight_time(distance, altitude_change)
        return (self.power_consumption * flight_time) / 3600  # Convert to Wh
    
    def to_dict(self) -> dict:
        """Convert drone to dictionary."""
        return {
            "name": self.name,
            "max_speed": self.max_speed,
            "max_altitude": self.max_altitude,
            "min_altitude": self.min_altitude,
            "battery_capacity": self.battery_capacity,
            "power_consumption": self.power_consumption,
            "max_flight_time": self.max_flight_time,
            "max_range": self.max_range,
            "turn_radius": self.turn_radius,
            "climb_rate": self.climb_rate,
            "descent_rate": self.descent_rate
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> "Drone":
        """Create drone from dictionary."""
        return cls(
            name=data["name"],
            max_speed=data["max_speed"],
            max_altitude=data["max_altitude"],
            min_altitude=data["min_altitude"],
            battery_capacity=data["battery_capacity"],
            power_consumption=data["power_consumption"],
            max_flight_time=data.get("max_flight_time"),
            max_range=data.get("max_range"),
            turn_radius=data.get("turn_radius", 50.0),
            climb_rate=data.get("climb_rate", 5.0),
            descent_rate=data.get("descent_rate", 5.0)
        )

