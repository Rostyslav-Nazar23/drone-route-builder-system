"""Waypoint domain model."""
from dataclasses import dataclass
from typing import Optional


@dataclass
class Waypoint:
    """Represents a waypoint in 3D space."""
    latitude: float
    longitude: float
    altitude: float  # in meters
    name: Optional[str] = None
    waypoint_type: str = "target"  # target, depot, intermediate
    
    def __post_init__(self):
        """Validate waypoint coordinates."""
        if not -90 <= self.latitude <= 90:
            raise ValueError(f"Latitude must be between -90 and 90, got {self.latitude}")
        if not -180 <= self.longitude <= 180:
            raise ValueError(f"Longitude must be between -180 and 180, got {self.longitude}")
        if self.altitude < 0:
            raise ValueError(f"Altitude must be non-negative, got {self.altitude}")
    
    def to_dict(self) -> dict:
        """Convert waypoint to dictionary."""
        return {
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
            "name": self.name,
            "type": self.waypoint_type
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> "Waypoint":
        """Create waypoint from dictionary."""
        return cls(
            latitude=data["latitude"],
            longitude=data["longitude"],
            altitude=data.get("altitude", 0.0),
            name=data.get("name"),
            waypoint_type=data.get("type", "target")
        )

