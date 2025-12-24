"""Route domain model."""
from dataclasses import dataclass, field
from typing import List, Optional
from .waypoint import Waypoint


@dataclass
class RouteMetrics:
    """Metrics for a route."""
    total_distance: float = 0.0  # meters
    total_time: float = 0.0  # seconds
    total_energy: float = 0.0  # Wh
    max_altitude: float = 0.0
    min_altitude: float = 0.0
    waypoint_count: int = 0
    
    def to_dict(self) -> dict:
        """Convert metrics to dictionary."""
        return {
            "total_distance": self.total_distance,
            "total_time": self.total_time,
            "total_energy": self.total_energy,
            "max_altitude": self.max_altitude,
            "min_altitude": self.min_altitude,
            "waypoint_count": self.waypoint_count
        }


@dataclass
class Route:
    """Represents a planned route for a drone."""
    waypoints: List[Waypoint] = field(default_factory=list)
    drone_name: Optional[str] = None
    metrics: Optional[RouteMetrics] = None
    validation_result: Optional[dict] = None
    
    def add_waypoint(self, waypoint: Waypoint):
        """Add a waypoint to the route."""
        self.waypoints.append(waypoint)
    
    def calculate_metrics(self, drone) -> RouteMetrics:
        """Calculate route metrics based on drone capabilities."""
        if not self.waypoints:
            return RouteMetrics()
        
        total_distance = 0.0
        total_energy = 0.0
        altitudes = [wp.altitude for wp in self.waypoints]
        
        # Calculate distances between consecutive waypoints
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            distance = self._haversine_distance(
                wp1.latitude, wp1.longitude,
                wp2.latitude, wp2.longitude
            )
            altitude_change = wp2.altitude - wp1.altitude
            total_distance += distance
            total_energy += drone.estimate_energy_consumption(distance, altitude_change)
        
        total_time = drone.estimate_flight_time(total_distance, 0)
        
        self.metrics = RouteMetrics(
            total_distance=total_distance,
            total_time=total_time,
            total_energy=total_energy,
            max_altitude=max(altitudes),
            min_altitude=min(altitudes),
            waypoint_count=len(self.waypoints)
        )
        return self.metrics
    
    @staticmethod
    def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two points using Haversine formula."""
        from math import radians, sin, cos, sqrt, atan2
        
        R = 6371000  # Earth radius in meters
        
        lat1_rad = radians(lat1)
        lat2_rad = radians(lat2)
        delta_lat = radians(lat2 - lat1)
        delta_lon = radians(lon2 - lon1)
        
        a = sin(delta_lat / 2) ** 2 + cos(lat1_rad) * cos(lat2_rad) * sin(delta_lon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        
        return R * c
    
    def to_dict(self) -> dict:
        """Convert route to dictionary."""
        return {
            "waypoints": [wp.to_dict() for wp in self.waypoints],
            "drone_name": self.drone_name,
            "metrics": self.metrics.to_dict() if self.metrics else None
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> "Route":
        """Create route from dictionary."""
        waypoints = [Waypoint.from_dict(wp) for wp in data.get("waypoints", [])]
        metrics = None
        if data.get("metrics"):
            metrics = RouteMetrics(**data["metrics"])
        
        return cls(
            waypoints=waypoints,
            drone_name=data.get("drone_name"),
            metrics=metrics
        )

