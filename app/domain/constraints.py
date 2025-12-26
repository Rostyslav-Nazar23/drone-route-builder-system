"""Constraints domain model."""
from dataclasses import dataclass, field
from typing import List, Optional
from shapely.geometry import Polygon, Point
from shapely.geometry.base import BaseGeometry


@dataclass
class NoFlyZone:
    """Represents a no-fly zone."""
    geometry: BaseGeometry  # Shapely geometry (Polygon, MultiPolygon, etc.)
    min_altitude: float = 0.0  # meters
    max_altitude: float = 1000.0  # meters
    name: Optional[str] = None
    
    def contains(self, point: Point, altitude: float) -> bool:
        """Check if a point at given altitude is within the no-fly zone."""
        if not (self.min_altitude <= altitude <= self.max_altitude):
            return False
        return self.geometry.contains(point) or self.geometry.touches(point)
    
    def intersects(self, geometry: BaseGeometry) -> bool:
        """Check if geometry intersects with the no-fly zone."""
        return self.geometry.intersects(geometry)


@dataclass
class MissionConstraints:
    """Constraints for a mission."""
    no_fly_zones: List[NoFlyZone] = field(default_factory=list)
    max_altitude: Optional[float] = None  # meters, global max altitude
    min_altitude: Optional[float] = None  # meters, global min altitude
    max_distance: Optional[float] = None  # meters, max distance from start
    max_flight_time: Optional[float] = None  # seconds
    require_return_to_depot: bool = True  # Deprecated: use Mission.finish_point_type instead
    
    def add_no_fly_zone(self, zone: NoFlyZone):
        """Add a no-fly zone."""
        self.no_fly_zones.append(zone)
    
    def check_point(self, latitude: float, longitude: float, altitude: float, 
                   is_ground_point: bool = False) -> tuple[bool, Optional[str]]:
        """Check if a point violates constraints. Returns (is_valid, error_message).
        
        Args:
            latitude: Point latitude
            longitude: Point longitude
            altitude: Point altitude
            is_ground_point: If True, skip minimum altitude checks (for depot/finish points)
        """
        point = Point(longitude, latitude)
        
        # Check altitude constraints
        # Skip minimum altitude check for ground points (depot/finish)
        if not is_ground_point and self.min_altitude is not None and altitude < self.min_altitude:
            return False, f"Altitude {altitude}m is below minimum {self.min_altitude}m"
        # Always check maximum altitude
        if self.max_altitude is not None and altitude > self.max_altitude:
            return False, f"Altitude {altitude}m is above maximum {self.max_altitude}m"
        
        # Check no-fly zones
        for zone in self.no_fly_zones:
            if zone.contains(point, altitude):
                zone_name = zone.name or "unnamed"
                return False, f"Point is in no-fly zone: {zone_name}"
        
        return True, None
    
    def to_dict(self) -> dict:
        """Convert constraints to dictionary."""
        # Note: Shapely geometries need to be serialized to GeoJSON
        from shapely.geometry import mapping
        return {
            "no_fly_zones": [
                {
                    "geometry": mapping(zone.geometry),
                    "min_altitude": zone.min_altitude,
                    "max_altitude": zone.max_altitude,
                    "name": zone.name
                }
                for zone in self.no_fly_zones
            ],
            "max_altitude": self.max_altitude,
            "min_altitude": self.min_altitude,
            "max_distance": self.max_distance,
            "max_flight_time": self.max_flight_time,
            "require_return_to_depot": self.require_return_to_depot
        }

