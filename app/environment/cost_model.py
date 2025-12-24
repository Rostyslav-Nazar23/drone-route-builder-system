"""Cost model for navigation graph edges."""
from typing import Optional, Dict
from app.domain.drone import Drone
from app.domain.constraints import MissionConstraints
from app.weather.weather_provider import WeatherConditions
from shapely.geometry import LineString, Point
import math


class CostModel:
    """Model for calculating edge costs in navigation graph."""
    
    def __init__(self, drone: Drone, constraints: Optional[MissionConstraints] = None,
                 weather_data: Optional[Dict[tuple[float, float], WeatherConditions]] = None):
        """Initialize cost model.
        
        Args:
            drone: Drone capabilities
            constraints: Mission constraints (optional)
            weather_data: Dictionary mapping (lat, lon) to WeatherConditions (optional)
        """
        self.drone = drone
        self.constraints = constraints or MissionConstraints()
        self.weather_data = weather_data or {}
    
    def calculate_distance(self, lat1: float, lon1: float, alt1: float,
                          lat2: float, lon2: float, alt2: float) -> float:
        """Calculate 3D distance between two points."""
        # Horizontal distance (Haversine)
        horizontal_dist = self._haversine_distance(lat1, lon1, lat2, lon2)
        
        # Vertical distance
        vertical_dist = abs(alt2 - alt1)
        
        # 3D Euclidean distance
        return (horizontal_dist ** 2 + vertical_dist ** 2) ** 0.5
    
    def calculate_cost(self, lat1: float, lon1: float, alt1: float,
                      lat2: float, lon2: float, alt2: float) -> float:
        """Calculate cost for traversing from point 1 to point 2.
        
        Cost is based on:
        - Distance (primary factor)
        - Energy consumption
        - Time
        - Penalties for altitude changes
        - Weather conditions (wind, precipitation)
        
        Returns:
            Cost value (lower is better)
        """
        distance = self.calculate_distance(lat1, lon1, alt1, lat2, lon2, alt2)
        horizontal_distance = self._haversine_distance(lat1, lon1, lat2, lon2)
        
        # Base cost is distance
        cost = distance
        
        # Add penalty for altitude changes (climbing/descending is more expensive)
        altitude_change = abs(alt2 - alt1)
        if altitude_change > 0:
            # Penalty proportional to altitude change
            cost += altitude_change * 1.5
        
        # Calculate heading for wind effect
        heading = self._calculate_heading(lat1, lon1, lat2, lon2)
        avg_altitude = (alt1 + alt2) / 2.0
        
        # Get weather conditions and apply wind effects
        weather_penalty = 0.0
        energy_multiplier = 1.0
        
        # Try to get weather for both points (use midpoint if available)
        mid_lat = (lat1 + lat2) / 2.0
        mid_lon = (lon1 + lon2) / 2.0
        
        weather = self._get_weather_for_point(mid_lat, mid_lon)
        if weather:
            # Calculate effective wind (headwind/tailwind)
            effective_wind = weather.get_effective_wind_speed(heading, avg_altitude)
            
            # Wind effect on energy consumption
            # Headwind increases energy, tailwind decreases
            # Formula: energy_multiplier = 1 + (wind_effect / max_speed) * factor
            wind_factor = 0.3  # 30% impact of wind on energy
            wind_effect_ratio = effective_wind / max(self.drone.max_speed, 1.0)
            energy_multiplier = 1.0 + (wind_effect_ratio * wind_factor)
            
            # Add penalty for strong headwinds
            if effective_wind > 5.0:  # Strong headwind
                weather_penalty += effective_wind * 10.0
            
            # Penalty for precipitation (reduces visibility, increases risk)
            if weather.precipitation > 0:
                weather_penalty += weather.precipitation * 50.0
            
            # Penalty for high cloud cover (reduces visibility)
            if weather.cloud_cover > 80:
                weather_penalty += (weather.cloud_cover - 80) * 2.0
        
        # Add energy cost (normalized, with weather adjustment)
        base_energy_cost = self.drone.estimate_energy_consumption(
            horizontal_distance,
            alt2 - alt1
        )
        adjusted_energy_cost = base_energy_cost * energy_multiplier
        
        # Normalize energy cost (assume 100 Wh is max reasonable cost)
        cost += (adjusted_energy_cost / 100.0) * distance * 0.1
        
        # Add weather penalty
        cost += weather_penalty
        
        return cost
    
    def _get_weather_for_point(self, latitude: float, longitude: float) -> Optional[WeatherConditions]:
        """Get weather conditions for a point (find nearest available)."""
        if not self.weather_data:
            return None
        
        # Find closest weather data point
        min_distance = float('inf')
        closest_weather = None
        
        for (lat, lon), weather in self.weather_data.items():
            distance = self._haversine_distance(latitude, longitude, lat, lon)
            if distance < min_distance:
                min_distance = distance
                closest_weather = weather
        
        # Use weather if within reasonable distance (10km)
        if closest_weather and min_distance < 10000:
            return closest_weather
        
        return None
    
    @staticmethod
    def _calculate_heading(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate heading from point 1 to point 2 in degrees (0-360, 0 = North)."""
        from math import radians, degrees, atan2, sin, cos
        
        lat1_rad = radians(lat1)
        lat2_rad = radians(lat2)
        delta_lon = radians(lon2 - lon1)
        
        y = sin(delta_lon) * cos(lat2_rad)
        x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon)
        
        heading = degrees(atan2(y, x))
        heading = (heading + 360) % 360  # Normalize to 0-360
        
        return heading
    
    def is_valid_edge(self, lat1: float, lon1: float, alt1: float,
                     lat2: float, lon2: float, alt2: float) -> tuple[bool, Optional[str]]:
        """Check if edge is valid (doesn't violate constraints).
        
        Returns:
            (is_valid, error_message)
        """
        # Check start point
        is_valid, error = self.constraints.check_point(lat1, lon1, alt1)
        if not is_valid:
            return False, f"Start point: {error}"
        
        # Check end point
        is_valid, error = self.constraints.check_point(lat2, lon2, alt2)
        if not is_valid:
            return False, f"End point: {error}"
        
        # Check weather conditions if available
        mid_lat = (lat1 + lat2) / 2.0
        mid_lon = (lon1 + lon2) / 2.0
        avg_altitude = (alt1 + alt2) / 2.0
        
        weather = self._get_weather_for_point(mid_lat, mid_lon)
        if weather:
            is_safe, error_msg = weather.is_safe_for_flight()
            if not is_safe:
                return False, f"Weather conditions: {error_msg}"
        
        # Check if line segment intersects no-fly zones
        line = LineString([
            Point(lon1, lat1, alt1),
            Point(lon2, lat2, alt2)
        ])
        
        for zone in self.constraints.no_fly_zones:
            # Check if line intersects zone geometry
            if zone.geometry.intersects(line):
                # Check altitude range along the line
                # Simplified: check if any point in the altitude range intersects
                min_alt = min(alt1, alt2)
                max_alt = max(alt1, alt2)
                
                if (zone.min_altitude <= max_alt and zone.max_altitude >= min_alt):
                    zone_name = zone.name or "unnamed"
                    return False, f"Edge intersects no-fly zone: {zone_name}"
        
        return True, None
    
    @staticmethod
    def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate horizontal distance using Haversine formula."""
        from math import radians, sin, cos, sqrt, atan2
        
        R = 6371000  # Earth radius in meters
        
        lat1_rad = radians(lat1)
        lat2_rad = radians(lat2)
        delta_lat = radians(lat2 - lat1)
        delta_lon = radians(lon2 - lon1)
        
        a = sin(delta_lat / 2) ** 2 + cos(lat1_rad) * cos(lat2_rad) * sin(delta_lon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        
        return R * c

