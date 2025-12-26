"""Cost model for navigation graph edges."""
from typing import Optional, Dict
from app.domain.drone import Drone
from app.domain.constraints import MissionConstraints
from app.weather.weather_provider import WeatherConditions
from app.weather.weather_manager import WeatherManager
from shapely.geometry import LineString, Point
import math


class CostModel:
    """Model for calculating edge costs in navigation graph."""
    
    def __init__(self, drone: Drone, constraints: Optional[MissionConstraints] = None,
                 weather_data: Optional[Dict[tuple[float, float], WeatherConditions]] = None,
                 weather_manager: Optional[WeatherManager] = None):
        """Initialize cost model.
        
        Args:
            drone: Drone capabilities
            constraints: Mission constraints (optional)
            weather_data: Dictionary mapping (lat, lon) to WeatherConditions (optional, initial cache)
            weather_manager: WeatherManager instance for dynamic weather fetching (optional)
        """
        self.drone = drone
        self.constraints = constraints or MissionConstraints()
        self.weather_data = weather_data or {}
        self.weather_manager = weather_manager
        
        # Update weather_data from weather_manager if available
        if weather_manager:
            self.weather_data = weather_manager.get_all_weather_data()
    
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
                      lat2: float, lon2: float, alt2: float,
                      current_speed: float = 0.0) -> float:
        """Calculate cost for traversing from point 1 to point 2.
        
        Cost is based on:
        - Distance (primary factor)
        - Energy consumption
        - Time (including inertia/acceleration/deceleration)
        - Penalties for altitude changes
        - Weather conditions (wind, precipitation)
        
        Args:
            lat1, lon1, alt1: Start point coordinates
            lat2, lon2, alt2: End point coordinates
            current_speed: Current speed at start point (m/s), for inertia calculation
        
        Returns:
            Cost value (lower is better)
        """
        distance = self.calculate_distance(lat1, lon1, alt1, lat2, lon2, alt2)
        horizontal_distance = self._haversine_distance(lat1, lon1, lat2, lon2)
        
        # Base cost is distance
        cost = distance
        
        # Dubins Airplane model: Check kinematic constraints
        altitude_change = alt2 - alt1
        altitude_change_abs = abs(altitude_change)
        
        # Calculate time for horizontal movement (assuming average speed)
        avg_speed = self.drone.max_speed * 0.7  # Use 70% of max speed for planning
        time_horizontal = horizontal_distance / avg_speed if avg_speed > 0 else 0
        
        # Check climb/descent rate constraints (Dubins Airplane)
        if time_horizontal > 0:
            required_climb_rate = altitude_change_abs / time_horizontal
            
            if altitude_change > 0:  # Climbing
                if required_climb_rate > self.drone.climb_rate:
                    # Cannot climb fast enough - add large penalty
                    cost += 10000 * (required_climb_rate / self.drone.climb_rate - 1)
                else:
                    # Penalty for climbing (more energy intensive)
                    cost += altitude_change_abs * 2.0
            elif altitude_change < 0:  # Descending
                if required_climb_rate > self.drone.descent_rate:
                    # Cannot descend fast enough - add large penalty
                    cost += 10000 * (required_climb_rate / self.drone.descent_rate - 1)
                else:
                    # Penalty for descending (less than climbing, but still costs energy)
                    cost += altitude_change_abs * 1.2
        
        # Turn radius constraint (Dubins Airplane) - simplified check
        # For waypoint graph, we estimate if the turn is feasible
        # This is a simplified check - full Dubins would check all path segments
        # The turn radius constraint is primarily enforced during pathfinding
        # by checking if waypoints are too close together for the required turn
        if horizontal_distance > 0:
            # Estimate minimum turn radius based on speed
            # Simplified: minimum distance for a 90-degree turn
            min_turn_distance = self.drone.turn_radius * math.pi / 2  # Quarter circle
            if horizontal_distance < min_turn_distance:
                # Very short segment - might require sharp turn, add small penalty
                cost += (min_turn_distance - horizontal_distance) * 0.1
        
        # Calculate heading for wind effect
        heading = self._calculate_heading(lat1, lon1, lat2, lon2)
        avg_altitude = (alt1 + alt2) / 2.0
        
        # Get weather conditions and apply wind effects
        weather_penalty = 0.0
        energy_multiplier = 1.0
        effective_max_speed = self.drone.max_speed
        
        # Try to get weather for both points (use midpoint if available)
        mid_lat = (lat1 + lat2) / 2.0
        mid_lon = (lon1 + lon2) / 2.0
        avg_altitude = (alt1 + alt2) / 2.0
        
        weather = self._get_weather_for_point(mid_lat, mid_lon, avg_altitude)
        if weather:
            # Calculate effective wind (headwind/tailwind)
            effective_wind = weather.get_effective_wind_speed(heading, avg_altitude)
            
            # Wind effect on speed (headwind reduces effective speed, tailwind increases it)
            effective_max_speed = max(0.1 * self.drone.max_speed, 
                                    min(self.drone.max_speed * 1.2, 
                                        self.drone.max_speed - effective_wind * 0.5))
            
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
        
        # Calculate time with inertia (acceleration/deceleration)
        # This affects both time cost and energy consumption
        acceleration = self.drone.max_speed / 5.0  # Reach max speed in 5 seconds
        deceleration = self.drone.max_speed / 5.0  # Decelerate in 5 seconds
        
        if horizontal_distance > 0:
            # Time to accelerate to effective max speed
            accel_time = max(0, (effective_max_speed - current_speed) / acceleration) if acceleration > 0 else 0
            accel_distance = current_speed * accel_time + 0.5 * acceleration * accel_time ** 2
            
            # Time to decelerate (assume we need to slow down at end)
            decel_time = effective_max_speed / deceleration if deceleration > 0 else 0
            decel_distance = effective_max_speed * decel_time - 0.5 * deceleration * decel_time ** 2
            
            # Cruise distance
            cruise_distance = max(0, horizontal_distance - accel_distance - decel_distance)
            cruise_time = cruise_distance / effective_max_speed if effective_max_speed > 0 else 0
            
            total_time = accel_time + cruise_time + decel_time
        else:
            total_time = 0
        
        # Add time cost (time is valuable, convert to distance equivalent)
        # 1 second ≈ 10 meters of cost (adjustable)
        time_cost_factor = 10.0
        cost += total_time * time_cost_factor
        
        # Add energy cost (normalized, with weather adjustment and speed effects)
        base_energy_cost = self.drone.estimate_energy_consumption(
            horizontal_distance,
            alt2 - alt1
        )
        
        # High speed energy multiplier (quadratic relationship)
        speed_factor = (effective_max_speed / self.drone.max_speed) ** 2
        speed_energy_multiplier = 1.0 + 0.5 * (speed_factor - 1.0)  # 50% more energy at max speed
        
        adjusted_energy_cost = base_energy_cost * energy_multiplier * speed_energy_multiplier
        
        # Normalize energy cost (assume 100 Wh is max reasonable cost)
        cost += (adjusted_energy_cost / 100.0) * distance * 0.1
        
        # Add weather penalty
        cost += weather_penalty
        
        return cost
    
    def _get_weather_for_point(self, latitude: float, longitude: float, altitude: float = 0.0) -> Optional[WeatherConditions]:
        """Get weather conditions for a point (find nearest available or fetch if too far).
        
        Uses WeatherManager if available for optimized fetching, otherwise falls back to cache lookup.
        
        Args:
            latitude: Point latitude
            longitude: Point longitude
            altitude: Point altitude (for fetching new weather)
        
        Returns:
            WeatherConditions or None
        """
        # Use WeatherManager if available (preferred method)
        if self.weather_manager:
            weather = self.weather_manager.get_weather_for_point(latitude, longitude, altitude)
            if weather:
                # Update local cache
                key = (latitude, longitude)
                self.weather_data[key] = weather
                return weather
        
        # Fallback to cache lookup if no weather_manager
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
        
        # Minimum distance threshold: if > 5km, try to fetch new weather
        MIN_WEATHER_DISTANCE = 5000.0  # 5km in meters
        
        if closest_weather and min_distance < MIN_WEATHER_DISTANCE:
            return closest_weather
        
        # If weather_manager is not available and point is too far, return None
        # (We don't want to fetch here if weather_manager exists, as it should handle it)
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
                     lat2: float, lon2: float, alt2: float,
                     is_start_ground: bool = False,
                     is_end_ground: bool = False) -> tuple[bool, Optional[str]]:
        """Check if edge is valid (doesn't violate constraints).
        
        Args:
            lat1, lon1, alt1: Start point coordinates
            lat2, lon2, alt2: End point coordinates
            is_start_ground: If True, start point is depot/finish (skip min altitude check)
            is_end_ground: If True, end point is depot/finish (skip min altitude check)
        
        Returns:
            (is_valid, error_message)
        """
        # Check start point
        is_valid, error = self.constraints.check_point(lat1, lon1, alt1, is_ground_point=is_start_ground)
        if not is_valid:
            return False, f"Start point: {error}"
        
        # Check end point
        is_valid, error = self.constraints.check_point(lat2, lon2, alt2, is_ground_point=is_end_ground)
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
        # Use 2D LineString for intersection check (Shapely doesn't support 3D LineString intersection with 2D geometry)
        line_2d = LineString([
            (lon1, lat1),
            (lon2, lat2)
        ])
        
        for zone in self.constraints.no_fly_zones:
            # Check if 2D line intersects zone geometry
            if zone.geometry.intersects(line_2d):
                # Check altitude range along the line
                # The line segment intersects the zone if altitude ranges overlap
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

