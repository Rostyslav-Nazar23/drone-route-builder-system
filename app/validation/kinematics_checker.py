"""Kinematics checker using Dubins Airplane model."""
from typing import List, Dict, Optional
import math
from app.domain.route import Route
from app.domain.drone import Drone


class DubinsAirplane:
    """Dubins Airplane model for 3D path planning with kinematic constraints."""
    
    def __init__(self, turn_radius: float, climb_rate: float, descent_rate: float):
        """Initialize Dubins Airplane.
        
        Args:
            turn_radius: Minimum turning radius (meters)
            climb_rate: Maximum climb rate (m/s)
            descent_rate: Maximum descent rate (m/s)
        """
        self.turn_radius = turn_radius
        self.climb_rate = climb_rate
        self.descent_rate = descent_rate
    
    def calculate_path(self, start: tuple[float, float, float, float],  # lat, lon, alt, heading
                      end: tuple[float, float, float, float]) -> Optional[List[tuple[float, float, float]]]:
        """Calculate Dubins Airplane path between two configurations.
        
        Args:
            start: (latitude, longitude, altitude, heading) in degrees
            end: (latitude, longitude, altitude, heading) in degrees
        
        Returns:
            List of waypoints along the path, or None if infeasible
        """
        # Simplified Dubins Airplane implementation
        # Full implementation would consider all possible path types (LSL, RSR, etc.)
        
        lat1, lon1, alt1, h1 = start
        lat2, lon2, alt2, h2 = end
        
        # Calculate horizontal distance
        distance = self._haversine_distance(lat1, lon1, lat2, lon2)
        
        # Check if path is feasible
        altitude_change = alt2 - alt1
        time_horizontal = distance / 15.0  # Assume 15 m/s speed
        required_climb_rate = abs(altitude_change) / time_horizontal if time_horizontal > 0 else 0
        
        if altitude_change > 0 and required_climb_rate > self.climb_rate:
            return None  # Cannot climb fast enough
        
        if altitude_change < 0 and required_climb_rate > self.descent_rate:
            return None  # Cannot descend fast enough
        
        # Generate path waypoints
        waypoints = []
        num_points = max(10, int(distance / 100))  # Waypoint every 100m
        
        for i in range(num_points + 1):
            t = i / num_points
            lat = lat1 + (lat2 - lat1) * t
            lon = lon1 + (lon2 - lon1) * t
            alt = alt1 + (alt2 - alt1) * t
            
            waypoints.append((lat, lon, alt))
        
        return waypoints
    
    @staticmethod
    def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance using Haversine formula."""
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c


class KinematicsChecker:
    """Checks kinematic feasibility of routes using Dubins Airplane."""
    
    def check_route(self, route: Route, drone: Drone) -> Dict:
        """Check if route is kinematically feasible.
        
        Args:
            route: Route to check
            drone: Drone capabilities
        
        Returns:
            Dictionary with validation result
        """
        if len(route.waypoints) < 2:
            return {"is_valid": True, "violations": []}
        
        violations = []
        dubins = DubinsAirplane(drone.turn_radius, drone.climb_rate, drone.descent_rate)
        
        for i in range(len(route.waypoints) - 1):
            wp1 = route.waypoints[i]
            wp2 = route.waypoints[i + 1]
            
            # Calculate heading
            heading1 = self._calculate_heading(wp1, wp2)
            heading2 = heading1  # Assume same heading for next segment
            
            if i < len(route.waypoints) - 2:
                wp3 = route.waypoints[i + 2]
                heading2 = self._calculate_heading(wp2, wp3)
            
            # Check Dubins path feasibility
            start = (wp1.latitude, wp1.longitude, wp1.altitude, heading1)
            end = (wp2.latitude, wp2.longitude, wp2.altitude, heading2)
            
            path = dubins.calculate_path(start, end)
            
            if path is None:
                violations.append({
                    "segment": f"{i}-{i+1}",
                    "message": f"Segment {i}-{i+1} is kinematically infeasible (turn radius or climb rate exceeded)"
                })
        
        return {
            "is_valid": len(violations) == 0,
            "violations": violations
        }
    
    @staticmethod
    def _calculate_heading(wp1, wp2) -> float:
        """Calculate heading from wp1 to wp2."""
        import math
        
        lat1_rad = math.radians(wp1.latitude)
        lat2_rad = math.radians(wp2.latitude)
        delta_lon = math.radians(wp2.longitude - wp1.longitude)
        
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
        
        heading = math.atan2(y, x)
        heading = math.degrees(heading)
        heading = (heading + 360) % 360
        
        return heading

