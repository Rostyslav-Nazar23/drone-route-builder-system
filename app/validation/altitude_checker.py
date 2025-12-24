"""Altitude constraint checker."""
from typing import List, Dict, Optional
from app.domain.route import Route
from app.domain.drone import Drone
from app.domain.constraints import MissionConstraints


class AltitudeChecker:
    """Checks altitude constraints."""
    
    def check_route(self, route: Route, drone: Drone,
                   constraints: Optional[MissionConstraints] = None) -> List[Dict]:
        """Check altitude constraints.
        
        Args:
            route: Route to check
            drone: Drone capabilities
            constraints: Mission constraints
        
        Returns:
            List of violation dictionaries
        """
        violations = []
        
        if not route.waypoints:
            return violations
        
        for idx, waypoint in enumerate(route.waypoints):
            # Check drone altitude limits
            if waypoint.altitude < drone.min_altitude:
                violations.append({
                    "message": f"Waypoint {idx} altitude {waypoint.altitude}m is below drone minimum {drone.min_altitude}m",
                    "waypoint_index": idx
                })
            
            if waypoint.altitude > drone.max_altitude:
                violations.append({
                    "message": f"Waypoint {idx} altitude {waypoint.altitude}m is above drone maximum {drone.max_altitude}m",
                    "waypoint_index": idx
                })
            
            # Check mission altitude limits
            if constraints:
                if constraints.min_altitude is not None and waypoint.altitude < constraints.min_altitude:
                    violations.append({
                        "message": f"Waypoint {idx} altitude {waypoint.altitude}m is below mission minimum {constraints.min_altitude}m",
                        "waypoint_index": idx
                    })
                
                if constraints.max_altitude is not None and waypoint.altitude > constraints.max_altitude:
                    violations.append({
                        "message": f"Waypoint {idx} altitude {waypoint.altitude}m is above mission maximum {constraints.max_altitude}m",
                        "waypoint_index": idx
                    })
            
            # Check climb/descent rates
            if idx > 0:
                prev_waypoint = route.waypoints[idx - 1]
                altitude_change = waypoint.altitude - prev_waypoint.altitude
                distance = route._haversine_distance(
                    prev_waypoint.latitude, prev_waypoint.longitude,
                    waypoint.latitude, waypoint.longitude
                )
                
                if distance > 0:
                    # Calculate required climb/descent rate
                    time = distance / drone.max_speed
                    required_rate = abs(altitude_change) / time if time > 0 else 0
                    
                    if altitude_change > 0 and required_rate > drone.climb_rate:
                        violations.append({
                            "message": f"Waypoint {idx} requires climb rate {required_rate:.2f}m/s, exceeds maximum {drone.climb_rate}m/s",
                            "waypoint_index": idx
                        })
                    
                    if altitude_change < 0 and required_rate > drone.descent_rate:
                        violations.append({
                            "message": f"Waypoint {idx} requires descent rate {required_rate:.2f}m/s, exceeds maximum {drone.descent_rate}m/s",
                            "waypoint_index": idx
                        })
        
        return violations

