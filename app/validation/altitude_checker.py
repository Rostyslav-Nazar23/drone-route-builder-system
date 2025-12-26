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
            # Skip minimum altitude checks for:
            # - depot and finish points (they're on ground)
            # - landing_segment waypoints (gradual descent to finish, may go below min alt)
            # - landing_approach waypoints (vertical landing approach, already at min alt)
            is_ground_point = waypoint.waypoint_type in ["depot", "finish"]
            is_landing_segment = waypoint.waypoint_type == "landing_segment"
            is_landing_approach = waypoint.waypoint_type == "landing_approach"
            
            # Check drone altitude limits
            # Only check minimum for flight waypoints (not depot/finish/landing_segment/landing_approach)
            if not is_ground_point and not is_landing_segment and not is_landing_approach and waypoint.altitude < drone.min_altitude:
                violations.append({
                    "message": f"Waypoint {idx} altitude {waypoint.altitude}m is below drone minimum {drone.min_altitude}m",
                    "waypoint_index": idx
                })
            
            # Always check maximum altitude (even for ground points)
            if waypoint.altitude > drone.max_altitude:
                violations.append({
                    "message": f"Waypoint {idx} altitude {waypoint.altitude}m is above drone maximum {drone.max_altitude}m",
                    "waypoint_index": idx
                })
            
            # Check mission altitude limits
            if constraints:
                # Only check minimum for flight waypoints (not depot/finish/landing_segment/landing_approach)
                if not is_ground_point and not is_landing_segment and not is_landing_approach and constraints.min_altitude is not None and waypoint.altitude < constraints.min_altitude:
                    violations.append({
                        "message": f"Waypoint {idx} altitude {waypoint.altitude}m is below mission minimum {constraints.min_altitude}m",
                        "waypoint_index": idx
                    })
                
                # Always check maximum altitude
                if constraints.max_altitude is not None and waypoint.altitude > constraints.max_altitude:
                    violations.append({
                        "message": f"Waypoint {idx} altitude {waypoint.altitude}m is above mission maximum {constraints.max_altitude}m",
                        "waypoint_index": idx
                    })
            
            # Check climb/descent rates
            # Skip descent rate check for landing_segment and landing_approach waypoints in vertical landing
            # These waypoints are part of the landing sequence and may have rapid altitude changes
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
                    
                    # Check climb rate (always check)
                    if altitude_change > 0 and required_rate > drone.climb_rate:
                        violations.append({
                            "message": f"Waypoint {idx} requires climb rate {required_rate:.2f}m/s, exceeds maximum {drone.climb_rate}m/s",
                            "waypoint_index": idx
                        })
                    
                    # Check descent rate - skip for landing segments/approach (vertical landing allows rapid descent)
                    # Also skip if previous waypoint is landing_segment or landing_approach
                    skip_descent_check = (
                        waypoint.waypoint_type in ["landing_segment", "landing_approach", "finish", "depot"] or
                        prev_waypoint.waypoint_type in ["landing_segment", "landing_approach"]
                    )
                    
                    if altitude_change < 0 and required_rate > drone.descent_rate and not skip_descent_check:
                        violations.append({
                            "message": f"Waypoint {idx} requires descent rate {required_rate:.2f}m/s, exceeds maximum {drone.descent_rate}m/s",
                            "waypoint_index": idx
                        })
        
        return violations

