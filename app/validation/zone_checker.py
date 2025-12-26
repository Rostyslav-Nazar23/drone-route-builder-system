"""No-fly zone checker."""
from typing import List, Dict
from shapely.geometry import LineString, Point
from app.domain.route import Route
from app.domain.constraints import MissionConstraints


class ZoneChecker:
    """Checks routes against no-fly zones."""
    
    def check_route(self, route: Route, constraints: MissionConstraints) -> List[Dict]:
        """Check if route intersects no-fly zones.
        
        Args:
            route: Route to check
            constraints: Mission constraints
        
        Returns:
            List of violation dictionaries
        """
        violations = []
        
        if not route.waypoints or not constraints.no_fly_zones:
            return violations
        
        # Check each waypoint
        for idx, waypoint in enumerate(route.waypoints):
            point = Point(waypoint.longitude, waypoint.latitude)
            
            for zone in constraints.no_fly_zones:
                if zone.contains(point, waypoint.altitude):
                    zone_name = zone.name or "unnamed"
                    violations.append({
                        "message": f"Waypoint {idx} is in no-fly zone: {zone_name}",
                        "waypoint_index": idx
                    })
        
        # Check route segments
        for idx in range(len(route.waypoints) - 1):
            wp1 = route.waypoints[idx]
            wp2 = route.waypoints[idx + 1]
            
            # Create 2D line segment for intersection check (Shapely doesn't support 3D LineString intersection with 2D geometry)
            line_2d = LineString([
                (wp1.longitude, wp1.latitude),
                (wp2.longitude, wp2.latitude)
            ])
            
            for zone in constraints.no_fly_zones:
                # Check if 2D line intersects zone geometry
                if zone.geometry.intersects(line_2d):
                    # Check altitude range - the segment intersects if altitude ranges overlap
                    min_alt = min(wp1.altitude, wp2.altitude)
                    max_alt = max(wp1.altitude, wp2.altitude)
                    
                    if zone.min_altitude <= max_alt and zone.max_altitude >= min_alt:
                        zone_name = zone.name or "unnamed"
                        violations.append({
                            "message": f"Route segment {idx}-{idx+1} intersects no-fly zone: {zone_name}",
                            "waypoint_index": idx
                        })
        
        return violations

