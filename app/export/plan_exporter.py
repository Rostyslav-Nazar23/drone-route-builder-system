"""Exporter for .plan file format."""
from typing import List, Optional
from pathlib import Path
import math
from app.domain.mission import Mission
from app.domain.route import Route
from app.domain.drone import Drone


class PlanExporter:
    """Exports routes to .plan file format."""
    
    # MAVLink command constants
    MAV_CMD_NAV_WAYPOINT = 16
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_CMD_NAV_LAND = 21
    MAV_CMD_NAV_LOITER_TIME = 19
    MAV_CMD_NAV_LOITER_UNLIM = 17
    MAV_CMD_NAV_LOITER_TURNS = 18
    
    @staticmethod
    def _calculate_heading(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate heading from point 1 to point 2 in degrees (0-360, 0 = North)."""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlon_rad = math.radians(lon2 - lon1)
        
        y = math.sin(dlon_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)
        heading = math.degrees(math.atan2(y, x))
        if heading < 0:
            heading += 360
        return heading
    
    @staticmethod
    def _calculate_speed_for_segment(wp1, wp2, drone: Optional[Drone], avg_speed: Optional[float]) -> float:
        """Calculate speed for a segment between two waypoints.
        
        Args:
            wp1: First waypoint
            wp2: Second waypoint
            drone: Drone object (optional, for max_speed)
            avg_speed: Average speed from route metrics (optional)
        
        Returns:
            Speed in m/s
        """
        # Use average speed from metrics if available
        if avg_speed and avg_speed > 0:
            return avg_speed
        
        # Otherwise use 70% of max speed as default cruise speed
        if drone and drone.max_speed > 0:
            return drone.max_speed * 0.7
        
        # Default fallback
        return 10.0  # 10 m/s default
    
    @staticmethod
    def _get_command_for_waypoint(waypoint, idx: int, total: int) -> int:
        """Get MAVLink command for a waypoint based on its type and position.
        
        Args:
            waypoint: Waypoint object
            idx: Current waypoint index
            total: Total number of waypoints
        
        Returns:
            MAVLink command code
        """
        wp_type = waypoint.waypoint_type.lower() if waypoint.waypoint_type else ""
        
        # First waypoint is always TAKEOFF
        if idx == 0:
            return PlanExporter.MAV_CMD_NAV_TAKEOFF
        
        # Last waypoint is always LAND
        if idx == total - 1:
            return PlanExporter.MAV_CMD_NAV_LAND
        
        # Check waypoint type
        if "landing" in wp_type or "finish" in wp_type:
            return PlanExporter.MAV_CMD_NAV_LAND
        elif "depot" in wp_type:
            # Depot in middle of route - use waypoint
            return PlanExporter.MAV_CMD_NAV_WAYPOINT
        elif "target" in wp_type:
            # Target points - could use LOITER if needed, but default to WAYPOINT
            return PlanExporter.MAV_CMD_NAV_WAYPOINT
        else:
            # Default to waypoint
            return PlanExporter.MAV_CMD_NAV_WAYPOINT
    
    @staticmethod
    def export_route(route: Route, file_path: str, drone: Optional[Drone] = None, mission: Optional[Mission] = None):
        """Export a single route to .plan file.
        
        Format: INDEX, CURRENT_WP, COORD_FRAME, COMMAND, PARAM1, PARAM2, PARAM3, PARAM4, PARAM5/X, PARAM6/Y, PARAM7/Z, AUTOCONTINUE
        
        Args:
            route: Route to export
            file_path: Output file path
            drone: Drone object (optional, for max_speed and other parameters)
            mission: Mission object (optional, to find drone if not provided)
        """
        # Try to get drone if not provided
        if drone is None and mission is not None and route.drone_name:
            drone = next((d for d in mission.drones if d.name == route.drone_name), None)
        
        lines = []
        
        # Header
        lines.append("QGC WPL 110")
        
        # Get average speed from metrics if available
        avg_speed = route.metrics.avg_speed if route.metrics else None
        
        # Waypoints
        for idx, waypoint in enumerate(route.waypoints):
            # Get command based on waypoint type and position
            command = PlanExporter._get_command_for_waypoint(waypoint, idx, len(route.waypoints))
            
            # Calculate heading (yaw) for this waypoint
            yaw = 0.0  # Default: no yaw change
            if idx < len(route.waypoints) - 1:
                # Calculate heading to next waypoint
                wp1 = waypoint
                wp2 = route.waypoints[idx + 1]
                yaw = PlanExporter._calculate_heading(wp1.latitude, wp1.longitude, wp2.latitude, wp2.longitude)
            elif idx > 0:
                # Last waypoint: use heading from previous waypoint
                wp1 = route.waypoints[idx - 1]
                wp2 = waypoint
                yaw = PlanExporter._calculate_heading(wp1.latitude, wp1.longitude, wp2.latitude, wp2.longitude)
            
            # Calculate speed for this segment
            speed = 0.0  # Default speed
            if idx < len(route.waypoints) - 1:
                speed = PlanExporter._calculate_speed_for_segment(
                    waypoint, route.waypoints[idx + 1], drone, avg_speed
                )
            elif idx > 0:
                # Last waypoint: use speed from previous segment
                speed = PlanExporter._calculate_speed_for_segment(
                    route.waypoints[idx - 1], waypoint, drone, avg_speed
                )
            
            # PARAM1: Hold time (for LOITER) or acceptance radius (for WAYPOINT)
            # For waypoints, use acceptance radius (meters)
            param1 = 5.0  # Default acceptance radius: 5 meters
            
            # PARAM2: Pass radius (0 = pass through waypoint, >0 = orbit radius)
            param2 = 0.0  # Pass through waypoint
            
            # PARAM3: Yaw angle (heading in degrees, 0 = North, -1 = no change)
            # Use -1 for no yaw change, or specific angle
            param3 = -1.0  # No yaw change (drone maintains current heading)
            # Alternatively, can set to yaw value: param3 = yaw
            
            # PARAM4: Loiter radius (for LOITER commands) or latitude (for some commands)
            param4 = 0.0
            
            # PARAM5/X: Latitude
            param5 = waypoint.latitude
            
            # PARAM6/Y: Longitude
            param6 = waypoint.longitude
            
            # PARAM7/Z: Altitude
            param7 = waypoint.altitude
            
            # CURRENT_WP: 1 for first waypoint, 0 for others
            current_wp = 1 if idx == 0 else 0
            
            # COORD_FRAME: 0 = MAV_FRAME_GLOBAL (WGS84)
            coord_frame = 0
            
            # AUTOCONTINUE: 1 = continue to next waypoint automatically
            autocontinue = 1
            
            # Format line: INDEX, CURRENT_WP, COORD_FRAME, COMMAND, PARAM1, PARAM2, PARAM3, PARAM4, PARAM5/X, PARAM6/Y, PARAM7/Z, AUTOCONTINUE
            line = (f"{idx}\t{current_wp}\t{coord_frame}\t{command}\t"
                   f"{param1:.6f}\t{param2:.6f}\t{param3:.6f}\t{param4:.6f}\t"
                   f"{param5:.10f}\t{param6:.10f}\t{param7:.2f}\t{autocontinue}")
            lines.append(line)
        
        # Write to file
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))
    
    @staticmethod
    def export_mission(mission: Mission, output_dir: str):
        """Export all routes in a mission to separate .plan files.
        
        Args:
            mission: Mission to export
            output_dir: Output directory
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        for drone_name, route in mission.routes.items():
            # Find drone for this route
            drone = next((d for d in mission.drones if d.name == drone_name), None)
            file_path = output_path / f"{mission.name}_{drone_name}.plan"
            PlanExporter.export_route(route, str(file_path), drone=drone, mission=mission)

