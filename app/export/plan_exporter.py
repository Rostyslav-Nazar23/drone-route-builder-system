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
    MAV_CMD_DO_CHANGE_SPEED = 178  # Change speed command
    MAV_CMD_CONDITION_YAW = 115  # Set yaw/heading
    MAV_CMD_NAV_LOITER_TO_ALT = 31  # Loiter to altitude
    
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
        
        # Track previous speed to detect changes
        previous_speed = None
        
        # Waypoints
        waypoint_index = 0  # Track actual waypoint index (may include speed commands)
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
            else:
                # First waypoint: use default speed
                speed = PlanExporter._calculate_speed_for_segment(None, None, drone, avg_speed)
            
            # Add speed change command if speed changed (except for first waypoint)
            if previous_speed is not None and abs(speed - previous_speed) > 0.1:  # Speed changed by more than 0.1 m/s
                # MAV_CMD_DO_CHANGE_SPEED: PARAM1 = speed type (0=air speed, 1=ground speed), PARAM2 = speed (m/s), PARAM3 = throttle (-1=no change), PARAM4 = absolute/relative (0=absolute)
                speed_cmd = PlanExporter.MAV_CMD_DO_CHANGE_SPEED
                speed_line = (f"{waypoint_index}\t0\t0\t{speed_cmd}\t"
                            f"1.0\t{speed:.2f}\t-1.0\t0.0\t"
                            f"0.0\t0.0\t0.0\t1")
                lines.append(speed_line)
                waypoint_index += 1
            
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
            
            # Add yaw command before waypoint (except for first waypoint which is TAKEOFF)
            if idx > 0 and command == PlanExporter.MAV_CMD_NAV_WAYPOINT:
                # MAV_CMD_CONDITION_YAW: PARAM1 = target angle (degrees), PARAM2 = angular speed (deg/s), PARAM3 = direction (-1=shortest, 0=cw, 1=ccw), PARAM4 = relative/absolute (0=absolute)
                yaw_cmd = PlanExporter.MAV_CMD_CONDITION_YAW
                yaw_line = (f"{waypoint_index}\t0\t0\t{yaw_cmd}\t"
                           f"{yaw:.2f}\t45.0\t-1.0\t0.0\t"
                           f"0.0\t0.0\t0.0\t1")
                lines.append(yaw_line)
                waypoint_index += 1
            
            # PARAM1: Hold time (for LOITER) or acceptance radius (for WAYPOINT)
            # For waypoints, use acceptance radius (meters)
            param1 = 5.0  # Default acceptance radius: 5 meters
            
            # PARAM2: Pass radius (0 = pass through waypoint, >0 = orbit radius)
            param2 = 0.0  # Pass through waypoint
            
            # PARAM3: Yaw angle (heading in degrees, 0 = North, -1 = no change)
            # Set to calculated yaw for waypoints, -1 for TAKEOFF/LAND
            if command == PlanExporter.MAV_CMD_NAV_WAYPOINT:
                param3 = yaw  # Set yaw for waypoints
            else:
                param3 = -1.0  # No yaw change for TAKEOFF/LAND
            
            # PARAM4: Loiter radius (for LOITER commands) or latitude (for some commands)
            param4 = 0.0
            
            # For target waypoints, consider adding loiter time
            if "target" in (waypoint.waypoint_type.lower() if waypoint.waypoint_type else "") and idx < len(route.waypoints) - 1:
                # Add a small loiter time at target points (2 seconds)
                param1 = 2.0  # Hold time in seconds for target points
            
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
            line = (f"{waypoint_index}\t{current_wp}\t{coord_frame}\t{command}\t"
                   f"{param1:.6f}\t{param2:.6f}\t{param3:.6f}\t{param4:.6f}\t"
                   f"{param5:.10f}\t{param6:.10f}\t{param7:.2f}\t{autocontinue}")
            lines.append(line)
            
            # Update indices
            waypoint_index += 1
            previous_speed = speed
        
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

