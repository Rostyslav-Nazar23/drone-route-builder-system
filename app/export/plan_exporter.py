"""Exporter for .plan file format."""
from typing import List
from pathlib import Path
from app.domain.mission import Mission
from app.domain.route import Route


class PlanExporter:
    """Exports routes to .plan file format."""
    
    @staticmethod
    def export_route(route: Route, file_path: str):
        """Export a single route to .plan file.
        
        Args:
            route: Route to export
            file_path: Output file path
        """
        lines = []
        
        # Header
        lines.append("QGC WPL 110")
        
        # Waypoints
        for idx, waypoint in enumerate(route.waypoints):
            # Format: INDEX, CURRENT_WP, COORD_FRAME, COMMAND, PARAM1, PARAM2, PARAM3, PARAM4, PARAM5/X, PARAM6/Y, PARAM7/Z, AUTOCONTINUE
            # MAVLink waypoint format
            command = 16  # MAV_CMD_NAV_WAYPOINT
            if idx == 0:
                command = 22  # MAV_CMD_NAV_TAKEOFF
            elif idx == len(route.waypoints) - 1:
                command = 21  # MAV_CMD_NAV_LAND
            
            line = f"{idx}\t{1 if idx == 0 else 0}\t0\t{command}\t0\t0\t0\t0\t{waypoint.latitude:.10f}\t{waypoint.longitude:.10f}\t{waypoint.altitude:.2f}\t1"
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
            file_path = output_path / f"{mission.name}_{drone_name}.plan"
            PlanExporter.export_route(route, str(file_path))

