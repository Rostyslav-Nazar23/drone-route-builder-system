"""JSON exporter for missions and routes."""
import json
from pathlib import Path
from app.domain.mission import Mission
from app.domain.route import Route


class JSONExporter:
    """Exports missions and routes to JSON format."""
    
    @staticmethod
    def export_mission(mission: Mission, file_path: str):
        """Export mission to JSON file.
        
        Args:
            mission: Mission to export
            file_path: Output file path
        """
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(mission.to_dict(), f, indent=2, ensure_ascii=False)
    
    @staticmethod
    def export_route(route: Route, file_path: str):
        """Export route to JSON file.
        
        Args:
            route: Route to export
            file_path: Output file path
        """
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(route.to_dict(), f, indent=2, ensure_ascii=False)

