"""Main importer interface."""
from typing import List, Optional
from pathlib import Path
from app.domain.waypoint import Waypoint
from app.domain.constraints import NoFlyZone
from app.data_import.csv_loader import load_waypoints_from_csv, save_waypoints_to_csv
from app.data_import.geojson_loader import load_no_fly_zones_from_geojson, load_waypoints_from_geojson


class DataImporter:
    """Main interface for importing mission data."""
    
    @staticmethod
    def import_waypoints(file_path: str) -> List[Waypoint]:
        """Import waypoints from file (CSV or GeoJSON).
        
        Args:
            file_path: Path to CSV or GeoJSON file
        
        Returns:
            List of Waypoint objects
        """
        path = Path(file_path)
        suffix = path.suffix.lower()
        
        if suffix == '.csv':
            return load_waypoints_from_csv(file_path)
        elif suffix in ['.geojson', '.json']:
            return load_waypoints_from_geojson(file_path)
        else:
            raise ValueError(f"Unsupported file format: {suffix}. Supported: .csv, .geojson, .json")
    
    @staticmethod
    def import_no_fly_zones(file_path: str,
                           min_altitude: float = 0.0,
                           max_altitude: float = 1000.0) -> List[NoFlyZone]:
        """Import no-fly zones from GeoJSON file.
        
        Args:
            file_path: Path to GeoJSON file
            min_altitude: Minimum altitude for zones (meters)
            max_altitude: Maximum altitude for zones (meters)
        
        Returns:
            List of NoFlyZone objects
        """
        return load_no_fly_zones_from_geojson(file_path, min_altitude, max_altitude)
    
    @staticmethod
    def export_waypoints(waypoints: List[Waypoint], file_path: str):
        """Export waypoints to CSV file.
        
        Args:
            waypoints: List of Waypoint objects
            file_path: Output CSV file path
        """
        path = Path(file_path)
        suffix = path.suffix.lower()
        
        if suffix == '.csv':
            save_waypoints_to_csv(waypoints, file_path)
        else:
            raise ValueError(f"Unsupported export format: {suffix}. Supported: .csv")

