"""CSV loader for waypoints and mission data."""
import csv
from typing import List, Optional
from pathlib import Path
from app.domain.waypoint import Waypoint
from app.data_import.validators import validate_waypoint


def load_waypoints_from_csv(file_path: str, 
                            lat_col: str = "latitude",
                            lon_col: str = "longitude",
                            alt_col: str = "altitude",
                            name_col: Optional[str] = "name") -> List[Waypoint]:
    """Load waypoints from CSV file.
    
    Args:
        file_path: Path to CSV file
        lat_col: Name of latitude column
        lon_col: Name of longitude column
        alt_col: Name of altitude column
        name_col: Name of name column (optional)
    
    Returns:
        List of Waypoint objects
    """
    waypoints = []
    
    with open(file_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        
        for row_num, row in enumerate(reader, start=2):  # Start at 2 (header is row 1)
            try:
                lat = float(row[lat_col])
                lon = float(row[lon_col])
                alt = float(row.get(alt_col, 0.0))
                name = row.get(name_col) if name_col and name_col in row else None
                
                # Validate
                is_valid, error = validate_waypoint(lat, lon, alt)
                if not is_valid:
                    raise ValueError(f"Row {row_num}: {error}")
                
                waypoint = Waypoint(
                    latitude=lat,
                    longitude=lon,
                    altitude=alt,
                    name=name
                )
                waypoints.append(waypoint)
            except KeyError as e:
                raise ValueError(f"Row {row_num}: Missing required column: {e}")
            except ValueError as e:
                raise ValueError(f"Row {row_num}: {str(e)}")
    
    return waypoints


def save_waypoints_to_csv(waypoints: List[Waypoint], file_path: str):
    """Save waypoints to CSV file."""
    with open(file_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['name', 'latitude', 'longitude', 'altitude', 'type'])
        
        for wp in waypoints:
            writer.writerow([
                wp.name or '',
                wp.latitude,
                wp.longitude,
                wp.altitude,
                wp.waypoint_type
            ])

