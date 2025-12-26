"""GeoJSON loader for no-fly zones and spatial data."""
import json
from typing import List, Optional
from pathlib import Path
from shapely.geometry import shape
from shapely.geometry.base import BaseGeometry
from shapely.validation import make_valid
from app.domain.constraints import NoFlyZone
from app.domain.waypoint import Waypoint
from app.data_import.validators import validate_geojson_geometry


def load_no_fly_zones_from_geojson(file_path: str,
                                   min_altitude: float = 0.0,
                                   max_altitude: float = 1000.0) -> List[NoFlyZone]:
    """Load no-fly zones from GeoJSON file.
    
    Args:
        file_path: Path to GeoJSON file
        min_altitude: Minimum altitude for zones (meters)
        max_altitude: Maximum altitude for zones (meters)
    
    Returns:
        List of NoFlyZone objects
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        geojson_data = json.load(f)
    
    zones = []
    
    # Handle FeatureCollection
    if geojson_data.get("type") == "FeatureCollection":
        features = geojson_data.get("features", [])
    # Handle single Feature
    elif geojson_data.get("type") == "Feature":
        features = [geojson_data]
    # Handle raw geometry
    elif geojson_data.get("type") in ["Polygon", "MultiPolygon", "Point", "LineString", "MultiLineString"]:
        features = [{"geometry": geojson_data, "properties": {}}]
    else:
        raise ValueError(f"Unsupported GeoJSON type: {geojson_data.get('type')}")
    
    for idx, feature in enumerate(features):
        geometry_data = feature.get("geometry")
        if not geometry_data:
            continue
        
        properties = feature.get("properties", {})
        name = properties.get("name") or f"Zone_{idx + 1}"
        
        # Validate and create geometry
        is_valid, error, geometry = validate_geojson_geometry(geometry_data)
        if not is_valid:
            raise ValueError(f"Feature {idx + 1}: {error}")
        
        # Only create zones for Polygon/MultiPolygon geometries
        if geometry.geom_type not in ["Polygon", "MultiPolygon"]:
            continue
        
        # Get altitude constraints from properties if available
        zone_min_alt = properties.get("min_altitude", min_altitude)
        zone_max_alt = properties.get("max_altitude", max_altitude)
        
        zone = NoFlyZone(
            geometry=geometry,
            min_altitude=zone_min_alt,
            max_altitude=zone_max_alt,
            name=name
        )
        zones.append(zone)
    
    return zones


def load_waypoints_from_geojson(file_path: str) -> List[Waypoint]:
    """Load waypoints from GeoJSON file (Point features).
    
    Args:
        file_path: Path to GeoJSON file
    
    Returns:
        List of Waypoint objects
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        geojson_data = json.load(f)
    
    waypoints = []
    
    # Handle FeatureCollection
    if geojson_data.get("type") == "FeatureCollection":
        features = geojson_data.get("features", [])
    # Handle single Feature
    elif geojson_data.get("type") == "Feature":
        features = [geojson_data]
    else:
        raise ValueError(f"Unsupported GeoJSON type: {geojson_data.get('type')}")
    
    for idx, feature in enumerate(features):
        geometry_data = feature.get("geometry")
        if not geometry_data or geometry_data.get("type") != "Point":
            continue
        
        coordinates = geometry_data.get("coordinates")
        if len(coordinates) < 2:
            continue
        
        lon, lat = coordinates[0], coordinates[1]
        alt = coordinates[2] if len(coordinates) > 2 else 0.0
        
        properties = feature.get("properties", {})
        name = properties.get("name")
        
        waypoint = Waypoint(
            latitude=lat,
            longitude=lon,
            altitude=alt,
            name=name
        )
        waypoints.append(waypoint)
    
    return waypoints

