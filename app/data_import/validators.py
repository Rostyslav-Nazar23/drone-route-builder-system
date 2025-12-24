"""Validators for imported data."""
from typing import List, Tuple, Optional
from app.domain.waypoint import Waypoint
from app.domain.constraints import NoFlyZone
from shapely.geometry import Point, Polygon, shape, BaseGeometry
from shapely.validation import make_valid


def validate_waypoint(latitude: float, longitude: float, altitude: float) -> Tuple[bool, Optional[str]]:
    """Validate waypoint coordinates."""
    if not -90 <= latitude <= 90:
        return False, f"Latitude must be between -90 and 90, got {latitude}"
    if not -180 <= longitude <= 180:
        return False, f"Longitude must be between -180 and 180, got {longitude}"
    if altitude < 0:
        return False, f"Altitude must be non-negative, got {altitude}"
    return True, None


def validate_geojson_geometry(geometry: dict) -> Tuple[bool, Optional[str], Optional[BaseGeometry]]:
    """Validate and create Shapely geometry from GeoJSON."""
    try:
        geom = shape(geometry)
        # Make geometry valid if it's not
        if not geom.is_valid:
            geom = make_valid(geom)
        return True, None, geom
    except Exception as e:
        return False, f"Invalid geometry: {str(e)}", None

