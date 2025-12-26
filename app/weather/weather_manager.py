"""Weather manager for integrated weather fetching during route planning."""
from typing import Dict, Optional, List, Tuple, Set
from datetime import datetime
from app.weather.weather_provider import WeatherProvider, WeatherConditions
from app.domain.waypoint import Waypoint
import math


class WeatherManager:
    """Manages weather data fetching during route planning.
    
    Features:
    - Lazy fetching: only fetches weather when needed
    - Caching: stores fetched weather data
    - Smart interpolation: uses nearby weather data when appropriate
    - Batch optimization: can fetch weather for multiple points efficiently
    """
    
    # Minimum distance threshold for using cached weather (5km)
    MIN_WEATHER_DISTANCE = 5000.0  # meters
    
    # Grid resolution for weather caching (1km)
    WEATHER_GRID_RESOLUTION = 1000.0  # meters
    
    def __init__(self, weather_provider: Optional[WeatherProvider] = None,
                 initial_weather_data: Optional[Dict[tuple[float, float], WeatherConditions]] = None,
                 use_weather: bool = True):
        """Initialize weather manager.
        
        Args:
            weather_provider: WeatherProvider instance (creates new one if None)
            initial_weather_data: Pre-fetched weather data to use as cache
            use_weather: Whether to fetch weather data (if False, returns None for all requests)
        """
        self.weather_provider = weather_provider or WeatherProvider()
        self.weather_cache: Dict[tuple[float, float], WeatherConditions] = {}
        self.use_weather = use_weather
        
        # Initialize cache with provided data
        if initial_weather_data:
            self.weather_cache.update(initial_weather_data)
        
        # Track which points we've fetched weather for (to avoid duplicate requests)
        self.fetched_points: Set[tuple[float, float]] = set()
    
    def get_weather_for_point(self, latitude: float, longitude: float, 
                             altitude: float = 0.0,
                             timestamp: Optional[datetime] = None) -> Optional[WeatherConditions]:
        """Get weather conditions for a point.
        
        Strategy:
        1. Check cache for exact match
        2. Check cache for nearby point (within MIN_WEATHER_DISTANCE)
        3. Fetch new weather if needed
        4. Cache the result
        
        Args:
            latitude: Point latitude
            longitude: Point longitude
            altitude: Point altitude
            timestamp: Time for weather data (default: current time)
            
        Returns:
            WeatherConditions or None if use_weather is False or fetch fails
        """
        if not self.use_weather:
            return None
        
        # Round coordinates to grid for caching (1km resolution)
        grid_key = self._round_to_grid(latitude, longitude)
        
        # Check exact cache first
        if grid_key in self.weather_cache:
            return self.weather_cache[grid_key]
        
        # Check for nearby cached weather
        cached_weather = self._find_nearby_weather(latitude, longitude)
        if cached_weather:
            return cached_weather
        
        # Fetch new weather if not already fetched for this grid point
        if grid_key not in self.fetched_points:
            weather = self.weather_provider.get_weather(latitude, longitude, altitude, timestamp)
            if weather:
                self.weather_cache[grid_key] = weather
                self.fetched_points.add(grid_key)
                return weather
        
        # If we've already tried to fetch but failed, return None
        return None
    
    def get_weather_for_waypoints(self, waypoints: List[Waypoint],
                                  timestamp: Optional[datetime] = None) -> Dict[tuple[float, float], WeatherConditions]:
        """Get weather for multiple waypoints efficiently.
        
        Args:
            waypoints: List of waypoints
            timestamp: Time for weather data
            
        Returns:
            Dictionary mapping (lat, lon) to WeatherConditions
        """
        weather_map = {}
        
        for wp in waypoints:
            key = (wp.latitude, wp.longitude)
            if key not in weather_map:
                weather = self.get_weather_for_point(wp.latitude, wp.longitude, wp.altitude, timestamp)
                if weather:
                    weather_map[key] = weather
        
        return weather_map
    
    def get_weather_for_route_segment(self, lat1: float, lon1: float, alt1: float,
                                     lat2: float, lon2: float, alt2: float,
                                     num_points: int = 3,
                                     timestamp: Optional[datetime] = None) -> List[Optional[WeatherConditions]]:
        """Get weather for points along a route segment.
        
        This is useful for getting weather data along the path between waypoints,
        not just at the waypoints themselves.
        
        Args:
            lat1, lon1, alt1: Start point
            lat2, lon2, alt2: End point
            num_points: Number of points to sample along the segment
            timestamp: Time for weather data
            
        Returns:
            List of WeatherConditions (may contain None if fetch fails)
        """
        weather_list = []
        
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0.0
            lat = lat1 + (lat2 - lat1) * t
            lon = lon1 + (lon2 - lon1) * t
            alt = alt1 + (alt2 - alt1) * t
            
            weather = self.get_weather_for_point(lat, lon, alt, timestamp)
            weather_list.append(weather)
        
        return weather_list
    
    def pre_fetch_weather_for_mission(self, mission, timestamp: Optional[datetime] = None):
        """Pre-fetch weather data for all mission waypoints.
        
        This can be called before route planning to ensure weather data is available.
        
        Args:
            mission: Mission object
            timestamp: Time for weather data
        """
        if not self.use_weather:
            return
        
        waypoints_to_fetch = []
        
        # Add depot
        if mission.depot:
            waypoints_to_fetch.append(mission.depot)
        
        # Add target points
        waypoints_to_fetch.extend(mission.target_points)
        
        # Add finish point
        if mission.finish_point:
            waypoints_to_fetch.append(mission.finish_point)
        
        # Fetch weather for all waypoints
        for wp in waypoints_to_fetch:
            self.get_weather_for_point(wp.latitude, wp.longitude, wp.altitude, timestamp)
    
    def _find_nearby_weather(self, latitude: float, longitude: float) -> Optional[WeatherConditions]:
        """Find weather data for a nearby point (within MIN_WEATHER_DISTANCE).
        
        Args:
            latitude: Point latitude
            longitude: Point longitude
            
        Returns:
            WeatherConditions if found nearby, None otherwise
        """
        min_distance = float('inf')
        closest_weather = None
        
        for (lat, lon), weather in self.weather_cache.items():
            distance = self._haversine_distance(latitude, longitude, lat, lon)
            if distance < min_distance and distance < self.MIN_WEATHER_DISTANCE:
                min_distance = distance
                closest_weather = weather
        
        return closest_weather
    
    def _round_to_grid(self, latitude: float, longitude: float) -> tuple[float, float]:
        """Round coordinates to weather grid for caching.
        
        Args:
            latitude: Latitude
            longitude: Longitude
            
        Returns:
            Rounded (lat, lon) tuple
        """
        # Convert resolution from meters to degrees
        lat_per_meter = 1.0 / 111320.0
        lon_per_meter = 1.0 / (111320.0 * math.cos(math.radians(latitude)))
        
        lat_grid = round(latitude / (self.WEATHER_GRID_RESOLUTION * lat_per_meter)) * (self.WEATHER_GRID_RESOLUTION * lat_per_meter)
        lon_grid = round(longitude / (self.WEATHER_GRID_RESOLUTION * lon_per_meter)) * (self.WEATHER_GRID_RESOLUTION * lon_per_meter)
        
        return (lat_grid, lon_grid)
    
    @staticmethod
    def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two points using Haversine formula.
        
        Args:
            lat1, lon1: First point
            lat2, lon2: Second point
            
        Returns:
            Distance in meters
        """
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c
    
    def get_all_weather_data(self) -> Dict[tuple[float, float], WeatherConditions]:
        """Get all cached weather data.
        
        Returns:
            Dictionary of all cached weather data
        """
        return self.weather_cache.copy()

