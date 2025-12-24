"""Weather provider using Open Meteo API."""
import requests
from typing import Optional, Dict, List
from datetime import datetime, timedelta
from dataclasses import dataclass
import math


@dataclass
class WeatherConditions:
    """Weather conditions at a specific location and time."""
    latitude: float
    longitude: float
    altitude: float  # meters
    timestamp: datetime
    
    # Wind conditions
    wind_speed_10m: float  # m/s at 10m altitude
    wind_direction_10m: float  # degrees (0-360)
    wind_speed_80m: Optional[float] = None  # m/s at 80m altitude
    wind_direction_80m: Optional[float] = None  # degrees
    
    # Temperature
    temperature_2m: float  # Celsius
    
    # Precipitation
    precipitation: float = 0.0  # mm
    
    # Cloud cover
    cloud_cover: float = 0.0  # percentage (0-100)
    
    # Visibility
    visibility: Optional[float] = None  # km
    
    def get_wind_speed_at_altitude(self, altitude_m: float) -> float:
        """Estimate wind speed at given altitude using power law.
        
        Args:
            altitude_m: Altitude in meters
        
        Returns:
            Estimated wind speed in m/s
        """
        # Power law wind profile: v(z) = v_ref * (z/z_ref)^alpha
        # alpha typically 0.1-0.3, using 0.15 as default
        alpha = 0.15
        
        if altitude_m <= 10:
            return self.wind_speed_10m
        elif altitude_m >= 80 and self.wind_speed_80m:
            # Use 80m data if available
            ref_speed = self.wind_speed_80m
            ref_alt = 80.0
        else:
            ref_speed = self.wind_speed_10m
            ref_alt = 10.0
        
        if ref_alt <= 0:
            return ref_speed
        
        # Power law estimation
        wind_speed = ref_speed * ((altitude_m / ref_alt) ** alpha)
        return wind_speed
    
    def get_effective_wind_speed(self, heading: float, altitude_m: float) -> float:
        """Calculate effective wind speed for a given heading.
        
        Positive = headwind (increases energy consumption)
        Negative = tailwind (decreases energy consumption)
        
        Args:
            heading: Direction of travel in degrees (0-360, 0 = North)
            altitude_m: Altitude in meters
        
        Returns:
            Effective wind speed (positive = headwind, negative = tailwind)
        """
        wind_speed = self.get_wind_speed_at_altitude(altitude_m)
        wind_dir = self.wind_direction_10m
        
        # Calculate angle difference
        angle_diff = abs(heading - wind_dir)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        # Convert to radians
        angle_rad = math.radians(angle_diff)
        
        # Effective wind = wind_speed * cos(angle)
        # Positive = headwind, negative = tailwind
        effective_wind = wind_speed * math.cos(angle_rad)
        
        return effective_wind
    
    def is_safe_for_flight(self, max_wind_speed: float = 15.0, 
                          max_precipitation: float = 5.0,
                          min_visibility: float = 1.0) -> tuple[bool, Optional[str]]:
        """Check if weather conditions are safe for flight.
        
        Args:
            max_wind_speed: Maximum acceptable wind speed (m/s)
            max_precipitation: Maximum acceptable precipitation (mm/h)
            min_visibility: Minimum acceptable visibility (km)
        
        Returns:
            (is_safe, error_message)
        """
        if self.wind_speed_10m > max_wind_speed:
            return False, f"Wind speed {self.wind_speed_10m:.1f} m/s exceeds maximum {max_wind_speed} m/s"
        
        if self.precipitation > max_precipitation:
            return False, f"Precipitation {self.precipitation:.1f} mm/h exceeds maximum {max_precipitation} mm/h"
        
        if self.visibility is not None and self.visibility < min_visibility:
            return False, f"Visibility {self.visibility:.1f} km is below minimum {min_visibility} km"
        
        return True, None


class WeatherProvider:
    """Provider for weather data from Open Meteo API."""
    
    BASE_URL = "https://api.open-meteo.com/v1/forecast"
    
    def __init__(self, base_url: Optional[str] = None):
        """Initialize weather provider.
        
        Args:
            base_url: Base URL for Open Meteo API (default: public API)
        """
        self.base_url = base_url or self.BASE_URL
    
    def get_weather(self, latitude: float, longitude: float,
                   altitude: float = 0.0,
                   timestamp: Optional[datetime] = None) -> Optional[WeatherConditions]:
        """Get current weather conditions for a location.
        
        Args:
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude in meters
            timestamp: Time for weather data (default: current time)
        
        Returns:
            WeatherConditions object, or None if request fails
        """
        if timestamp is None:
            timestamp = datetime.now()
        
        try:
            # Open Meteo API parameters
            params = {
                "latitude": latitude,
                "longitude": longitude,
                "hourly": "temperature_2m,precipitation,windspeed_10m,winddirection_10m,cloudcover,visibility",
                "windspeed_unit": "ms",
                "timezone": "auto",
                "forecast_days": 1
            }
            
            # Add elevation if needed
            if altitude > 0:
                params["elevation"] = altitude
            
            # Make request
            response = requests.get(self.BASE_URL, params=params, timeout=10)
            response.raise_for_status()
            data = response.json()
            
            # Extract hourly data
            hourly = data.get("hourly", {})
            times = hourly.get("time", [])
            temperatures = hourly.get("temperature_2m", [])
            precipitations = hourly.get("precipitation", [])
            wind_speeds_10m = hourly.get("windspeed_10m", [])
            wind_directions_10m = hourly.get("winddirection_10m", [])
            cloud_covers = hourly.get("cloudcover", [])
            visibilities = hourly.get("visibility", [])
            
            # Find closest time index
            target_time_str = timestamp.strftime("%Y-%m-%dT%H:00")
            time_index = 0
            if times:
                try:
                    time_index = times.index(target_time_str)
                except ValueError:
                    # Use first available time if exact match not found
                    time_index = 0
            
            # Get values at time index
            temperature = temperatures[time_index] if time_index < len(temperatures) else 15.0
            precipitation = precipitations[time_index] if time_index < len(precipitations) else 0.0
            wind_speed_10m = wind_speeds_10m[time_index] if time_index < len(wind_speeds_10m) else 0.0
            wind_direction_10m = wind_directions_10m[time_index] if time_index < len(wind_directions_10m) else 0.0
            cloud_cover = cloud_covers[time_index] if time_index < len(cloud_covers) else 0.0
            visibility = visibilities[time_index] if time_index < len(visibilities) else None
            
            # Try to get wind at 80m if available
            params_80m = params.copy()
            params_80m["hourly"] = "windspeed_80m,winddirection_80m"
            try:
                response_80m = requests.get(self.BASE_URL, params=params_80m, timeout=10)
                response_80m.raise_for_status()
                data_80m = response_80m.json()
                hourly_80m = data_80m.get("hourly", {})
                wind_speeds_80m = hourly_80m.get("windspeed_80m", [])
                wind_directions_80m = hourly_80m.get("winddirection_80m", [])
                
                wind_speed_80m = wind_speeds_80m[time_index] if time_index < len(wind_speeds_80m) else None
                wind_direction_80m = wind_directions_80m[time_index] if time_index < len(wind_directions_80m) else None
            except:
                wind_speed_80m = None
                wind_direction_80m = None
            
            return WeatherConditions(
                latitude=latitude,
                longitude=longitude,
                altitude=altitude,
                timestamp=timestamp,
                wind_speed_10m=wind_speed_10m,
                wind_direction_10m=wind_direction_10m,
                wind_speed_80m=wind_speed_80m,
                wind_direction_80m=wind_direction_80m,
                temperature_2m=temperature,
                precipitation=precipitation,
                cloud_cover=cloud_cover,
                visibility=visibility
            )
        
        except Exception as e:
            print(f"Error fetching weather data: {e}")
            return None
    
    def get_weather_along_route(self, waypoints: List[tuple[float, float, float]],
                                timestamp: Optional[datetime] = None) -> Dict[tuple[float, float], WeatherConditions]:
        """Get weather conditions along a route.
        
        Args:
            waypoints: List of (latitude, longitude, altitude) tuples
            timestamp: Time for weather data
        
        Returns:
            Dictionary mapping (lat, lon) to WeatherConditions
        """
        weather_map = {}
        
        # Get weather for each waypoint
        for lat, lon, alt in waypoints:
            key = (lat, lon)
            if key not in weather_map:
                weather = self.get_weather(lat, lon, alt, timestamp)
                if weather:
                    weather_map[key] = weather
        
        return weather_map
    
    def get_weather_grid(self, center_lat: float, center_lon: float,
                        width: float, height: float,
                        resolution: float = 1000.0,
                        timestamp: Optional[datetime] = None) -> Dict[tuple[float, float], WeatherConditions]:
        """Get weather data for a grid of points.
        
        Args:
            center_lat: Center latitude
            center_lon: Center longitude
            width: Grid width in meters
            height: Grid height in meters
            resolution: Grid resolution in meters
            timestamp: Time for weather data
        
        Returns:
            Dictionary mapping (lat, lon) to WeatherConditions
        """
        weather_map = {}
        
        # Calculate grid points
        lat_per_meter = 1.0 / 111320.0
        lon_per_meter = 1.0 / (111320.0 * math.cos(math.radians(center_lat)))
        
        cols = int(width / resolution) + 1
        rows = int(height / resolution) + 1
        
        for i in range(rows):
            for j in range(cols):
                lat_offset = (i - rows / 2) * resolution * lat_per_meter
                lon_offset = (j - cols / 2) * resolution * lon_per_meter
                
                lat = center_lat + lat_offset
                lon = center_lon + lon_offset
                
                key = (lat, lon)
                if key not in weather_map:
                    weather = self.get_weather(lat, lon, 0.0, timestamp)
                    if weather:
                        weather_map[key] = weather
        
        return weather_map

