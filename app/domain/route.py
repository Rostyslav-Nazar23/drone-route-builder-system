"""Route domain model."""
from dataclasses import dataclass, field
from typing import List, Optional
from math import radians, sin, cos, atan2, degrees, sqrt
from .waypoint import Waypoint


@dataclass
class RouteMetrics:
    """Metrics for a route."""
    total_distance: float = 0.0  # meters
    total_time: float = 0.0  # seconds
    total_energy: float = 0.0  # Wh
    max_altitude: float = 0.0
    min_altitude: float = 0.0
    waypoint_count: int = 0
    risk_score: float = 0.0  # Risk value (0-1, higher = more risky)
    avg_speed: float = 0.0  # Average speed (m/s)
    
    def to_dict(self) -> dict:
        """Convert metrics to dictionary."""
        return {
            "total_distance": self.total_distance,
            "total_time": self.total_time,
            "total_energy": self.total_energy,
            "max_altitude": self.max_altitude,
            "min_altitude": self.min_altitude,
            "waypoint_count": self.waypoint_count,
            "risk_score": self.risk_score,
            "avg_speed": self.avg_speed
        }


@dataclass
class Route:
    """Represents a planned route for a drone."""
    waypoints: List[Waypoint] = field(default_factory=list)
    drone_name: Optional[str] = None
    metrics: Optional[RouteMetrics] = None
    validation_result: Optional[dict] = None
    
    def add_waypoint(self, waypoint: Waypoint):
        """Add a waypoint to the route."""
        self.waypoints.append(waypoint)
    
    def calculate_metrics(self, drone, weather_data=None) -> RouteMetrics:
        """Calculate route metrics based on drone capabilities.
        
        Includes:
        - Inertia for speed reduction/acceleration
        - Energy consumption at high speeds
        - Weather conditions influence on speed and energy
        - Risk calculation for high humidity/rain
        
        Args:
            drone: Drone object
            weather_data: Optional dict mapping (lat, lon) to WeatherConditions.
                        This dict may be modified to add new weather data for route points.
        """
        if not self.waypoints:
            return RouteMetrics()
        
        from app.weather.weather_provider import WeatherConditions
        
        total_distance = 0.0
        total_energy = 0.0
        total_time = 0.0
        altitudes = [wp.altitude for wp in self.waypoints]
        risk_factors = []
        
        current_speed = 0.0  # Start from rest
        acceleration = drone.max_speed / 5.0  # Reach max speed in 5 seconds
        deceleration = drone.max_speed / 5.0  # Decelerate in 5 seconds
        
        # Calculate distances between consecutive waypoints
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            
            # Calculate distance and heading
            distance = self._haversine_distance(
                wp1.latitude, wp1.longitude,
                wp2.latitude, wp2.longitude
            )
            altitude_change = wp2.altitude - wp1.altitude
            
            # Calculate heading (0-360 degrees, 0 = North)
            lat1_rad = radians(wp1.latitude)
            lat2_rad = radians(wp2.latitude)
            dlon_rad = radians(wp2.longitude - wp1.longitude)
            
            y = sin(dlon_rad) * cos(lat2_rad)
            x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dlon_rad)
            heading = degrees(atan2(y, x))
            if heading < 0:
                heading += 360
            
            total_distance += distance
            
            # Get weather conditions for this segment
            weather = None
            if weather_data:
                # Try to get weather for midpoint of segment
                mid_lat = (wp1.latitude + wp2.latitude) / 2
                mid_lon = (wp1.longitude + wp2.longitude) / 2
                mid_alt = (wp1.altitude + wp2.altitude) / 2
                
                # Find closest weather data
                closest_key = None
                min_dist = float('inf')
                for key in weather_data.keys():
                    key_lat, key_lon = key
                    dist = self._haversine_distance(mid_lat, mid_lon, key_lat, key_lon)
                    if dist < min_dist:
                        min_dist = dist
                        closest_key = key
                
                # Minimum distance threshold: if > 5km, fetch weather for this point
                MIN_WEATHER_DISTANCE = 5000.0  # 5km in meters
                
                if closest_key and min_dist < MIN_WEATHER_DISTANCE:
                    weather = weather_data[closest_key]
                else:
                    # Point is too far from existing weather data, fetch new weather
                    from app.weather.weather_provider import WeatherProvider
                    weather_provider = WeatherProvider()
                    new_weather = weather_provider.get_weather(mid_lat, mid_lon, mid_alt)
                    if new_weather:
                        weather_key = (mid_lat, mid_lon)
                        weather_data[weather_key] = new_weather  # Cache it
                        weather = new_weather
            
            # Calculate effective speed considering weather
            effective_max_speed = drone.max_speed
            
            if weather:
                # Wind impact on speed
                effective_wind = weather.get_effective_wind_speed(heading, mid_alt)
                # Headwind reduces effective speed, tailwind increases it (up to max)
                effective_max_speed = max(0.1 * drone.max_speed, 
                                        min(drone.max_speed * 1.2, 
                                            drone.max_speed - effective_wind * 0.5))
                
                # Calculate risk from weather
                risk = 0.0
                # High humidity (>80%) increases risk
                if weather.temperature_2m > 0:  # Only if we have temp data
                    # Estimate humidity (simplified - in real system would use actual humidity)
                    # High precipitation = high humidity
                    if weather.precipitation > 2.0:  # >2mm/h
                        risk += 0.3
                    if weather.precipitation > 5.0:  # >5mm/h (heavy rain)
                        risk += 0.4
                
                # High wind increases risk
                wind_speed = weather.get_wind_speed_at_altitude(mid_alt)
                if wind_speed > 10.0:
                    risk += 0.2
                if wind_speed > 15.0:
                    risk += 0.3
                
                # Low visibility increases risk
                if weather.visibility and weather.visibility < 2.0:
                    risk += 0.2
                
                risk_factors.append(risk)
            
            # Calculate time with inertia (acceleration/deceleration)
            # Simplified: assume we accelerate to max, cruise, then decelerate
            if distance > 0:
                # Time to accelerate to effective max speed
                accel_time = (effective_max_speed - current_speed) / acceleration if acceleration > 0 else 0
                accel_distance = current_speed * accel_time + 0.5 * acceleration * accel_time ** 2
                
                # Time to decelerate (assume we need to slow down at end)
                decel_time = effective_max_speed / deceleration if deceleration > 0 else 0
                decel_distance = effective_max_speed * decel_time - 0.5 * deceleration * decel_time ** 2
                
                # Cruise distance
                cruise_distance = max(0, distance - accel_distance - decel_distance)
                cruise_time = cruise_distance / effective_max_speed if effective_max_speed > 0 else 0
                
                segment_time = accel_time + cruise_time + decel_time
                total_time += segment_time
                
                # Update current speed for next segment
                current_speed = max(0, effective_max_speed - deceleration * decel_time)
            else:
                segment_time = 0
            
            # Calculate energy consumption with weather and speed effects
            # Base energy
            base_energy = drone.estimate_energy_consumption(distance, altitude_change)
            
            # High speed energy multiplier (quadratic relationship)
            speed_factor = (effective_max_speed / drone.max_speed) ** 2
            energy_multiplier = 1.0 + 0.5 * (speed_factor - 1.0)  # 50% more energy at max speed
            
            # Weather impact on energy
            weather_energy_multiplier = 1.0
            if weather:
                # Headwind increases energy consumption
                effective_wind = weather.get_effective_wind_speed(heading, mid_alt)
                if effective_wind > 0:  # Headwind
                    weather_energy_multiplier = 1.0 + (effective_wind / drone.max_speed) * 0.3
                elif effective_wind < 0:  # Tailwind
                    weather_energy_multiplier = 1.0 + (effective_wind / drone.max_speed) * 0.1  # Slight reduction
                
                # Precipitation increases energy (water resistance)
                if weather.precipitation > 0:
                    weather_energy_multiplier += weather.precipitation * 0.05
            
            segment_energy = base_energy * energy_multiplier * weather_energy_multiplier
            total_energy += segment_energy
        
        # Calculate average risk
        avg_risk = sum(risk_factors) / len(risk_factors) if risk_factors else 0.0
        avg_risk = min(1.0, avg_risk)  # Cap at 1.0
        
        # Calculate average speed
        avg_speed = total_distance / total_time if total_time > 0 else 0.0
        
        self.metrics = RouteMetrics(
            total_distance=total_distance,
            total_time=total_time,
            total_energy=total_energy,
            max_altitude=max(altitudes) if altitudes else 0.0,
            min_altitude=min(altitudes) if altitudes else 0.0,
            waypoint_count=len(self.waypoints),
            risk_score=avg_risk,
            avg_speed=avg_speed
        )
        return self.metrics
    
    @staticmethod
    def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two points using Haversine formula."""
        from math import radians, sin, cos, sqrt, atan2
        
        R = 6371000  # Earth radius in meters
        
        lat1_rad = radians(lat1)
        lat2_rad = radians(lat2)
        delta_lat = radians(lat2 - lat1)
        delta_lon = radians(lon2 - lon1)
        
        a = sin(delta_lat / 2) ** 2 + cos(lat1_rad) * cos(lat2_rad) * sin(delta_lon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        
        return R * c
    
    def to_dict(self) -> dict:
        """Convert route to dictionary."""
        return {
            "waypoints": [wp.to_dict() for wp in self.waypoints],
            "drone_name": self.drone_name,
            "metrics": self.metrics.to_dict() if self.metrics else None
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> "Route":
        """Create route from dictionary."""
        waypoints = [Waypoint.from_dict(wp) for wp in data.get("waypoints", [])]
        metrics = None
        if data.get("metrics"):
            metrics = RouteMetrics(**data["metrics"])
        
        return cls(
            waypoints=waypoints,
            drone_name=data.get("drone_name"),
            metrics=metrics
        )

