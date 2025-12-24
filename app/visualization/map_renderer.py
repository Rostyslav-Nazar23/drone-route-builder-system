"""Map renderer for visualizing routes."""
from typing import List, Optional, Dict
import folium
from folium import plugins
from app.domain.route import Route
from app.domain.waypoint import Waypoint
from app.domain.constraints import MissionConstraints, NoFlyZone
from app.domain.mission import Mission
from app.weather.weather_provider import WeatherConditions


class MapRenderer:
    """Renders routes and missions on interactive maps."""
    
    def __init__(self, center_lat: float = 50.0, center_lon: float = 30.0, zoom_start: int = 10):
        """Initialize map renderer.
        
        Args:
            center_lat: Center latitude
            center_lon: Center longitude
            zoom_start: Initial zoom level
        """
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.zoom_start = zoom_start
    
    def render_route(self, route: Route, 
                    show_waypoints: bool = True,
                    show_path: bool = True,
                    color: str = "blue") -> folium.Map:
        """Render a single route on a map.
        
        Args:
            route: Route to render
            show_waypoints: Whether to show waypoint markers
            show_path: Whether to show path lines
            color: Color for route
        
        Returns:
            Folium Map object
        """
        if not route.waypoints:
            # Create empty map
            return folium.Map(location=[self.center_lat, self.center_lon], zoom_start=self.zoom_start)
        
        # Calculate center from waypoints
        center_lat = sum(wp.latitude for wp in route.waypoints) / len(route.waypoints)
        center_lon = sum(wp.longitude for wp in route.waypoints) / len(route.waypoints)
        
        # Create map
        m = folium.Map(location=[center_lat, center_lon], zoom_start=self.zoom_start)
        
        # Add path
        if show_path and len(route.waypoints) > 1:
            path_coords = [[wp.latitude, wp.longitude] for wp in route.waypoints]
            folium.PolyLine(
                path_coords,
                color=color,
                weight=3,
                opacity=0.7,
                popup=f"Route for {route.drone_name or 'Drone'}"
            ).add_to(m)
        
        # Add waypoints
        if show_waypoints:
            for idx, waypoint in enumerate(route.waypoints):
                marker_color = "green" if idx == 0 else ("red" if idx == len(route.waypoints) - 1 else "blue")
                icon = "play" if idx == 0 else ("stop" if idx == len(route.waypoints) - 1 else "info-sign")
                
                popup_text = f"Waypoint {idx}<br>"
                popup_text += f"Lat: {waypoint.latitude:.6f}<br>"
                popup_text += f"Lon: {waypoint.longitude:.6f}<br>"
                popup_text += f"Alt: {waypoint.altitude:.1f}m"
                if waypoint.name:
                    popup_text = f"<b>{waypoint.name}</b><br>" + popup_text
                
                folium.Marker(
                    location=[waypoint.latitude, waypoint.longitude],
                    popup=folium.Popup(popup_text, max_width=200),
                    icon=folium.Icon(color=marker_color, icon=icon, prefix="glyphicon"),
                    tooltip=f"WP {idx}: {waypoint.altitude:.0f}m"
                ).add_to(m)
        
        # Add metrics info
        if route.metrics:
            metrics_html = f"""
            <div style="position: fixed; 
                        top: 10px; right: 10px; width: 250px; height: auto; 
                        background-color: white; z-index:9999; 
                        padding: 10px; border: 2px solid {color}; border-radius: 5px;">
                <h4>Route Metrics</h4>
                <p><b>Distance:</b> {route.metrics.total_distance/1000:.2f} km</p>
                <p><b>Time:</b> {route.metrics.total_time/60:.1f} min</p>
                <p><b>Energy:</b> {route.metrics.total_energy:.2f} Wh</p>
                <p><b>Waypoints:</b> {route.metrics.waypoint_count}</p>
            </div>
            """
            m.get_root().html.add_child(folium.Element(metrics_html))
        
        return m
    
    def render_mission(self, mission: Mission) -> folium.Map:
        """Render complete mission with all routes and constraints.
        
        Args:
            mission: Mission to render
        
        Returns:
            Folium Map object
        """
        # Determine center
        if mission.depot:
            center_lat = mission.depot.latitude
            center_lon = mission.depot.longitude
        elif mission.target_points:
            center_lat = sum(tp.latitude for tp in mission.target_points) / len(mission.target_points)
            center_lon = sum(tp.longitude for tp in mission.target_points) / len(mission.target_points)
        else:
            center_lat = self.center_lat
            center_lon = self.center_lon
        
        m = folium.Map(location=[center_lat, center_lon], zoom_start=self.zoom_start)
        
        # Add no-fly zones
        if mission.constraints:
            for zone in mission.constraints.no_fly_zones:
                self._add_no_fly_zone(m, zone)
        
        # Add depot
        if mission.depot:
            folium.Marker(
                location=[mission.depot.latitude, mission.depot.longitude],
                popup=f"<b>Depot</b><br>Lat: {mission.depot.latitude:.6f}<br>Lon: {mission.depot.longitude:.6f}<br>Alt: {mission.depot.altitude:.1f}m",
                icon=folium.Icon(color="green", icon="home", prefix="glyphicon"),
                tooltip="Depot"
            ).add_to(m)
        
        # Add target points
        for idx, target in enumerate(mission.target_points):
            folium.Marker(
                location=[target.latitude, target.longitude],
                popup=f"<b>{target.name or f'Target {idx+1}'}</b><br>Lat: {target.latitude:.6f}<br>Lon: {target.longitude:.6f}<br>Alt: {target.altitude:.1f}m",
                icon=folium.Icon(color="red", icon="flag", prefix="glyphicon"),
                tooltip=target.name or f"Target {idx+1}"
            ).add_to(m)
        
        # Add routes with different colors
        colors = ["blue", "purple", "orange", "darkred", "lightred", "beige", "darkblue", "darkgreen"]
        for idx, (drone_name, route) in enumerate(mission.routes.items()):
            color = colors[idx % len(colors)]
            self._add_route_to_map(m, route, color)
        
        # Add weather visualization if available
        if weather_data:
            self._add_weather_visualization(m, weather_data)
        
        # Add layer control
        folium.LayerControl().add_to(m)
        
        return m
    
    def _add_weather_visualization(self, m: folium.Map, weather_data: Dict[tuple[float, float], WeatherConditions]):
        """Add weather visualization to map.
        
        Args:
            m: Folium map
            weather_data: Dictionary mapping (lat, lon) to WeatherConditions
        """
        # Create weather layer
        weather_group = folium.FeatureGroup(name="Weather Conditions")
        
        for (lat, lon), weather in weather_data.items():
            # Determine marker color based on conditions
            color = self._get_weather_color(weather)
            icon = self._get_weather_icon(weather)
            
            # Create popup with weather info
            popup_html = f"""
            <div style="font-family: Arial; width: 200px;">
                <h4>🌤️ Weather Conditions</h4>
                <p><b>Wind:</b> {weather.wind_speed_10m:.1f} m/s @ {weather.wind_direction_10m:.0f}°</p>
                <p><b>Temperature:</b> {weather.temperature_2m:.1f}°C</p>
                <p><b>Precipitation:</b> {weather.precipitation:.1f} mm</p>
                <p><b>Cloud Cover:</b> {weather.cloud_cover:.0f}%</p>
                {f'<p><b>Visibility:</b> {weather.visibility:.1f} km</p>' if weather.visibility else ''}
            </div>
            """
            
            # Add marker
            folium.Marker(
                location=[lat, lon],
                popup=folium.Popup(popup_html, max_width=250),
                icon=folium.Icon(color=color, icon=icon, prefix="fa"),
                tooltip=f"Wind: {weather.wind_speed_10m:.1f} m/s"
            ).add_to(weather_group)
            
            # Add wind direction arrow
            self._add_wind_arrow(m, lat, lon, weather.wind_direction_10m, weather.wind_speed_10m)
        
        weather_group.add_to(m)
    
    def _get_weather_color(self, weather: WeatherConditions) -> str:
        """Get marker color based on weather conditions."""
        # Red for unsafe conditions
        if weather.wind_speed_10m > 15 or weather.precipitation > 5:
            return "red"
        # Orange for marginal conditions
        elif weather.wind_speed_10m > 10 or weather.precipitation > 2:
            return "orange"
        # Green for good conditions
        else:
            return "green"
    
    def _get_weather_icon(self, weather: WeatherConditions) -> str:
        """Get icon based on weather conditions."""
        if weather.precipitation > 0:
            return "cloud-rain"
        elif weather.cloud_cover > 80:
            return "cloud"
        elif weather.cloud_cover > 50:
            return "cloud-sun"
        else:
            return "sun"
    
    def _add_wind_arrow(self, m: folium.Map, lat: float, lon: float, 
                       direction: float, speed: float):
        """Add wind direction arrow to map.
        
        Args:
            m: Folium map
            lat: Latitude
            lon: Longitude
            direction: Wind direction in degrees
            speed: Wind speed in m/s
        """
        import math
        
        # Calculate arrow endpoint
        arrow_length = min(speed * 0.001, 0.01)  # Scale arrow length
        direction_rad = math.radians(direction)
        
        end_lat = lat + arrow_length * math.cos(direction_rad)
        end_lon = lon + arrow_length * math.sin(direction_rad)
        
        # Create arrow
        folium.PolyLine(
            locations=[[lat, lon], [end_lat, end_lon]],
            color="blue",
            weight=2,
            opacity=0.6,
            tooltip=f"Wind: {speed:.1f} m/s @ {direction:.0f}°"
        ).add_to(m)
    
    def _add_no_fly_zone(self, m: folium.Map, zone: NoFlyZone):
        """Add no-fly zone to map."""
        # Convert Shapely geometry to GeoJSON
        import json
        from shapely.geometry import mapping
        
        geojson = {
            "type": "Feature",
            "geometry": mapping(zone.geometry),
            "properties": {
                "name": zone.name or "No-Fly Zone",
                "min_altitude": zone.min_altitude,
                "max_altitude": zone.max_altitude
            }
        }
        
        folium.GeoJson(
            geojson,
            style_function=lambda feature: {
                "fillColor": "red",
                "color": "red",
                "weight": 2,
                "fillOpacity": 0.3
            },
            popup=folium.Popup(
                f"<b>{zone.name or 'No-Fly Zone'}</b><br>"
                f"Altitude: {zone.min_altitude:.0f}m - {zone.max_altitude:.0f}m",
                max_width=200
            ),
            tooltip=zone.name or "No-Fly Zone"
        ).add_to(m)
    
    def _add_route_to_map(self, m: folium.Map, route: Route, color: str):
        """Add route to existing map."""
        if not route.waypoints:
            return
        
        # Add path
        if len(route.waypoints) > 1:
            path_coords = [[wp.latitude, wp.longitude] for wp in route.waypoints]
            folium.PolyLine(
                path_coords,
                color=color,
                weight=3,
                opacity=0.7,
                popup=f"Route for {route.drone_name or 'Drone'}"
            ).add_to(m)
        
        # Add waypoints
        for idx, waypoint in enumerate(route.waypoints):
            marker_color = "green" if idx == 0 else ("red" if idx == len(route.waypoints) - 1 else color)
            
            folium.CircleMarker(
                location=[waypoint.latitude, waypoint.longitude],
                radius=5,
                popup=f"WP {idx}: {waypoint.altitude:.0f}m",
                color=marker_color,
                fill=True,
                fillColor=marker_color
            ).add_to(m)

