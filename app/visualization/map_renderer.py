"""Map renderer for visualizing routes."""
from typing import List, Optional, Dict
import folium
from folium import plugins
from app.domain.route import Route
from app.domain.waypoint import Waypoint
from app.domain.constraints import MissionConstraints, NoFlyZone
from app.domain.mission import Mission
from app.weather.weather_provider import WeatherConditions
from app.environment.navigation_graph import NavigationGraph


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
        
        # Add path - use AntPath for all routes (shows flight direction with animated dashed line)
        if show_path and len(route.waypoints) > 1:
            path_coords = [[wp.latitude, wp.longitude] for wp in route.waypoints]
            
            # Use AntPath for all routes (more informative, shows flight direction)
            try:
                from folium.plugins import AntPath
                AntPath(
                    path_coords,
                    color=color,
                    weight=3,
                    opacity=0.7,
                    dash_array=[10, 20],
                    delay=1000,
                    popup=f"Route for {route.drone_name or 'Drone'}"
                ).add_to(m)
            except ImportError:
                # Fallback to PolyLine if AntPath not available
                folium.PolyLine(
                    path_coords,
                    color=color,
                    weight=3,
                    opacity=0.7,
                    popup=f"Route for {route.drone_name or 'Drone'}"
                ).add_to(m)
        
        # Add waypoints with clear start/finish markers
        # Filter out intermediate waypoints from markers (they're shown in the path line)
        if show_waypoints:
            # Find actual start and finish indices (excluding intermediate)
            actual_waypoints = [(idx, wp) for idx, wp in enumerate(route.waypoints) if wp.waypoint_type != "intermediate"]
            
            for idx, waypoint in enumerate(route.waypoints):
                # Skip intermediate waypoints - they're only for path smoothing
                if waypoint.waypoint_type == "intermediate":
                    continue
                
                # Find position in actual_waypoints list
                actual_idx = next(i for i, (orig_idx, wp) in enumerate(actual_waypoints) if orig_idx == idx)
                is_start = actual_idx == 0
                is_finish = actual_idx == len(actual_waypoints) - 1
                
                popup_text = f"Waypoint {idx}<br>"
                popup_text += f"Lat: {waypoint.latitude:.6f}<br>"
                popup_text += f"Lon: {waypoint.longitude:.6f}<br>"
                popup_text += f"Alt: {waypoint.altitude:.1f}m"
                if waypoint.name:
                    popup_text = f"<b>{waypoint.name}</b><br>" + popup_text
                
                if is_start:
                    # START marker - зелений
                    folium.Marker(
                        location=[waypoint.latitude, waypoint.longitude],
                        popup=folium.Popup(f"<b>🏁 START</b><br>" + popup_text, max_width=200),
                        icon=folium.Icon(color="green", icon="play", prefix="glyphicon"),
                        tooltip=f"🏁 START - WP {idx}"
                    ).add_to(m)
                    folium.CircleMarker(
                        location=[waypoint.latitude, waypoint.longitude],
                        radius=12,
                        popup="START",
                        color="green",
                        fill=True,
                        fillColor="green",
                        fillOpacity=0.3,
                        weight=4
                    ).add_to(m)
                elif is_finish:
                    # FINISH marker - червоний
                    folium.Marker(
                        location=[waypoint.latitude, waypoint.longitude],
                        popup=folium.Popup(f"<b>🏁 FINISH</b><br>" + popup_text, max_width=200),
                        icon=folium.Icon(color="red", icon="stop", prefix="glyphicon"),
                        tooltip=f"🏁 FINISH - WP {idx}"
                    ).add_to(m)
                    folium.CircleMarker(
                        location=[waypoint.latitude, waypoint.longitude],
                        radius=12,
                        popup="FINISH",
                        color="red",
                        fill=True,
                        fillColor="red",
                        fillOpacity=0.3,
                        weight=4
                    ).add_to(m)
                else:
                    # Intermediate waypoint
                    folium.Marker(
                        location=[waypoint.latitude, waypoint.longitude],
                        popup=folium.Popup(popup_text, max_width=200),
                        icon=folium.Icon(color=color, icon="info-sign", prefix="glyphicon"),
                        tooltip=f"WP {idx}: {waypoint.altitude:.0f}m"
                    ).add_to(m)
                    folium.CircleMarker(
                        location=[waypoint.latitude, waypoint.longitude],
                        radius=6,
                        popup=f"WP {idx}",
                        color=color,
                        fill=True,
                        fillColor=color,
                        fillOpacity=0.5,
                        weight=2
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
    
    def render_mission(self, mission: Mission, 
                      weather_data: Optional[Dict[tuple[float, float], WeatherConditions]] = None) -> folium.Map:
        """Render complete mission with all routes and constraints.
        
        Args:
            mission: Mission to render
            weather_data: Optional dictionary mapping (lat, lon) to WeatherConditions
        
        Returns:
            Folium Map object
        """
        # Determine center and fit bounds to show all points
        all_points = []
        if mission.depot:
            all_points.append([mission.depot.latitude, mission.depot.longitude])
        if mission.finish_point:
            all_points.append([mission.finish_point.latitude, mission.finish_point.longitude])
        for tp in mission.target_points:
            all_points.append([tp.latitude, tp.longitude])
        
        if all_points:
            # Calculate center from all points
            center_lat = sum(p[0] for p in all_points) / len(all_points)
            center_lon = sum(p[1] for p in all_points) / len(all_points)
            # Use a zoom level that shows all points
            # If we have multiple points, we'll fit bounds later
            if len(all_points) == 1:
                zoom_start = 15  # Closer zoom for single point
            else:
                zoom_start = self.zoom_start
        else:
            center_lat = self.center_lat
            center_lon = self.center_lon
            zoom_start = self.zoom_start
        
        m = folium.Map(location=[center_lat, center_lon], zoom_start=zoom_start)
        
        # Add no-fly zones
        if mission.constraints:
            for zone in mission.constraints.no_fly_zones:
                self._add_no_fly_zone(m, zone)
        
        # Add depot (START) - check if it overlaps with finish point
        depot_added = False
        depot_key = None
        finish_key = None
        
        if mission.depot:
            depot_key = (round(mission.depot.latitude, 4), round(mission.depot.longitude, 4))
        
        if mission.finish_point:
            finish_key = (round(mission.finish_point.latitude, 4), round(mission.finish_point.longitude, 4))
        
        if mission.depot:
            # Check if depot and finish are at the same location
            if finish_key and depot_key == finish_key:
                # Combined START & FINISH marker
                folium.Marker(
                    location=[mission.depot.latitude, mission.depot.longitude],
                    popup=f"<b>🏁 START & FINISH (Depot)</b><br>Lat: {mission.depot.latitude:.6f}<br>Lon: {mission.depot.longitude:.6f}<br>Alt: {mission.depot.altitude:.1f}m",
                    icon=folium.Icon(color="purple", icon="home", prefix="glyphicon"),
                    tooltip="🏁 START & FINISH - Depot"
                ).add_to(m)
                folium.CircleMarker(
                    location=[mission.depot.latitude, mission.depot.longitude],
                    radius=12,
                    popup="START & FINISH",
                    color="purple",
                    fill=True,
                    fillColor="purple",
                    fillOpacity=0.3,
                    weight=4
                ).add_to(m)
                depot_added = True
            else:
                # Just START marker
                folium.Marker(
                    location=[mission.depot.latitude, mission.depot.longitude],
                    popup=f"<b>🏁 START (Depot)</b><br>Lat: {mission.depot.latitude:.6f}<br>Lon: {mission.depot.longitude:.6f}<br>Alt: {mission.depot.altitude:.1f}m",
                    icon=folium.Icon(color="green", icon="play", prefix="glyphicon"),
                    tooltip="🏁 START - Depot"
                ).add_to(m)
                folium.CircleMarker(
                    location=[mission.depot.latitude, mission.depot.longitude],
                    radius=10,
                    popup="START",
                    color="green",
                    fill=True,
                    fillColor="green",
                    fillOpacity=0.3,
                    weight=3
                ).add_to(m)
                depot_added = True
        
        # Add finish point (if not already added as combined with depot)
        if mission.finish_point and not (depot_added and finish_key and depot_key == finish_key):
            folium.Marker(
                location=[mission.finish_point.latitude, mission.finish_point.longitude],
                popup=f"<b>🏁 FINISH</b><br>Lat: {mission.finish_point.latitude:.6f}<br>Lon: {mission.finish_point.longitude:.6f}<br>Alt: {mission.finish_point.altitude:.1f}m",
                icon=folium.Icon(color="red", icon="stop", prefix="glyphicon"),
                tooltip="🏁 FINISH"
            ).add_to(m)
            folium.CircleMarker(
                location=[mission.finish_point.latitude, mission.finish_point.longitude],
                radius=10,
                popup="FINISH",
                color="red",
                fill=True,
                fillColor="red",
                fillOpacity=0.3,
                weight=3
            ).add_to(m)
        
        # Add target points - червоні маркери with weather info
        # Skip target points that are the same as depot or finish point to avoid overlapping markers
        target_index = 0  # Track actual target index (excluding skipped ones)
        for target in mission.target_points:
            # Check if this target point is the same as depot (within small tolerance)
            is_depot = False
            if mission.depot:
                depot_key = (round(mission.depot.latitude, 4), round(mission.depot.longitude, 4))
                target_key = (round(target.latitude, 4), round(target.longitude, 4))
                if depot_key == target_key:
                    is_depot = True
            
            # Check if this target point is the same as finish point (within small tolerance)
            is_finish = False
            if mission.finish_point:
                finish_key = (round(mission.finish_point.latitude, 4), round(mission.finish_point.longitude, 4))
                target_key = (round(target.latitude, 4), round(target.longitude, 4))
                if finish_key == target_key:
                    is_finish = True
            
            # Skip this target if it's the same as depot or finish (they're already shown as START/FINISH)
            if is_depot or is_finish:
                continue
            
            # Increment target index only for displayed targets
            target_index += 1
            
            # Get weather info for this target if available
            weather_info = ""
            if weather_data:
                target_key = (target.latitude, target.longitude)
                # Try exact match first
                weather = weather_data.get(target_key)
                if not weather:
                    # Find closest weather data
                    min_dist = float('inf')
                    closest_weather = None
                    for (lat, lon), w in weather_data.items():
                        dist = ((lat - target.latitude)**2 + (lon - target.longitude)**2)**0.5
                        if dist < min_dist:
                            min_dist = dist
                            closest_weather = w
                    if closest_weather and min_dist < 0.01:  # Within ~1km
                        weather = closest_weather
                
                if weather:
                    weather_info = f"<br><br><b>🌤️ Weather:</b><br>"
                    weather_info += f"Wind: {weather.wind_speed_10m:.1f} m/s @ {weather.wind_direction_10m:.0f}°<br>"
                    weather_info += f"Temp: {weather.temperature_2m:.1f}°C<br>"
                    weather_info += f"Precip: {weather.precipitation:.1f} mm<br>"
                    weather_info += f"Clouds: {weather.cloud_cover:.0f}%"
                    if weather.visibility:
                        weather_info += f"<br>Visibility: {weather.visibility:.1f} km"
            
            folium.Marker(
                location=[target.latitude, target.longitude],
                popup=f"<b>🎯 Target {target_index}: {target.name or 'Unnamed'}</b><br>Lat: {target.latitude:.6f}<br>Lon: {target.longitude:.6f}<br>Alt: {target.altitude:.1f}m{weather_info}",
                icon=folium.Icon(color="red", icon="flag", prefix="fa"),
                tooltip=f"🎯 Target {target_index}"
            ).add_to(m)
            
            # Додати коло для виділення
            folium.CircleMarker(
                location=[target.latitude, target.longitude],
                radius=8,
                popup=f"Target {target_index}",
                color="red",
                fill=True,
                fillColor="red",
                fillOpacity=0.2,
                weight=2
            ).add_to(m)
        
        # Add routes with different colors (avoid cyan to differentiate from wind arrows)
        colors = ["blue", "purple", "orange", "darkred", "lightred", "beige", "darkblue", "darkgreen", "red", "green"]
        landing_mode = getattr(mission, 'landing_mode', 'vertical')
        for idx, (drone_name, route) in enumerate(mission.routes.items()):
            color = colors[idx % len(colors)]
            self._add_route_to_map(m, route, color, landing_mode=landing_mode)
        
        # Add weather visualization (wind arrows only, weather info is in target markers)
        if weather_data:
            # Only add wind arrows, not separate weather markers
            for (lat, lon), weather in weather_data.items():
                self._add_wind_arrow(m, lat, lon, weather.wind_direction_10m, weather.wind_speed_10m)
        
        # Fit map bounds to show all points (depot, targets, finish)
        if all_points:
            # Add some padding
            bounds = [[min(p[0] for p in all_points) - 0.01, min(p[1] for p in all_points) - 0.01],
                     [max(p[0] for p in all_points) + 0.01, max(p[1] for p in all_points) + 0.01]]
            m.fit_bounds(bounds, padding=(20, 20))
        
        # Add layer control
        folium.LayerControl().add_to(m)
        
        # Add mouse coordinate display
        self._add_mouse_coordinates(m)
        
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
        """Add wind direction arrow to map with arrowhead.
        
        Args:
            m: Folium map
            lat: Latitude
            lon: Longitude
            direction: Wind direction in degrees (where wind is coming FROM)
            speed: Wind speed in m/s
        """
        import math
        
        # Wind direction is where wind comes FROM, so arrow should point opposite
        # Arrow points in the direction wind is GOING
        arrow_direction = (direction + 180) % 360
        direction_rad = math.radians(arrow_direction)
        
        # Make arrow length proportional to wind speed, with better scaling
        # Base length of 0.02 degrees (~2km), scaled by speed
        base_length = 0.02
        speed_factor = min(speed / 10.0, 2.0)  # Cap at 2x for very strong winds
        arrow_length = base_length * (0.5 + speed_factor * 0.5)
        
        # Calculate arrow endpoint
        end_lat = lat + arrow_length * math.cos(direction_rad)
        end_lon = lon + arrow_length * math.sin(direction_rad)
        
        # Determine color based on wind speed
        if speed > 15:
            arrow_color = "red"
        elif speed > 10:
            arrow_color = "orange"
        else:
            arrow_color = "cyan"
        
        # Create arrow line (thicker and more visible)
        folium.PolyLine(
            locations=[[lat, lon], [end_lat, end_lon]],
            color=arrow_color,
            weight=4,
            opacity=0.9,
            tooltip=f"Wind: {speed:.1f} m/s @ {direction:.0f}° (from)"
        ).add_to(m)
        
        # Calculate arrowhead points (larger and more visible)
        arrowhead_length = arrow_length * 0.4  # 40% of arrow length
        arrowhead_width = arrow_length * 0.2   # 20% of arrow length
        
        # Perpendicular direction for arrowhead
        perp_rad = direction_rad + math.pi / 2
        
        # Arrowhead tip (at end of arrow)
        tip_lat = end_lat
        tip_lon = end_lon
        
        # Arrowhead base points (wider base)
        base1_lat = end_lat - arrowhead_length * math.cos(direction_rad) + arrowhead_width * math.cos(perp_rad)
        base1_lon = end_lon - arrowhead_length * math.sin(direction_rad) + arrowhead_width * math.sin(perp_rad)
        
        base2_lat = end_lat - arrowhead_length * math.cos(direction_rad) - arrowhead_width * math.cos(perp_rad)
        base2_lon = end_lon - arrowhead_length * math.sin(direction_rad) - arrowhead_width * math.sin(perp_rad)
        
        # Draw arrowhead triangle (filled and more visible)
        folium.Polygon(
            locations=[[tip_lat, tip_lon], [base1_lat, base1_lon], [base2_lat, base2_lon], [tip_lat, tip_lon]],
            color=arrow_color,
            fill=True,
            fillColor=arrow_color,
            fillOpacity=0.9,
            weight=3
        ).add_to(m)
    
    def _add_mouse_coordinates(self, m: folium.Map):
        """Add mouse coordinate display to map.
        
        Args:
            m: Folium map
        """
        # Create HTML for moving coordinate hint only
        coordinate_html = """
        <div id="mouse-coord-hint" style="
            position: fixed;
            pointer-events: none;
            background-color: rgba(0, 0, 0, 0.85);
            color: #fff;
            padding: 5px 10px;
            border-radius: 4px;
            font-family: 'Courier New', monospace;
            font-size: 11px;
            z-index: 2000;
            display: none;
            white-space: nowrap;
            box-shadow: 0 2px 6px rgba(0,0,0,0.5);
        ">
            <span id="hint-lat">--</span>, <span id="hint-lon">--</span>
        </div>
        """
        
        # Add the HTML element to the map
        m.get_root().html.add_child(folium.Element(coordinate_html))
        
        # Add JavaScript to track mouse movement
        # Use a simpler, more reliable approach
        mouse_track_js = """
        <script>
        (function() {
            function initCoordinateTracking() {
                var hintDiv = document.getElementById('mouse-coord-hint');
                var hintLat = document.getElementById('hint-lat');
                var hintLon = document.getElementById('hint-lon');
                
                if (!hintDiv) {
                    setTimeout(initCoordinateTracking, 200);
                    return;
                }
                
                // Find all Leaflet map containers
                var mapContainers = document.querySelectorAll('.leaflet-container');
                if (mapContainers.length === 0) {
                    setTimeout(initCoordinateTracking, 200);
                    return;
                }
                
                // Get the first map instance
                var mapContainer = mapContainers[0];
                var map = null;
                
                // Try to get map from various possible locations
                if (mapContainer._leaflet_id) {
                    map = window.L && window.L._mapInstances && window.L._mapInstances[mapContainer._leaflet_id];
                }
                
                if (!map) {
                    // Try alternative method
                    for (var key in window) {
                        if (window[key] && window[key]._container === mapContainer) {
                            map = window[key];
                            break;
                        }
                    }
                }
                
                if (!map) {
                    setTimeout(initCoordinateTracking, 300);
                    return;
                }
                
                // Function to update coordinates
                function updateCoordinates(e) {
                    if (!e || !e.latlng) return;
                    
                    var lat = e.latlng.lat.toFixed(6);
                    var lon = e.latlng.lng.toFixed(6);
                    
                    // Update moving hint
                    if (hintLat) hintLat.textContent = lat;
                    if (hintLon) hintLon.textContent = lon;
                    
                    // Position hint near mouse cursor
                    if (hintDiv && e.originalEvent) {
                        hintDiv.style.display = 'block';
                        hintDiv.style.left = (e.originalEvent.clientX + 15) + 'px';
                        hintDiv.style.top = (e.originalEvent.clientY + 15) + 'px';
                    }
                }
                
                // Function to hide hint when mouse leaves map
                function hideHint() {
                    if (hintDiv) {
                        hintDiv.style.display = 'none';
                    }
                }
                
                // Add event listeners to map
                map.on('mousemove', updateCoordinates);
                map.on('mouseout', hideHint);
                
                // Also listen to map container events for better coverage
                mapContainer.addEventListener('mousemove', function(e) {
                    try {
                        var latlng = map.mouseEventToLatLng(e);
                        if (latlng) {
                            var event = {latlng: latlng, originalEvent: e};
                            updateCoordinates(event);
                        }
                    } catch(err) {
                        // Ignore errors
                    }
                });
                mapContainer.addEventListener('mouseleave', hideHint);
            }
            
            // Start initialization after a delay to ensure map is loaded
            setTimeout(initCoordinateTracking, 1000);
        })();
        </script>
        """
        
        # Add JavaScript to the map
        m.get_root().script.add_child(folium.Element(mouse_track_js))
    
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
    
    def _add_route_to_map(self, m: folium.Map, route: Route, color: str, landing_mode: str = "vertical"):
        """Add route to existing map."""
        if not route.waypoints:
            return
        
        # Add path - use AntPath for all routes (shows flight direction with animated dashed line)
        if len(route.waypoints) > 1:
            path_coords = [[wp.latitude, wp.longitude] for wp in route.waypoints]
            
            # Use AntPath for all routes (more informative, shows flight direction)
            try:
                from folium.plugins import AntPath
                AntPath(
                    path_coords,
                    color=color,
                    weight=4,
                    opacity=0.8,
                    dash_array=[10, 20],
                    delay=1000,
                    popup=f"Route for {route.drone_name or 'Drone'}"
                ).add_to(m)
            except ImportError:
                # Fallback to PolyLine if AntPath not available
                folium.PolyLine(
                    path_coords,
                    color=color,
                    weight=4,
                    opacity=0.8,
                    popup=f"Route for {route.drone_name or 'Drone'}"
                ).add_to(m)
        
        # Group waypoints by location to merge overlapping markers
        waypoint_groups = {}  # (lat, lon) -> list of (idx, waypoint, type_info)
        
        for idx, waypoint in enumerate(route.waypoints):
            is_start = idx == 0
            is_finish = idx == len(route.waypoints) - 1
            is_intermediate = waypoint.waypoint_type == "intermediate"
            # Hide landing_segment markers for vertical landing
            is_landing_segment = waypoint.waypoint_type == "landing_segment" and landing_mode != "vertical"
            is_landing_approach = waypoint.waypoint_type == "landing_approach"
            
            # Skip landing_segment waypoints for vertical landing (don't show markers)
            if waypoint.waypoint_type == "landing_segment" and landing_mode == "vertical":
                continue
            
            # Round coordinates to avoid floating point precision issues (0.0001 degrees ≈ 11m)
            key = (round(waypoint.latitude, 4), round(waypoint.longitude, 4))
            
            if key not in waypoint_groups:
                waypoint_groups[key] = []
            
            waypoint_groups[key].append({
                'idx': idx,
                'waypoint': waypoint,
                'is_start': is_start,
                'is_finish': is_finish,
                'is_intermediate': is_intermediate,
                'is_landing_segment': is_landing_segment,
                'is_landing_approach': is_landing_approach
            })
        
        # Add markers for each group (merged if overlapping)
        for key, group in waypoint_groups.items():
            lat, lon = key
            waypoint = group[0]['waypoint']  # Use first waypoint's coordinates
            
            # Determine marker type and create combined popup
            is_start = any(w['is_start'] for w in group)
            is_finish = any(w['is_finish'] for w in group)
            is_landing_approach = any(w['is_landing_approach'] for w in group)
            # Hide landing_segment markers for vertical landing
            is_landing_segment = any(w['is_landing_segment'] for w in group) and landing_mode != "vertical"
            is_intermediate = all(w['is_intermediate'] for w in group)
            
            # Skip groups that only contain landing_segment waypoints for vertical landing
            if landing_mode == "vertical" and all(w['waypoint'].waypoint_type == "landing_segment" for w in group):
                continue
            
            # Build combined popup text
            popup_parts = []
            if is_start and is_finish:
                popup_parts.append("<b>🏁 START & FINISH</b>")
                icon_color = "purple"
                icon_name = "home"
            elif is_start:
                popup_parts.append("<b>🏁 START</b>")
                icon_color = "green"
                icon_name = "play"
            elif is_finish:
                popup_parts.append("<b>🏁 FINISH</b>")
                icon_color = "red"
                icon_name = "stop"
            elif is_landing_approach:
                popup_parts.append("<b>⬇️ Landing Approach</b>")
                icon_color = "orange"
                icon_name = "arrow-down"
            elif is_landing_segment:
                popup_parts.append("<b>⬇️ Landing Segment</b>")
                icon_color = "orange"
                icon_name = "arrow-down"
            else:
                icon_color = color
                icon_name = "info-sign"
            
            # Add waypoint info
            if len(group) == 1:
                wp_info = group[0]
                popup_parts.append(f"WP {wp_info['idx']}")
                popup_parts.append(f"Lat: {waypoint.latitude:.6f}")
                popup_parts.append(f"Lon: {waypoint.longitude:.6f}")
                popup_parts.append(f"Alt: {waypoint.altitude:.0f}m")
                if waypoint.name:
                    popup_parts.insert(1, f"<b>{waypoint.name}</b>")
            else:
                # Multiple waypoints at same location
                popup_parts.append(f"<b>Multiple waypoints ({len(group)})</b>")
                for wp_info in group:
                    popup_parts.append(f"<br>• WP {wp_info['idx']}: Alt {wp_info['waypoint'].altitude:.0f}m")
                popup_parts.append(f"<br>Location: Lat {waypoint.latitude:.6f}, Lon {waypoint.longitude:.6f}")
            
            popup_text = "<br>".join(popup_parts)
            
            # Add marker
            if is_intermediate:
                # Small marker for intermediate waypoints (increased size)
                folium.CircleMarker(
                    location=[lat, lon],
                    radius=8,  # Increased from 4 to 8
                    popup=folium.Popup(popup_text, max_width=250),
                    color=color,
                    fill=True,
                    fillColor=color,
                    fillOpacity=0.6,
                    weight=2,  # Increased from 1 to 2
                    tooltip=f"WP {group[0]['idx']}" + (" (merged)" if len(group) > 1 else "")
                ).add_to(m)
            else:
                # Regular marker
                folium.Marker(
                    location=[lat, lon],
                    popup=folium.Popup(popup_text, max_width=250),
                    icon=folium.Icon(color=icon_color, icon=icon_name, prefix="glyphicon"),
                    tooltip=f"WP {group[0]['idx']}" + (" (merged)" if len(group) > 1 else "")
                ).add_to(m)
                
                # Add circle marker for emphasis (only for start/finish/landing) - increased size
                if is_start or is_finish or is_landing_approach:
                    radius = 16 if (is_start or is_finish) else 12  # Increased from 12/8 to 16/12
                    circle_color = icon_color
                    folium.CircleMarker(
                        location=[lat, lon],
                        radius=radius,
                        popup=popup_text,
                        color=circle_color,
                        fill=True,
                        fillColor=circle_color,
                        fillOpacity=0.3 if (is_start or is_finish) else 0.4,
                        weight=4 if (is_start or is_finish) else 2
                    ).add_to(m)

