"""Builder for navigation graphs."""
from typing import List, Tuple, Optional, Dict
import numpy as np
from app.domain.waypoint import Waypoint
from app.domain.constraints import MissionConstraints
from app.environment.navigation_graph import NavigationGraph
from app.environment.cost_model import CostModel
from app.domain.drone import Drone
from app.weather.weather_provider import WeatherConditions


class GraphBuilder:
    """Builder for creating navigation graphs."""
    
    def __init__(self, drone: Drone, constraints: Optional[MissionConstraints] = None,
                 weather_data: Optional[Dict[tuple[float, float], WeatherConditions]] = None):
        """Initialize graph builder.
        
        Args:
            drone: Drone capabilities
            constraints: Mission constraints
            weather_data: Dictionary mapping (lat, lon) to WeatherConditions (optional)
        """
        self.drone = drone
        self.constraints = constraints or MissionConstraints()
        self.weather_data = weather_data
        self.cost_model = CostModel(drone, constraints, weather_data)
    
    def build_grid_graph(self, 
                        center_lat: float,
                        center_lon: float,
                        width: float,
                        height: float,
                        resolution: float = 100.0,
                        min_altitude: float = 0.0,
                        max_altitude: float = 100.0,
                        altitude_levels: int = 5) -> NavigationGraph:
        """Build a 3D grid graph.
        
        Args:
            center_lat: Center latitude
            center_lon: Center longitude
            width: Width of grid in meters
            height: Height of grid in meters
            resolution: Grid resolution in meters
            min_altitude: Minimum altitude
            max_altitude: Maximum altitude
            altitude_levels: Number of altitude levels
        
        Returns:
            NavigationGraph instance
        """
        graph = NavigationGraph()
        
        # Calculate grid dimensions
        lat_per_meter = 1.0 / 111320.0  # Approximate
        lon_per_meter = 1.0 / (111320.0 * np.cos(np.radians(center_lat)))
        
        cols = int(width / resolution) + 1
        rows = int(height / resolution) + 1
        
        altitude_step = (max_altitude - min_altitude) / max(altitude_levels - 1, 1)
        
        # Create nodes
        node_id = 0
        for i in range(rows):
            for j in range(cols):
                for k in range(altitude_levels):
                    # Calculate position
                    lat_offset = (i - rows / 2) * resolution * lat_per_meter
                    lon_offset = (j - cols / 2) * resolution * lon_per_meter
                    altitude = min_altitude + k * altitude_step
                    
                    lat = center_lat + lat_offset
                    lon = center_lon + lon_offset
                    
                    node_key = f"n_{i}_{j}_{k}"
                    graph.add_node(node_key, lat, lon, altitude)
                    node_id += 1
        
        # Create edges (connect neighbors in 3D space)
        for i in range(rows):
            for j in range(cols):
                for k in range(altitude_levels):
                    node_key = f"n_{i}_{j}_{k}"
                    
                    # Horizontal neighbors
                    if j < cols - 1:
                        neighbor_key = f"n_{i}_{j+1}_{k}"
                        self._add_edge_if_valid(graph, node_key, neighbor_key)
                    
                    if i < rows - 1:
                        neighbor_key = f"n_{i+1}_{j}_{k}"
                        self._add_edge_if_valid(graph, node_key, neighbor_key)
                    
                    # Vertical neighbors (altitude changes)
                    if k < altitude_levels - 1:
                        neighbor_key = f"n_{i}_{j}_{k+1}"
                        self._add_edge_if_valid(graph, node_key, neighbor_key)
                    
                    if k > 0:
                        neighbor_key = f"n_{i}_{j}_{k-1}"
                        self._add_edge_if_valid(graph, node_key, neighbor_key)
        
        return graph
    
    def build_waypoint_graph(self, waypoints: List[Waypoint], 
                            connect_all: bool = False,
                            max_distance: Optional[float] = None) -> NavigationGraph:
        """Build graph from waypoints.
        
        Args:
            waypoints: List of waypoints
            connect_all: If True, connect all waypoints (fully connected)
            max_distance: Maximum distance for connections (meters)
        
        Returns:
            NavigationGraph instance
        """
        graph = NavigationGraph()
        
        # Add all waypoints as nodes
        for idx, wp in enumerate(waypoints):
            node_id = f"wp_{idx}"
            graph.add_node(node_id, wp.latitude, wp.longitude, wp.altitude)
        
        # Connect waypoints
        for i, wp1 in enumerate(waypoints):
            node1_id = f"wp_{i}"
            for j, wp2 in enumerate(waypoints):
                if i == j:
                    continue
                
                node2_id = f"wp_{j}"
                
                # Check distance constraint
                if max_distance is not None:
                    distance = self.cost_model.calculate_distance(
                        wp1.latitude, wp1.longitude, wp1.altitude,
                        wp2.latitude, wp2.longitude, wp2.altitude
                    )
                    if distance > max_distance:
                        continue
                
                # Add edge if valid
                if connect_all or self._should_connect(wp1, wp2):
                    self._add_edge_if_valid(graph, node1_id, node2_id)
        
        return graph
    
    def _should_connect(self, wp1: Waypoint, wp2: Waypoint) -> bool:
        """Determine if two waypoints should be connected.
        Simple heuristic: connect if within reasonable distance."""
        distance = self.cost_model.calculate_distance(
            wp1.latitude, wp1.longitude, wp1.altitude,
            wp2.latitude, wp2.longitude, wp2.altitude
        )
        # Connect if within drone's max range
        return distance <= self.drone.max_range
    
    def _add_edge_if_valid(self, graph: NavigationGraph, node1: str, node2: str):
        """Add edge to graph if it's valid."""
        pos1 = graph.get_node_position(node1)
        pos2 = graph.get_node_position(node2)
        
        # Check if edge is valid
        is_valid, error = self.cost_model.is_valid_edge(
            pos1[1], pos1[0], pos1[2],  # lat, lon, alt
            pos2[1], pos2[0], pos2[2]
        )
        
        if is_valid:
            cost = self.cost_model.calculate_cost(
                pos1[1], pos1[0], pos1[2],
                pos2[1], pos2[0], pos2[2]
            )
            graph.add_edge(node1, node2, cost)
    
    def find_nearest_node(self, graph: NavigationGraph, 
                         latitude: float, longitude: float, altitude: float) -> Optional[str]:
        """Find nearest node in graph to given coordinates."""
        min_distance = float('inf')
        nearest_node = None
        
        for node_id in graph.nodes():
            pos = graph.get_node_position(node_id)
            distance = self.cost_model.calculate_distance(
                latitude, longitude, altitude,
                pos[1], pos[0], pos[2]  # lat, lon, alt
            )
            
            if distance < min_distance:
                min_distance = distance
                nearest_node = node_id
        
        return nearest_node

