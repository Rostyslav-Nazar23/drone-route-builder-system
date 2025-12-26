"""A* pathfinding algorithm implementation."""
from typing import List, Optional, Dict, Tuple
import heapq
import math
from app.environment.navigation_graph import NavigationGraph
from app.domain.waypoint import Waypoint


class AStar:
    """A* pathfinding algorithm."""
    
    def __init__(self, graph: NavigationGraph):
        """Initialize A* with navigation graph.
        
        Args:
            graph: NavigationGraph instance
        """
        self.graph = graph
    
    def find_path(self, start_node: str, goal_node: str) -> Optional[List[str]]:
        """Find path from start to goal using A* algorithm.
        
        Args:
            start_node: Start node ID
            goal_node: Goal node ID
        
        Returns:
            List of node IDs representing the path, or None if no path found
        """
        if not self.graph.has_node(start_node) or not self.graph.has_node(goal_node):
            return None
        
        # Priority queue: (f_score, node)
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        
        # Maps node -> came_from node
        came_from: Dict[str, Optional[str]] = {start_node: None}
        
        # g_score: cost from start to node
        g_score: Dict[str, float] = {start_node: 0.0}
        
        # f_score: g_score + heuristic
        f_score: Dict[str, float] = {start_node: self._heuristic(start_node, goal_node)}
        
        # Track speed at each node for inertia calculation
        node_speed: Dict[str, float] = {start_node: 0.0}  # Start from rest
        
        closed_set: set = set()
        
        while open_set:
            # Get node with lowest f_score
            current_f, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            # Check if we reached the goal
            if current == goal_node:
                # Reconstruct path
                path = []
                node = current
                while node is not None:
                    path.append(node)
                    node = came_from[node]
                path.reverse()
                return path
            
            # Explore neighbors
            for neighbor in self.graph.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                # Additional safety check: verify edge is still valid (includes no-fly zone check)
                # This ensures we don't use edges that might have been invalidated
                if hasattr(self.graph, 'cost_model') and self.graph.cost_model:
                    pos1 = self.graph.get_node_position(current)
                    pos2 = self.graph.get_node_position(neighbor)
                    is_valid, _ = self.graph.cost_model.is_valid_edge(
                        pos1[1], pos1[0], pos1[2],  # lat, lon, alt
                        pos2[1], pos2[0], pos2[2],
                        is_start_ground=False,
                        is_end_ground=False
                    )
                    if not is_valid:
                        continue  # Skip this edge if it's invalid (e.g., intersects no-fly zone)
                
                # Calculate tentative g_score with current speed for inertia
                current_speed_at_node = node_speed.get(current, 0.0)
                edge_weight = self.graph.get_edge_weight(current, neighbor, current_speed=current_speed_at_node)
                tentative_g = g_score[current] + edge_weight
                
                # Estimate speed at neighbor node (for next edge calculation)
                # Speed increases as we travel, up to max_speed
                if hasattr(self.graph, 'cost_model') and self.graph.cost_model:
                    pos1 = self.graph.get_node_position(current)
                    pos2 = self.graph.get_node_position(neighbor)
                    distance = self._euclidean_distance_3d(pos1, pos2)
                    
                    # Estimate final speed: accelerate from current_speed, but don't exceed max_speed
                    # Simplified: assume we reach effective speed based on distance
                    max_speed = self.graph.cost_model.drone.max_speed
                    acceleration = max_speed / 5.0  # Reach max speed in 5 seconds
                    time_to_travel = distance / max_speed if max_speed > 0 else 0
                    
                    # Estimate speed after traveling this edge
                    if time_to_travel > 0:
                        # Accelerate from current_speed
                        speed_gain = min(acceleration * time_to_travel, max_speed - current_speed_at_node)
                        estimated_speed = min(max_speed, current_speed_at_node + speed_gain)
                    else:
                        estimated_speed = current_speed_at_node
                else:
                    estimated_speed = 0.0
                
                # If this path to neighbor is better
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_node)
                    node_speed[neighbor] = estimated_speed  # Store estimated speed for this node
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        return None
    
    def find_path_to_waypoints(self, start_node: str, waypoint_nodes: List[str]) -> Optional[List[str]]:
        """Find path visiting multiple waypoints in order.
        
        Args:
            start_node: Start node ID
            waypoint_nodes: List of waypoint node IDs to visit
        
        Returns:
            Combined path, or None if any segment fails
        """
        if not waypoint_nodes:
            return [start_node]
        
        full_path = []
        current = start_node
        
        for waypoint in waypoint_nodes:
            segment = self.find_path(current, waypoint)
            if segment is None:
                return None
            
            # Add segment (avoid duplicating the current node)
            if full_path:
                full_path.extend(segment[1:])  # Skip first node (already in path)
            else:
                full_path.extend(segment)
            
            current = waypoint
        
        return full_path
    
    def path_to_waypoints(self, path_nodes: List[str]) -> List[Waypoint]:
        """Convert path node IDs to Waypoint objects.
        
        Args:
            path_nodes: List of node IDs
        
        Returns:
            List of Waypoint objects
        """
        return [self.graph.get_node_waypoint(node_id) for node_id in path_nodes]
    
    def _heuristic(self, node1: str, node2: str) -> float:
        """Heuristic function (Euclidean distance in 3D space).
        
        Args:
            node1: First node ID
            node2: Second node ID
        
        Returns:
            Estimated distance between nodes
        """
        pos1 = self.graph.get_node_position(node1)
        pos2 = self.graph.get_node_position(node2)
        
        # 3D Euclidean distance
        dx = pos1[0] - pos2[0]  # longitude
        dy = pos1[1] - pos2[1]  # latitude
        dz = pos1[2] - pos2[2]  # altitude
        
        # Convert lat/lon to meters (approximate)
        lat_m = dy * 111320.0
        lon_m = dx * 111320.0 * abs(pos1[1])  # Approximate, should use cos(lat)
        
        return (lat_m ** 2 + lon_m ** 2 + dz ** 2) ** 0.5
    
    def _euclidean_distance_3d(self, pos1: Tuple[float, float, float], pos2: Tuple[float, float, float]) -> float:
        """Calculate 3D Euclidean distance between two positions.
        
        Args:
            pos1: (lon, lat, alt) tuple
            pos2: (lon, lat, alt) tuple
        
        Returns:
            Distance in meters
        """
        # Convert lat/lon to meters
        lat1, lon1, alt1 = pos1[1], pos1[0], pos1[2]
        lat2, lon2, alt2 = pos2[1], pos2[0], pos2[2]
        
        lat_m = (lat2 - lat1) * 111320.0
        lon_m = (lon2 - lon1) * 111320.0 * abs(math.cos(math.radians(lat1)))
        alt_m = alt2 - alt1
        
        return math.sqrt(lat_m ** 2 + lon_m ** 2 + alt_m ** 2)

