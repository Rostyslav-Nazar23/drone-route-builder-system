"""Theta* pathfinding algorithm implementation."""
from typing import List, Optional, Dict, Tuple, Set
import heapq
import math
from app.environment.navigation_graph import NavigationGraph
from app.domain.waypoint import Waypoint


class ThetaStar:
    """Theta* pathfinding algorithm - any-angle pathfinding."""
    
    def __init__(self, graph: NavigationGraph):
        """Initialize Theta* with navigation graph.
        
        Args:
            graph: NavigationGraph instance
        """
        self.graph = graph
    
    def find_path(self, start_node: str, goal_node: str) -> Optional[List[str]]:
        """Find path from start to goal using Theta* algorithm.
        
        Theta* allows any-angle paths by checking line-of-sight between nodes.
        
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
        
        closed_set: Set[str] = set()
        
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
            
            # Get parent of current node
            parent = came_from[current]
            
            # Explore neighbors
            for neighbor in self.graph.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                # Theta*: Check line-of-sight from parent to neighbor
                if parent is not None and self._line_of_sight(parent, neighbor):
                    # Path through parent
                    tentative_g = g_score[parent] + self._direct_cost(parent, neighbor)
                else:
                    # Path through current node
                    tentative_g = g_score[current] + self.graph.get_edge_weight(current, neighbor)
                
                # If this path to neighbor is better
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = parent if (parent is not None and self._line_of_sight(parent, neighbor)) else current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_node)
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
    
    def _line_of_sight(self, node1: str, node2: str) -> bool:
        """Check if there's a direct line-of-sight between two nodes.
        
        In Theta*, this means checking if the direct path doesn't intersect obstacles.
        For now, we use a simplified check based on distance and intermediate nodes.
        
        Args:
            node1: First node ID
            node2: Second node ID
        
        Returns:
            True if line-of-sight exists
        """
        pos1 = self.graph.get_node_position(node1)
        pos2 = self.graph.get_node_position(node2)
        
        # Calculate distance
        distance = self._euclidean_distance_3d(pos1, pos2)
        
        # If nodes are very close, assume line-of-sight
        if distance < 100:  # 100 meters
            return True
        
        # Check if there are intermediate nodes that might block the path
        # Simplified: if distance is reasonable and nodes are connected in graph, allow
        # In a full implementation, we'd check against obstacles/no-fly zones
        
        # For now, allow line-of-sight if nodes are within reasonable distance
        # and there's a path in the graph (even if indirect)
        return distance < 5000  # 5km max line-of-sight
    
    def _direct_cost(self, node1: str, node2: str) -> float:
        """Calculate direct cost between two nodes (Euclidean distance).
        
        Args:
            node1: First node ID
            node2: Second node ID
        
        Returns:
            Direct cost
        """
        pos1 = self.graph.get_node_position(node1)
        pos2 = self.graph.get_node_position(node2)
        return self._euclidean_distance_3d(pos1, pos2)
    
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
        return self._euclidean_distance_3d(pos1, pos2)
    
    @staticmethod
    def _euclidean_distance_3d(pos1: Tuple[float, float, float], 
                                pos2: Tuple[float, float, float]) -> float:
        """Calculate 3D Euclidean distance.
        
        Args:
            pos1: (lon, lat, alt) tuple
            pos2: (lon, lat, alt) tuple
        
        Returns:
            Distance in meters
        """
        # Convert lat/lon to meters (approximate)
        lat1, lon1, alt1 = pos1[1], pos1[0], pos1[2]
        lat2, lon2, alt2 = pos2[1], pos2[0], pos2[2]
        
        # Horizontal distance
        lat_m = (lat2 - lat1) * 111320.0
        lon_m = (lon2 - lon1) * 111320.0 * abs(math.cos(math.radians(lat1)))
        
        # Vertical distance
        alt_m = alt2 - alt1
        
        return math.sqrt(lat_m ** 2 + lon_m ** 2 + alt_m ** 2)

