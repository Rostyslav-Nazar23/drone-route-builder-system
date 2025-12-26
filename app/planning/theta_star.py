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
            graph: NavigationGraph instance (should have cost_model for inertia/wind)
        """
        self.graph = graph
        self.cost_model = graph.cost_model  # Get CostModel for direct cost calculation
    
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
        
        # Track speed at each node for inertia calculation
        node_speed: Dict[str, float] = {start_node: 0.0}  # Start from rest
        
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
                    # Path through parent - use speed at parent for inertia calculation
                    parent_speed = node_speed.get(parent, 0.0)
                    tentative_g = g_score[parent] + self._direct_cost(parent, neighbor, parent_speed)
                    
                    # Estimate speed at neighbor after traveling from parent
                    if self.cost_model:
                        parent_pos = self.graph.get_node_position(parent)
                        neighbor_pos = self.graph.get_node_position(neighbor)
                        distance = self._euclidean_distance_3d(parent_pos, neighbor_pos)
                        max_speed = self.cost_model.drone.max_speed
                        acceleration = max_speed / 5.0
                        time_to_travel = distance / max_speed if max_speed > 0 else 0
                        
                        if time_to_travel > 0:
                            speed_gain = min(acceleration * time_to_travel, max_speed - parent_speed)
                            estimated_speed = min(max_speed, parent_speed + speed_gain)
                        else:
                            estimated_speed = parent_speed
                    else:
                        estimated_speed = parent_speed
                else:
                    # Path through current node - use current speed for inertia
                    current_speed_at_node = node_speed.get(current, 0.0)
                    tentative_g = g_score[current] + self.graph.get_edge_weight(current, neighbor, current_speed=current_speed_at_node)
                    
                    # Estimate speed at neighbor after traveling from current
                    if self.cost_model:
                        current_pos = self.graph.get_node_position(current)
                        neighbor_pos = self.graph.get_node_position(neighbor)
                        distance = self._euclidean_distance_3d(current_pos, neighbor_pos)
                        max_speed = self.cost_model.drone.max_speed
                        acceleration = max_speed / 5.0
                        time_to_travel = distance / max_speed if max_speed > 0 else 0
                        
                        if time_to_travel > 0:
                            speed_gain = min(acceleration * time_to_travel, max_speed - current_speed_at_node)
                            estimated_speed = min(max_speed, current_speed_at_node + speed_gain)
                        else:
                            estimated_speed = current_speed_at_node
                    else:
                        estimated_speed = current_speed_at_node
                
                # If this path to neighbor is better
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = parent if (parent is not None and self._line_of_sight(parent, neighbor)) else current
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
        """Convert path node IDs to Waypoint objects with smooth curves.
        
        Theta* produces waypoints that can be connected with curves for smoother trajectories.
        This method adds intermediate waypoints to create curved paths.
        
        Args:
            path_nodes: List of node IDs
        
        Returns:
            List of Waypoint objects with smooth curves
        """
        if not path_nodes or len(path_nodes) < 2:
            return [self.graph.get_node_waypoint(node_id) for node_id in path_nodes]
        
        waypoints = []
        
        for i in range(len(path_nodes) - 1):
            node1 = path_nodes[i]
            node2 = path_nodes[i + 1]
            
            wp1 = self.graph.get_node_waypoint(node1)
            wp2 = self.graph.get_node_waypoint(node2)
            
            # Add first waypoint
            if i == 0:
                waypoints.append(wp1)
            
            # Calculate distance between waypoints
            distance = self._euclidean_distance_3d(
                self.graph.get_node_position(node1),
                self.graph.get_node_position(node2)
            )
            
            # If distance is large, add intermediate waypoints for smooth curves
            # Optimized: only add points for longer segments to avoid too many waypoints
            if distance > 300:  # More than 300m
                # Add intermediate waypoints using smooth interpolation
                # Optimal: 1 point per 200-300m for good curve approximation without too many points
                segment_length = 250.0  # meters per segment
                num_intermediate = max(1, min(5, int(distance / segment_length)))  # 1-5 points max
                
                for j in range(1, num_intermediate + 1):
                    t = j / (num_intermediate + 1)
                    
                    # Use smooth interpolation (ease-in-out curve for natural motion)
                    smooth_t = t * t * (3 - 2 * t)  # Smoothstep function
                    
                    # Interpolate position
                    lat = wp1.latitude + (wp2.latitude - wp1.latitude) * smooth_t
                    lon = wp1.longitude + (wp2.longitude - wp1.longitude) * smooth_t
                    alt = wp1.altitude + (wp2.altitude - wp1.altitude) * smooth_t
                    
                    # Ensure intermediate waypoints respect minimum altitude
                    # Skip this check if wp1 or wp2 are ground points (depot/finish)
                    if (wp1.waypoint_type not in ["depot", "finish"] and 
                        wp2.waypoint_type not in ["depot", "finish"]):
                        # Both are flight waypoints, ensure intermediate respects min altitude
                        # Get drone min altitude from graph if available
                        if hasattr(self.graph, 'cost_model') and self.graph.cost_model:
                            min_alt = self.graph.cost_model.drone.min_altitude
                            alt = max(alt, min_alt)
                    
                    # Create intermediate waypoint
                    intermediate_wp = Waypoint(
                        latitude=lat,
                        longitude=lon,
                        altitude=alt,
                        waypoint_type="intermediate"
                    )
                    waypoints.append(intermediate_wp)
            
            # Add second waypoint
            waypoints.append(wp2)
        
        return waypoints
    
    def _line_of_sight(self, node1: str, node2: str) -> bool:
        """Check if there's a direct line-of-sight between two nodes.
        
        In Theta*, this means checking if the direct path doesn't intersect obstacles.
        Checks distance, intermediate nodes, and no-fly zones.
        
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
        
        # Check distance limit
        if distance >= 5000:  # 5km max line-of-sight
            return False
        
        # Check if edge is valid (includes no-fly zone checks)
        if self.cost_model:
            lat1, lon1, alt1 = pos1[1], pos1[0], pos1[2]  # pos is (lon, lat, alt)
            lat2, lon2, alt2 = pos2[1], pos2[0], pos2[2]
            
            # Check if this edge would be valid (includes no-fly zone intersection check)
            is_valid, _ = self.cost_model.is_valid_edge(
                lat1, lon1, alt1,
                lat2, lon2, alt2,
                is_start_ground=False,  # We don't know if these are ground points, but this is for waypoint graph
                is_end_ground=False
            )
            if not is_valid:
                return False
        
        return True
    
    def _direct_cost(self, node1: str, node2: str, current_speed: float = 0.0) -> float:
        """Calculate direct cost between two nodes using CostModel (includes inertia and wind).
        
        If CostModel is available, uses it for accurate cost calculation.
        Otherwise falls back to Euclidean distance.
        
        Args:
            node1: First node ID
            node2: Second node ID
            current_speed: Current speed at node1 (m/s) for inertia calculation
        
        Returns:
            Direct cost (includes inertia and wind effects)
        """
        pos1 = self.graph.get_node_position(node1)
        pos2 = self.graph.get_node_position(node2)
        
        # Use CostModel if available (includes inertia and wind)
        if self.cost_model:
            # pos format: (lon, lat, alt)
            return self.cost_model.calculate_cost(
                pos1[1], pos1[0], pos1[2],  # lat1, lon1, alt1
                pos2[1], pos2[0], pos2[2],  # lat2, lon2, alt2
                current_speed=current_speed
            )
        else:
            # Fallback to simple Euclidean distance
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

