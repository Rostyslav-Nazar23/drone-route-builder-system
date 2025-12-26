"""D* pathfinding algorithm implementation - dynamic replanning."""
from typing import List, Optional, Dict, Tuple, Set
import heapq
import math
from app.environment.navigation_graph import NavigationGraph
from app.domain.waypoint import Waypoint


class DStar:
    """D* pathfinding algorithm - supports dynamic replanning."""
    
    # Node states
    NEW = 0
    OPEN = 1
    CLOSED = 2
    
    def __init__(self, graph: NavigationGraph):
        """Initialize D* with navigation graph.
        
        Args:
            graph: NavigationGraph instance
        """
        self.graph = graph
        self.states: Dict[str, int] = {}  # Node state
        self.g_score: Dict[str, float] = {}  # Cost from start
        self.rhs: Dict[str, float] = {}  # Right-hand side (one-step lookahead)
        self.open_list: List[Tuple[float, float, str]] = []  # Priority queue: (key, node)
        self.km: float = 0.0  # Key modifier for dynamic updates
    
    def find_path(self, start_node: str, goal_node: str) -> Optional[List[str]]:
        """Find initial path from start to goal using D*.
        
        Args:
            start_node: Start node ID
            goal_node: Goal node ID
        
        Returns:
            List of node IDs representing the path, or None if no path found
        """
        if not self.graph.has_node(start_node) or not self.graph.has_node(goal_node):
            return None
        
        self.start_node = start_node
        self.goal_node = goal_node
        
        # Initialize
        for node_id in self.graph.nodes():
            self.g_score[node_id] = float('inf')
            self.rhs[node_id] = float('inf')
            self.states[node_id] = self.NEW
        
        # Initialize goal
        self.rhs[goal_node] = 0.0
        self._insert(goal_node, self._calculate_key(goal_node))
        
        # Compute initial path
        self._compute_shortest_path()
        
        # Reconstruct path from start to goal
        return self._reconstruct_path(start_node, goal_node)
    
    def replan(self, changed_edges: List[Tuple[str, str, float]]) -> Optional[List[str]]:
        """Replan path after edge costs have changed.
        
        Args:
            changed_edges: List of (node1, node2, new_cost) tuples
        
        Returns:
            Updated path, or None if no path exists
        """
        # Update edge costs
        for node1, node2, new_cost in changed_edges:
            # Update graph edge
            if self.graph.has_edge(node1, node2):
                self.graph[node1][node2]['weight'] = new_cost
            
            # Update affected nodes
            self._update_vertex(node1)
            self._update_vertex(node2)
        
        # Recompute path
        self._compute_shortest_path()
        
        # Reconstruct path
        return self._reconstruct_path(self.start_node, self.goal_node)
    
    def _compute_shortest_path(self):
        """Compute shortest path using D* algorithm."""
        while self.open_list and (self._top_key() < self._calculate_key(self.start_node) or 
                                  self.rhs[self.start_node] != self.g_score[self.start_node]):
            k_old = self._top_key()
            u = self._pop()
            
            k_new = self._calculate_key(u)
            
            if k_old < k_new:
                self._insert(u, k_new)
            elif self.g_score[u] > self.rhs[u]:
                self.g_score[u] = self.rhs[u]
                self.states[u] = self.CLOSED
                
                # Update neighbors
                for neighbor in self.graph.get_neighbors(u):
                    # Additional safety check: verify edge is still valid (includes no-fly zone check)
                    if hasattr(self.graph, 'cost_model') and self.graph.cost_model:
                        pos1 = self.graph.get_node_position(u)
                        pos2 = self.graph.get_node_position(neighbor)
                        is_valid, _ = self.graph.cost_model.is_valid_edge(
                            pos1[1], pos1[0], pos1[2],  # lat, lon, alt
                            pos2[1], pos2[0], pos2[2],
                            is_start_ground=False,
                            is_end_ground=False
                        )
                        if not is_valid:
                            continue  # Skip this edge if it's invalid (e.g., intersects no-fly zone)
                    self._update_vertex(neighbor)
            else:
                self.g_score[u] = float('inf')
                self._update_vertex(u)
                for neighbor in self.graph.get_neighbors(u):
                    # Additional safety check: verify edge is still valid (includes no-fly zone check)
                    if hasattr(self.graph, 'cost_model') and self.graph.cost_model:
                        pos1 = self.graph.get_node_position(u)
                        pos2 = self.graph.get_node_position(neighbor)
                        is_valid, _ = self.graph.cost_model.is_valid_edge(
                            pos1[1], pos1[0], pos1[2],  # lat, lon, alt
                            pos2[1], pos2[0], pos2[2],
                            is_start_ground=False,
                            is_end_ground=False
                        )
                        if not is_valid:
                            continue  # Skip this edge if it's invalid (e.g., intersects no-fly zone)
                    self._update_vertex(neighbor)
    
    def _update_vertex(self, node: str):
        """Update vertex in D* algorithm."""
        if node != self.goal_node:
            # Calculate minimum rhs from neighbors
            min_rhs = float('inf')
            for neighbor in self.graph.get_neighbors(node):
                # Additional safety check: verify edge is still valid (includes no-fly zone check)
                if hasattr(self.graph, 'cost_model') and self.graph.cost_model:
                    pos1 = self.graph.get_node_position(neighbor)
                    pos2 = self.graph.get_node_position(node)
                    is_valid, _ = self.graph.cost_model.is_valid_edge(
                        pos1[1], pos1[0], pos1[2],  # lat, lon, alt
                        pos2[1], pos2[0], pos2[2],
                        is_start_ground=False,
                        is_end_ground=False
                    )
                    if not is_valid:
                        continue  # Skip this edge if it's invalid (e.g., intersects no-fly zone)
                
                # Estimate current speed at neighbor for inertia calculation
                # In D*, we estimate speed based on distance from start
                estimated_speed = 0.0
                if hasattr(self.graph, 'cost_model') and self.graph.cost_model:
                    # Estimate speed based on g_score (cost from start)
                    # Higher g_score means more distance traveled, likely higher speed
                    max_speed = self.graph.cost_model.drone.max_speed
                    if self.g_score.get(neighbor, float('inf')) < float('inf'):
                        # Rough estimate: if we've traveled far, we're likely at higher speed
                        # This is a simplified heuristic
                        estimated_speed = min(max_speed, max_speed * 0.7)  # Assume 70% of max speed
                
                cost = self.graph.get_edge_weight(neighbor, node, current_speed=estimated_speed)
                candidate_rhs = self.g_score[neighbor] + cost
                if candidate_rhs < min_rhs:
                    min_rhs = candidate_rhs
            self.rhs[node] = min_rhs
        
        # Update state
        if self.states.get(node, self.NEW) == self.OPEN:
            self._remove(node)
        
        if self.g_score[node] != self.rhs[node]:
            self._insert(node, self._calculate_key(node))
            self.states[node] = self.OPEN
    
    def _calculate_key(self, node: str) -> Tuple[float, float]:
        """Calculate key for priority queue.
        
        Args:
            node: Node ID
        
        Returns:
            (key1, key2) tuple
        """
        g = self.g_score.get(node, float('inf'))
        rhs = self.rhs.get(node, float('inf'))
        
        key1 = min(g, rhs) + self._heuristic(self.start_node, node) + self.km
        key2 = min(g, rhs)
        
        return (key1, key2)
    
    def _insert(self, node: str, key: Tuple[float, float]):
        """Insert node into open list."""
        self._remove(node)  # Remove if already present
        heapq.heappush(self.open_list, (key[0], key[1], node))
        self.states[node] = self.OPEN
    
    def _remove(self, node: str):
        """Remove node from open list."""
        self.open_list = [(k1, k2, n) for k1, k2, n in self.open_list if n != node]
        heapq.heapify(self.open_list)
    
    def _pop(self) -> str:
        """Pop node with minimum key from open list."""
        while self.open_list:
            k1, k2, node = heapq.heappop(self.open_list)
            if self.states.get(node) == self.OPEN:
                return node
        return None
    
    def _top_key(self) -> Tuple[float, float]:
        """Get top key from open list."""
        if not self.open_list:
            return (float('inf'), float('inf'))
        
        # Rebuild heap to get actual top
        temp = [(k1, k2, n) for k1, k2, n in self.open_list if self.states.get(n) == self.OPEN]
        if not temp:
            return (float('inf'), float('inf'))
        
        temp.sort()
        return (temp[0][0], temp[0][1])
    
    def _reconstruct_path(self, start: str, goal: str) -> Optional[List[str]]:
        """Reconstruct path from start to goal.
        
        Args:
            start: Start node ID
            goal: Goal node ID
        
        Returns:
            List of node IDs, or None if no path
        """
        if self.g_score.get(start, float('inf')) == float('inf'):
            return None
        
        path = [start]
        current = start
        
        while current != goal:
            # Find best neighbor
            best_neighbor = None
            best_cost = float('inf')
            
            for neighbor in self.graph.get_neighbors(current):
                if neighbor in path:  # Avoid cycles
                    continue
                
                # Additional safety check: verify edge is still valid (includes no-fly zone check)
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
                
                # Estimate current speed at current node for inertia calculation
                estimated_speed = 0.0
                if hasattr(self.graph, 'cost_model') and self.graph.cost_model:
                    # Estimate speed based on g_score (cost from start)
                    max_speed = self.graph.cost_model.drone.max_speed
                    if self.g_score.get(current, float('inf')) < float('inf'):
                        # Rough estimate: assume we're at 70% of max speed after traveling
                        estimated_speed = min(max_speed, max_speed * 0.7)
                
                cost = self.graph.get_edge_weight(current, neighbor, current_speed=estimated_speed)
                total_cost = self.g_score.get(neighbor, float('inf')) + cost
                
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_neighbor = neighbor
            
            if best_neighbor is None:
                return None  # No path found
            
            path.append(best_neighbor)
            current = best_neighbor
        
        return path
    
    def _heuristic(self, node1: str, node2: str) -> float:
        """Heuristic function (Euclidean distance).
        
        Args:
            node1: First node ID
            node2: Second node ID
        
        Returns:
            Estimated distance
        """
        pos1 = self.graph.get_node_position(node1)
        pos2 = self.graph.get_node_position(node2)
        
        # Convert to meters
        lat1, lon1, alt1 = pos1[1], pos1[0], pos1[2]
        lat2, lon2, alt2 = pos2[1], pos2[0], pos2[2]
        
        lat_m = (lat2 - lat1) * 111320.0
        lon_m = (lon2 - lon1) * 111320.0 * abs(math.cos(math.radians(lat1)))
        alt_m = alt2 - alt1
        
        return math.sqrt(lat_m ** 2 + lon_m ** 2 + alt_m ** 2)
    
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
        """Convert path node IDs to Waypoint objects."""
        return [self.graph.get_node_waypoint(node_id) for node_id in path_nodes]

