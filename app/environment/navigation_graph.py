"""Navigation graph for pathfinding."""
from typing import Dict, List, Tuple, Optional, Set
import networkx as nx
from shapely.geometry import Point, LineString
from app.domain.waypoint import Waypoint
from app.domain.constraints import MissionConstraints


class NavigationGraph:
    """Navigation graph for pathfinding algorithms."""
    
    def __init__(self, graph: nx.Graph = None, cost_model=None):
        """Initialize navigation graph.
        
        Args:
            graph: NetworkX graph (if None, creates empty graph)
            cost_model: CostModel instance for dynamic cost calculation (optional)
        """
        self.graph = graph if graph is not None else nx.Graph()
        self.cost_model = cost_model  # Store CostModel for algorithms that need it
    
    def add_node(self, node_id: str, latitude: float, longitude: float, altitude: float,
                 waypoint_type: str = "target"):
        """Add a node to the graph.
        
        Args:
            node_id: Unique node identifier
            latitude: Latitude coordinate
            longitude: Longitude coordinate
            altitude: Altitude in meters
            waypoint_type: Type of waypoint ("depot", "finish", "target", "intermediate")
        """
        self.graph.add_node(
            node_id,
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            waypoint_type=waypoint_type,
            pos=(longitude, latitude, altitude)  # For 3D visualization
        )
    
    def add_edge(self, node1: str, node2: str, weight: float, **attributes):
        """Add an edge between two nodes.
        
        Args:
            node1: First node ID
            node2: Second node ID
            weight: Edge weight (cost)
            **attributes: Additional edge attributes
        """
        self.graph.add_edge(node1, node2, weight=weight, **attributes)
    
    def get_node_position(self, node_id: str) -> Tuple[float, float, float]:
        """Get node position (lon, lat, alt)."""
        node = self.graph.nodes[node_id]
        return node.get('pos', (0.0, 0.0, 0.0))
    
    def get_node_waypoint(self, node_id: str) -> Waypoint:
        """Get waypoint from node."""
        node = self.graph.nodes[node_id]
        return Waypoint(
            latitude=node['latitude'],
            longitude=node['longitude'],
            altitude=node['altitude'],
            waypoint_type=node.get('waypoint_type', 'target')
        )
    
    def get_neighbors(self, node_id: str) -> List[str]:
        """Get neighbor node IDs."""
        return list(self.graph.neighbors(node_id))
    
    def get_edge_weight(self, node1: str, node2: str, current_speed: float = 0.0) -> float:
        """Get edge weight between two nodes.
        
        If cost_model is available, calculates dynamic cost based on current_speed (for inertia).
        Otherwise, returns cached weight.
        
        Args:
            node1: First node ID
            node2: Second node ID
            current_speed: Current speed at node1 (m/s) for inertia calculation
            
        Returns:
            Edge weight (cost)
        """
        if not self.graph.has_edge(node1, node2):
            return float('inf')
        
        # If cost_model is available, calculate dynamic cost with current_speed
        if self.cost_model and current_speed > 0:
            pos1 = self.get_node_position(node1)
            pos2 = self.get_node_position(node2)
            # Calculate dynamic cost with current speed (accounts for inertia)
            dynamic_cost = self.cost_model.calculate_cost(
                pos1[1], pos1[0], pos1[2],  # lat, lon, alt
                pos2[1], pos2[0], pos2[2],
                current_speed=current_speed
            )
            return dynamic_cost
        
        # Fallback to cached weight (calculated with current_speed=0.0)
        return self.graph[node1][node2].get('weight', 1.0)
    
    def has_node(self, node_id: str) -> bool:
        """Check if node exists."""
        return self.graph.has_node(node_id)
    
    def has_edge(self, node1: str, node2: str) -> bool:
        """Check if edge exists."""
        return self.graph.has_edge(node1, node2)
    
    def nodes(self):
        """Get all node IDs."""
        return self.graph.nodes()
    
    def edges(self):
        """Get all edges."""
        return self.graph.edges()
    
    def number_of_nodes(self) -> int:
        """Get number of nodes."""
        return self.graph.number_of_nodes()
    
    def number_of_edges(self) -> int:
        """Get number of edges."""
        return self.graph.number_of_edges()

