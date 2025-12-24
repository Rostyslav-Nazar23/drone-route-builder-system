"""Navigation graph for pathfinding."""
from typing import Dict, List, Tuple, Optional, Set
import networkx as nx
from shapely.geometry import Point, LineString
from app.domain.waypoint import Waypoint
from app.domain.constraints import MissionConstraints


class NavigationGraph:
    """Navigation graph for pathfinding algorithms."""
    
    def __init__(self, graph: nx.Graph = None):
        """Initialize navigation graph.
        
        Args:
            graph: NetworkX graph (if None, creates empty graph)
        """
        self.graph = graph if graph is not None else nx.Graph()
    
    def add_node(self, node_id: str, latitude: float, longitude: float, altitude: float):
        """Add a node to the graph.
        
        Args:
            node_id: Unique node identifier
            latitude: Latitude coordinate
            longitude: Longitude coordinate
            altitude: Altitude in meters
        """
        self.graph.add_node(
            node_id,
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
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
            altitude=node['altitude']
        )
    
    def get_neighbors(self, node_id: str) -> List[str]:
        """Get neighbor node IDs."""
        return list(self.graph.neighbors(node_id))
    
    def get_edge_weight(self, node1: str, node2: str) -> float:
        """Get edge weight between two nodes."""
        if not self.graph.has_edge(node1, node2):
            return float('inf')
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

