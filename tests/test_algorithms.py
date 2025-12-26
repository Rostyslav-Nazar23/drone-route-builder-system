"""Tests for pathfinding algorithms."""
import unittest
from app.environment.navigation_graph import NavigationGraph
from app.planning.a_star import AStar
from app.planning.theta_star import ThetaStar
from app.planning.d_star import DStar
from app.domain.waypoint import Waypoint


class TestAlgorithms(unittest.TestCase):
    """Test pathfinding algorithms."""
    
    def setUp(self):
        """Set up test graph."""
        self.graph = NavigationGraph()
        
        # Create simple grid graph
        # Nodes: 0-1-2
        #        | | |
        #        3-4-5
        #        | | |
        #        6-7-8
        
        positions = [
            (0, 0, 0), (1, 0, 0), (2, 0, 0),  # 0, 1, 2
            (0, 1, 0), (1, 1, 0), (2, 1, 0),  # 3, 4, 5
            (0, 2, 0), (1, 2, 0), (2, 2, 0),  # 6, 7, 8
        ]
        
        for i, (lon, lat, alt) in enumerate(positions):
            self.graph.add_node(f"n{i}", lat, lon, alt)
        
        # Add edges (horizontal and vertical connections)
        edges = [
            (0, 1), (1, 2),  # Top row
            (0, 3), (1, 4), (2, 5),  # Vertical
            (3, 4), (4, 5),  # Middle row
            (3, 6), (4, 7), (5, 8),  # Vertical
            (6, 7), (7, 8),  # Bottom row
        ]
        
        for n1, n2 in edges:
            pos1 = self.graph.get_node_position(f"n{n1}")
            pos2 = self.graph.get_node_position(f"n{n2}")
            # Simple distance as weight
            weight = ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2)**0.5
            self.graph.add_edge(f"n{n1}", f"n{n2}", weight)
    
    def test_astar_finds_path(self):
        """Test A* finds path."""
        a_star = AStar(self.graph)
        path = a_star.find_path("n0", "n8")
        
        self.assertIsNotNone(path, "A* should find a path")
        self.assertEqual(path[0], "n0", "Path should start at start node")
        self.assertEqual(path[-1], "n8", "Path should end at goal node")
        self.assertGreater(len(path), 1, "Path should have multiple nodes")
    
    def test_astar_path_validity(self):
        """Test A* path is valid (all nodes connected)."""
        a_star = AStar(self.graph)
        path = a_star.find_path("n0", "n8")
        
        if path:
            for i in range(len(path) - 1):
                self.assertTrue(
                    self.graph.has_edge(path[i], path[i+1]),
                    f"Path should have edge between {path[i]} and {path[i+1]}"
                )
    
    def test_thetastar_finds_path(self):
        """Test Theta* finds path."""
        theta_star = ThetaStar(self.graph)
        path = theta_star.find_path("n0", "n8")
        
        self.assertIsNotNone(path, "Theta* should find a path")
        self.assertEqual(path[0], "n0", "Path should start at start node")
        self.assertEqual(path[-1], "n8", "Path should end at goal node")
    
    def test_dstar_finds_path(self):
        """Test D* finds path."""
        d_star = DStar(self.graph)
        path = d_star.find_path("n0", "n8")
        
        self.assertIsNotNone(path, "D* should find a path")
        self.assertEqual(path[0], "n0", "Path should start at start node")
        self.assertEqual(path[-1], "n8", "Path should end at goal node")
    
    def test_algorithms_find_same_goal(self):
        """Test all algorithms find path to same goal."""
        start = "n0"
        goal = "n8"
        
        a_star = AStar(self.graph)
        theta_star = ThetaStar(self.graph)
        d_star = DStar(self.graph)
        
        path_astar = a_star.find_path(start, goal)
        path_thetastar = theta_star.find_path(start, goal)
        path_dstar = d_star.find_path(start, goal)
        
        self.assertIsNotNone(path_astar, "A* should find path")
        self.assertIsNotNone(path_thetastar, "Theta* should find path")
        self.assertIsNotNone(path_dstar, "D* should find path")
        
        # All should reach the goal
        self.assertEqual(path_astar[-1], goal)
        self.assertEqual(path_thetastar[-1], goal)
        self.assertEqual(path_dstar[-1], goal)
    
    def test_path_to_waypoints(self):
        """Test finding path through multiple waypoints."""
        a_star = AStar(self.graph)
        waypoints = ["n1", "n4", "n7"]
        path = a_star.find_path_to_waypoints("n0", waypoints)
        
        self.assertIsNotNone(path, "Should find path through waypoints")
        self.assertEqual(path[0], "n0", "Should start at start node")
        # Check that waypoints are visited
        waypoint_indices = [path.index(wp) for wp in waypoints if wp in path]
        self.assertEqual(len(waypoint_indices), len(waypoints), "Should visit all waypoints")


if __name__ == '__main__':
    unittest.main()

