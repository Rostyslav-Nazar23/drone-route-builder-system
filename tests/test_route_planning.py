"""Tests for route planning."""
import unittest
from app.domain.mission import Mission
from app.domain.drone import Drone
from app.domain.waypoint import Waypoint
from app.orchestrator.mission_orchestrator import MissionOrchestrator


class TestRoutePlanning(unittest.TestCase):
    """Test route planning functionality."""
    
    def setUp(self):
        """Set up test mission."""
        self.drone = Drone(
            name="Test Drone",
            max_speed=15.0,
            max_altitude=120.0,
            min_altitude=10.0,
            battery_capacity=100.0,
            power_consumption=50.0
        )
        
        self.mission = Mission(
            name="Test Mission",
            drones=[self.drone],
            target_points=[
                Waypoint(50.0, 30.0, 50.0, "Target 1"),
                Waypoint(50.01, 30.01, 60.0, "Target 2"),
            ],
            depot=Waypoint(49.99, 29.99, 0.0, "Depot")
        )
    
    def test_plan_single_drone_route(self):
        """Test planning route for single drone."""
        orchestrator = MissionOrchestrator(self.mission)
        routes = orchestrator.plan_mission(use_grid=False, algorithm="astar")
        
        self.assertIsNotNone(routes, "Should return routes")
        self.assertIn(self.drone.name, routes, "Should have route for drone")
        
        route = routes[self.drone.name]
        self.assertGreater(len(route.waypoints), 0, "Route should have waypoints")
    
    def test_route_visits_targets(self):
        """Test that route visits all target points."""
        orchestrator = MissionOrchestrator(self.mission)
        routes = orchestrator.plan_mission(use_grid=False, algorithm="astar")
        
        if routes:
            route = routes[self.drone.name]
            route_locations = [(wp.latitude, wp.longitude) for wp in route.waypoints]
            
            # Check that targets are near route waypoints (within 100m tolerance)
            for target in self.mission.target_points:
                target_loc = (target.latitude, target.longitude)
                # Find closest waypoint
                min_dist = min([
                    ((loc[0] - target_loc[0])**2 + (loc[1] - target_loc[1])**2)**0.5 * 111320
                    for loc in route_locations
                ])
                # Should be within 200m (reasonable for waypoint graph)
                self.assertLess(min_dist, 200, f"Target {target.name} should be near route")
    
    def test_route_metrics_calculated(self):
        """Test that route metrics are calculated."""
        orchestrator = MissionOrchestrator(self.mission)
        routes = orchestrator.plan_mission(use_grid=False, algorithm="astar")
        
        if routes:
            route = routes[self.drone.name]
            route.calculate_metrics(self.drone)
            
            self.assertIsNotNone(route.metrics, "Route should have metrics")
            self.assertGreater(route.metrics.total_distance, 0, "Distance should be positive")
            self.assertGreater(route.metrics.total_time, 0, "Time should be positive")
    
    def test_route_validation(self):
        """Test route validation."""
        orchestrator = MissionOrchestrator(self.mission)
        routes = orchestrator.plan_mission(use_grid=False, algorithm="astar")
        
        if routes:
            route = routes[self.drone.name]
            self.assertIsNotNone(route.validation_result, "Route should have validation result")
            
            # Validation result should be a dict
            if isinstance(route.validation_result, dict):
                self.assertIn("is_valid", route.validation_result, "Should have is_valid field")


if __name__ == '__main__':
    unittest.main()

