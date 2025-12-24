"""Mission orchestrator for coordinating the full planning pipeline."""
from typing import Optional, Dict
from app.domain.mission import Mission
from app.domain.route import Route
from app.planning.route_planner import RoutePlanner
from app.validation.constraint_checker import ConstraintChecker
from app.weather.weather_provider import WeatherConditions


class MissionOrchestrator:
    """Orchestrates the complete mission planning pipeline."""
    
    def __init__(self, mission: Mission, weather_data: Optional[Dict[tuple[float, float], WeatherConditions]] = None):
        """Initialize orchestrator with mission.
        
        Args:
            mission: Mission object
            weather_data: Dictionary mapping (lat, lon) to WeatherConditions (optional)
        """
        self.mission = mission
        self.weather_data = weather_data
        self.planner = RoutePlanner(mission, weather_data)
        self.checker = ConstraintChecker()
    
    def plan_mission(self, use_grid: bool = True, use_weather: bool = True,
                    algorithm: str = "astar", use_vrp: bool = True,
                    use_genetic: bool = False) -> Dict[str, Route]:
        """Plan complete mission.
        
        Args:
            use_grid: Whether to use grid-based graph
        
        Returns:
            Dictionary mapping drone name to Route
        """
        routes = {}
        
        if len(self.mission.drones) == 1:
            # Single drone mission
            route = self.planner.plan_single_drone_route(
                self.mission.drones[0], 
                use_grid=use_grid,
                algorithm=algorithm
            )
            if route:
                routes[self.mission.drones[0].name] = route
        else:
            # Multi-drone mission
            routes = self.planner.plan_multi_drone_routes(use_vrp=use_vrp)
        
        # Optimize routes with genetic algorithm if requested
        if use_genetic:
            from app.optimization.mission_optimizer import MissionOptimizer
            optimizer = MissionOptimizer(self.mission)
            # Temporarily store routes for optimization
            for drone_name, route in routes.items():
                self.mission.add_route(drone_name, route)
            optimized_routes = optimizer.optimize_routes(use_genetic=True)
            routes = optimized_routes
        
        # Validate routes
        for drone_name, route in routes.items():
            drone = next((d for d in self.mission.drones if d.name == drone_name), None)
            if drone:
                validation_result = self.checker.validate_route(route, drone, self.mission.constraints)
                route.validation_result = validation_result
        
        # Store routes in mission
        for drone_name, route in routes.items():
            self.mission.add_route(drone_name, route)
        
        return routes
    
    def replan_route(self, drone_name: str, use_grid: bool = True, use_weather: bool = True) -> Optional[Route]:
        """Replan route for a specific drone.
        
        Args:
            drone_name: Name of drone to replan
            use_grid: Whether to use grid-based graph
        
        Returns:
            New Route, or None if planning fails
        """
        drone = next((d for d in self.mission.drones if d.name == drone_name), None)
        if not drone:
            return None
        
        route = self.planner.plan_single_drone_route(drone, use_grid)
        if route:
            validation_result = self.checker.validate_route(route, drone, self.mission.constraints)
            route.validation_result = validation_result
            self.mission.add_route(drone_name, route)
        
        return route

