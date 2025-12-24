"""Mission optimizer using genetic algorithms."""
from typing import Dict, Optional
from app.domain.mission import Mission
from app.domain.route import Route
from app.optimization.genetic_optimizer import GeneticOptimizer


class MissionOptimizer:
    """Optimizes mission routes using genetic algorithms."""
    
    def __init__(self, mission: Mission):
        """Initialize mission optimizer.
        
        Args:
            mission: Mission to optimize
        """
        self.mission = mission
    
    def optimize_routes(self, use_genetic: bool = True) -> Dict[str, Route]:
        """Optimize all routes in mission.
        
        Args:
            use_genetic: Whether to use genetic algorithm optimization
        
        Returns:
            Dictionary of optimized routes
        """
        optimized_routes = {}
        
        for drone_name, route in self.mission.routes.items():
            drone = next((d for d in self.mission.drones if d.name == drone_name), None)
            if not drone:
                continue
            
            if use_genetic and len(route.waypoints) > 2:
                # Use genetic algorithm
                optimizer = GeneticOptimizer(route, drone)
                optimized_route = optimizer.optimize()
            else:
                optimized_route = route
            
            optimized_routes[drone_name] = optimized_route
        
        return optimized_routes

