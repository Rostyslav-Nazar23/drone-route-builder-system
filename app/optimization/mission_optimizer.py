"""Mission optimizer using various optimization algorithms."""
from typing import Dict, Optional
from app.domain.mission import Mission
from app.domain.route import Route
from app.optimization.genetic_optimizer import GeneticOptimizer
from app.optimization.aco_optimizer import ACOOptimizer
from app.optimization.pso_optimizer import PSOOptimizer


class MissionOptimizer:
    """Optimizes mission routes using various optimization algorithms."""
    
    def __init__(self, mission: Mission):
        """Initialize mission optimizer.
        
        Args:
            mission: Mission to optimize
        """
        self.mission = mission
    
    def optimize_routes(self, optimization_algorithm: str = "genetic") -> Dict[str, Route]:
        """Optimize all routes in mission.
        
        Args:
            optimization_algorithm: "genetic", "aco", or "pso"
        
        Returns:
            Dictionary of optimized routes (preserves all drone routes)
        """
        optimized_routes = {}
        
        # Ensure we process all routes in the mission
        if not self.mission.routes:
            return optimized_routes
        
        for drone_name, route in self.mission.routes.items():
            drone = next((d for d in self.mission.drones if d.name == drone_name), None)
            if not drone:
                # Keep route even if drone not found (shouldn't happen, but be safe)
                optimized_routes[drone_name] = route
                continue
            
            if len(route.waypoints) > 2:
                try:
                    constraints = self.mission.constraints if self.mission.constraints else None
                    if optimization_algorithm == "aco":
                        optimizer = ACOOptimizer(route, drone, constraints=constraints)
                        optimized_route = optimizer.optimize()
                    elif optimization_algorithm == "pso":
                        optimizer = PSOOptimizer(route, drone, constraints=constraints)
                        optimized_route = optimizer.optimize()
                    else:  # default to genetic
                        optimizer = GeneticOptimizer(route, drone, constraints=constraints)
                        optimized_route = optimizer.optimize()
                    
                    optimized_routes[drone_name] = optimized_route
                except Exception as e:
                    # If optimization fails, use original route
                    optimized_routes[drone_name] = route
            else:
                # Too few waypoints to optimize
                optimized_routes[drone_name] = route
        
        return optimized_routes

