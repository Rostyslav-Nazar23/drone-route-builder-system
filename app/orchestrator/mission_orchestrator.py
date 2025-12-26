"""Mission orchestrator for coordinating the full planning pipeline."""
from typing import Optional, Dict, List, Tuple
from datetime import datetime
from app.domain.mission import Mission
from app.domain.route import Route
from app.planning.route_planner import RoutePlanner
from app.validation.constraint_checker import ConstraintChecker
from app.weather.weather_provider import WeatherConditions
from app.environment.navigation_graph import NavigationGraph


class MissionOrchestrator:
    """Orchestrates the complete mission planning pipeline."""
    
    def __init__(self, mission: Mission, 
                 weather_data: Optional[Dict[tuple[float, float], WeatherConditions]] = None,
                 use_weather: bool = True,
                 weather_timestamp: Optional[datetime] = None):
        """Initialize orchestrator with mission.
        
        Args:
            mission: Mission object
            weather_data: Dictionary mapping (lat, lon) to WeatherConditions (optional, used as initial cache)
            use_weather: Whether to fetch and use weather data during route planning
            weather_timestamp: Timestamp for weather data (default: current time)
        """
        from datetime import datetime
        self.mission = mission
        self.weather_data = weather_data
        self.use_weather = use_weather
        self.weather_timestamp = weather_timestamp or datetime.now()
        self.planner = RoutePlanner(mission, weather_data, use_weather=use_weather, weather_timestamp=self.weather_timestamp)
        self.checker = ConstraintChecker()
        self.current_graph: Optional[NavigationGraph] = None  # Store graph for visualization
    
    def plan_mission(self, use_weather: Optional[bool] = None,
                    algorithm: str = "astar",
                    optimization_algorithm: Optional[str] = None,
                    optimization_metric: str = "distance",
                    landing_mode: Optional[str] = None,
                    finish_point_type: Optional[str] = None,
                    finish_point: Optional = None) -> tuple[Dict[str, Route], Optional[str]]:
        """Plan complete mission.
        
        New logic:
        1. For multi-drone: Solve VRP first to assign targets to drones
        2. Then build trajectories between points using pathfinding algorithms (A*, D*, Theta*)
        
        Args:
            use_weather: Whether to use weather data
            algorithm: Pathfinding algorithm ("astar", "thetastar", "dstar")
            optimization_algorithm: Route optimization algorithm ("genetic", "aco", "pso", or None)
            optimization_metric: Optimization metric ("distance", "energy", "time")
            landing_mode: Landing mode ("vertical" or "gradual") - overrides mission setting
            finish_point_type: Finish point type ("depot", "last_target", "custom") - overrides mission setting
            finish_point: Custom finish point - overrides mission setting
        
        Returns:
            Tuple of (routes dictionary, error_message)
        """
        routes = {}
        error_message = None
        
        # Update mission route options if provided
        if landing_mode is not None:
            self.mission.landing_mode = landing_mode
        if finish_point_type is not None:
            self.mission.finish_point_type = finish_point_type
        if finish_point is not None:
            self.mission.finish_point = finish_point
        
        # Debug: Check number of drones
        if len(self.mission.drones) == 0:
            error_message = "No drones configured in mission"
            return routes, error_message
        
        # Check target points against no-fly zones before planning
        if self.mission.constraints and self.mission.constraints.no_fly_zones:
            violations = self._check_target_points_against_zones()
            if violations:
                violation_messages = [v["message"] for v in violations]
                error_message = f"Cannot plan route: Target points are in no-fly zones:\n" + "\n".join(f"- {msg}" for msg in violation_messages)
                return routes, error_message
        
        if len(self.mission.drones) == 1:
            # Single drone mission: direct pathfinding
            route = self.planner.plan_single_drone_route(
                self.mission.drones[0], 
                algorithm=algorithm,
                optimization_metric=optimization_metric
            )
            if route:
                routes[self.mission.drones[0].name] = route
            else:
                error_message = "No route found. Possible reasons:\n" \
                              "- Target points are unreachable\n" \
                              "- No-fly zones block all possible paths\n" \
                              "- Constraints are too restrictive\n" \
                              "- Try adjusting target points or no-fly zones"
        else:
            # Multi-drone mission: VRP first, then pathfinding
            # Step 1: Solve VRP to assign targets to drones
            from app.optimization.vrp_solver import VRPSolver
            vrp_solver = VRPSolver(self.mission)
            assignments = vrp_solver.solve()
            
            if not assignments:
                error_message = f"VRP solver failed to assign targets to drones. Mission has {len(self.mission.drones)} drone(s) and {len(self.mission.target_points)} target(s)."
                return routes, error_message
            
            # Debug: Log assignments
            # Ensure all drones are in assignments
            for drone in self.mission.drones:
                if drone.name not in assignments:
                    assignments[drone.name] = []
            
            # Step 2: Build trajectories for each drone using pathfinding
            # Ensure all drones get routes, even if they have no targets assigned
            for drone in self.mission.drones:
                target_indices = assignments.get(drone.name, [])
                
                # If no targets assigned, still create a minimal route
                if not target_indices:
                    # Create a route from depot to finish point (if finish is different from depot)
                    if self.mission.depot:
                        waypoints = [self.mission.depot]
                        # Add finish point if it's different from depot
                        if self.mission.finish_point_type == "custom" and self.mission.finish_point:
                            if (self.mission.finish_point.latitude != self.mission.depot.latitude or 
                                self.mission.finish_point.longitude != self.mission.depot.longitude):
                                waypoints.append(self.mission.finish_point)
                        elif self.mission.finish_point_type == "depot":
                            # Finish is depot, so route is just depot
                            pass
                        # For "last_target", there are no targets, so finish is depot
                        
                        # Always create a route, even if it's just depot (for consistency)
                        if len(waypoints) >= 1:
                            route = Route(waypoints=waypoints, drone_name=drone.name)
                            # Get weather data from planner's weather manager
                            weather_for_metrics = self.weather_data
                            if hasattr(self.planner, 'weather_manager') and self.planner.weather_manager:
                                weather_for_metrics = self.planner.weather_manager.get_all_weather_data()
                            route.calculate_metrics(drone, weather_for_metrics)
                            routes[drone.name] = route
                    continue
                
                drone_targets = [self.mission.target_points[i] for i in target_indices if i < len(self.mission.target_points)]
                
                if drone_targets:
                    # Create temporary mission for this drone with same route options
                    temp_mission = Mission(
                        name=f"{self.mission.name}_drone_{drone.name}",
                        drones=[drone],
                        target_points=drone_targets,
                        depot=self.mission.depot,
                        finish_point=self.mission.finish_point,
                        finish_point_type=self.mission.finish_point_type,
                        landing_mode=getattr(self.mission, 'landing_mode', 'vertical'),
                        constraints=self.mission.constraints
                    )
                    
                    # Plan route using pathfinding algorithm
                    # Use same weather manager from main planner for consistency
                    temp_planner = RoutePlanner(
                        temp_mission, 
                        weather_data=self.weather_data,
                        use_weather=self.use_weather,
                        weather_timestamp=self.weather_timestamp
                    )
                    # Share weather manager to maintain cache
                    if hasattr(self.planner, 'weather_manager') and self.planner.weather_manager:
                        temp_planner.weather_manager = self.planner.weather_manager
                    
                    route = temp_planner.plan_single_drone_route(
                        drone,
                        algorithm=algorithm,
                        optimization_metric=optimization_metric
                    )
                    
                    # Update weather_data from temp_planner
                    if hasattr(temp_planner, 'weather_manager') and temp_planner.weather_manager:
                        self.weather_data.update(temp_planner.weather_manager.get_all_weather_data())
                    elif hasattr(temp_planner, 'weather_data'):
                        self.weather_data.update(temp_planner.weather_data)
                    
                    if route:
                        routes[drone.name] = route
                    else:
                        if not error_message:
                            error_message = f"Failed to plan route for {drone.name}"
                        else:
                            error_message += f"\nFailed to plan route for {drone.name}"
            
            if not routes:
                if not error_message:
                    error_message = "No routes found for multi-drone mission. Possible reasons:\n" \
                                  "- Target points are unreachable\n" \
                                  "- No-fly zones block all possible paths\n" \
                                  "- Constraints are too restrictive"
        
        # Optimize routes with optimization algorithm if requested
        if optimization_algorithm and routes:
            from app.optimization.mission_optimizer import MissionOptimizer
            # Store original routes as backup
            original_routes = routes.copy()
            
            # Temporarily store routes in mission for optimization
            for drone_name, route in routes.items():
                self.mission.add_route(drone_name, route)
            
            # Optimize all routes
            optimizer = MissionOptimizer(self.mission)
            optimized_routes = optimizer.optimize_routes(optimization_algorithm=optimization_algorithm)
            
            # Ensure all drones have routes (optimizer should preserve all routes)
            if optimized_routes:
                # Check if all original routes are present in optimized routes
                missing_routes = set(original_routes.keys()) - set(optimized_routes.keys())
                if missing_routes:
                    # Add missing routes from originals
                    for drone_name in missing_routes:
                        optimized_routes[drone_name] = original_routes[drone_name]
                routes = optimized_routes
            else:
                # Optimization failed, use original routes
                routes = original_routes
        
        # Validate routes
        for drone_name, route in routes.items():
            drone = next((d for d in self.mission.drones if d.name == drone_name), None)
            if drone:
                validation_result = self.checker.validate_route(route, drone, self.mission.constraints)
                route.validation_result = validation_result.to_dict() if hasattr(validation_result, 'to_dict') else validation_result
        
        # Store routes in mission
        for drone_name, route in routes.items():
            self.mission.add_route(drone_name, route)
        
        # Store graph for visualization
        self.current_graph = self.planner.current_graph
        
        # Update weather_data from planner's weather manager for visualization
        if hasattr(self.planner, 'weather_manager') and self.planner.weather_manager:
            self.weather_data = self.planner.weather_manager.get_all_weather_data()
        elif hasattr(self.planner, 'weather_data'):
            self.weather_data = self.planner.weather_data
        
        return routes, error_message
    
    def _check_target_points_against_zones(self) -> List[Dict]:
        """Check if target points are in no-fly zones.
        
        Returns:
            List of violation dictionaries
        """
        violations = []
        
        if not self.mission.constraints or not self.mission.constraints.no_fly_zones:
            return violations
        
        from shapely.geometry import Point
        
        # Check depot
        if self.mission.depot:
            point = Point(self.mission.depot.longitude, self.mission.depot.latitude)
            for zone in self.mission.constraints.no_fly_zones:
                if zone.contains(point, self.mission.depot.altitude):
                    zone_name = zone.name or "unnamed"
                    violations.append({
                        "message": f"Depot is in no-fly zone: {zone_name}",
                        "waypoint_index": None
                    })
        
        # Check target points
        for idx, target in enumerate(self.mission.target_points):
            point = Point(target.longitude, target.latitude)
            for zone in self.mission.constraints.no_fly_zones:
                if zone.contains(point, target.altitude):
                    zone_name = zone.name or "unnamed"
                    violations.append({
                        "message": f"Target point {idx + 1} ({target.name or 'Unnamed'}) is in no-fly zone: {zone_name}",
                        "waypoint_index": idx
                    })
        
        return violations
    
    def replan_route(self, drone_name: str, use_weather: bool = True) -> Optional[Route]:
        """Replan route for a specific drone.
        
        Args:
            drone_name: Name of drone to replan
        
        Returns:
            New Route, or None if planning fails
        """
        drone = next((d for d in self.mission.drones if d.name == drone_name), None)
        if not drone:
            return None
        
        route = self.planner.plan_single_drone_route(drone)
        if route:
            validation_result = self.checker.validate_route(route, drone, self.mission.constraints)
            route.validation_result = validation_result.to_dict() if hasattr(validation_result, 'to_dict') else validation_result
            self.mission.add_route(drone_name, route)
        
        return route

