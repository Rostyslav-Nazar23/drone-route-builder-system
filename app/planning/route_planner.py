"""Route planner for single and multi-drone missions."""
from typing import List, Optional, Dict
from app.domain.mission import Mission
from app.domain.route import Route
from app.domain.drone import Drone
from app.environment.graph_builder import GraphBuilder
from app.environment.navigation_graph import NavigationGraph
from app.planning.a_star import AStar
from app.planning.theta_star import ThetaStar
from app.planning.d_star import DStar
from app.weather.weather_provider import WeatherConditions


class RoutePlanner:
    """Planner for generating routes."""
    
    def __init__(self, mission: Mission, weather_data: Optional[Dict[tuple[float, float], WeatherConditions]] = None):
        """Initialize route planner with mission.
        
        Args:
            mission: Mission object
            weather_data: Dictionary mapping (lat, lon) to WeatherConditions (optional)
        """
        self.mission = mission
        self.weather_data = weather_data
    
    def plan_single_drone_route(self, drone: Drone, use_grid: bool = True, 
                               algorithm: str = "astar") -> Optional[Route]:
        """Plan route for a single drone.
        
        Args:
            drone: Drone to plan route for
            use_grid: If True, use grid graph; if False, use waypoint graph
        
        Returns:
            Route object, or None if planning fails
        """
        if not self.mission.target_points:
            return None
        
        # Build navigation graph
        graph_builder = GraphBuilder(drone, self.mission.constraints, self.weather_data)
        
        if use_grid and self.mission.depot:
            # Build grid graph centered on depot
            graph = graph_builder.build_grid_graph(
                center_lat=self.mission.depot.latitude,
                center_lon=self.mission.depot.longitude,
                width=5000,  # 5km
                height=5000,
                resolution=200,  # 200m resolution
                min_altitude=drone.min_altitude,
                max_altitude=min(drone.max_altitude, 500),
                altitude_levels=5
            )
            
            # Find nearest nodes for depot and targets
            start_node = graph_builder.find_nearest_node(
                graph,
                self.mission.depot.latitude,
                self.mission.depot.longitude,
                self.mission.depot.altitude
            )
            
            target_nodes = []
            for target in self.mission.target_points:
                node = graph_builder.find_nearest_node(
                    graph,
                    target.latitude,
                    target.longitude,
                    target.altitude
                )
                if node:
                    target_nodes.append(node)
        else:
            # Build waypoint graph
            all_waypoints = [self.mission.depot] if self.mission.depot else []
            all_waypoints.extend(self.mission.target_points)
            
            graph = graph_builder.build_waypoint_graph(
                all_waypoints,
                connect_all=True,
                max_distance=drone.max_range
            )
            
            start_node = "wp_0" if self.mission.depot else "wp_0"
            target_nodes = [f"wp_{i+1}" for i in range(len(self.mission.target_points))]
        
        if not start_node or not target_nodes:
            return None
        
        # Select pathfinding algorithm
        if algorithm == "thetastar":
            pathfinder = ThetaStar(graph)
        elif algorithm == "dstar":
            pathfinder = DStar(graph)
        else:  # default to astar
            pathfinder = AStar(graph)
        
        # If we need to return to depot
        if self.mission.constraints and self.mission.constraints.require_return_to_depot:
            if use_grid and self.mission.depot:
                end_node = graph_builder.find_nearest_node(
                    graph,
                    self.mission.depot.latitude,
                    self.mission.depot.longitude,
                    self.mission.depot.altitude
                )
                target_nodes.append(end_node)
        else:
            end_node = target_nodes[-1] if target_nodes else None
        
        # Find path visiting all targets
        path_nodes = pathfinder.find_path_to_waypoints(start_node, target_nodes)
        
        if not path_nodes:
            return None
        
        # Convert to route
        waypoints = pathfinder.path_to_waypoints(path_nodes)
        route = Route(waypoints=waypoints, drone_name=drone.name)
        route.calculate_metrics(drone)
        
        return route
    
    def plan_multi_drone_routes(self, use_vrp: bool = True) -> dict[str, Route]:
        """Plan routes for multiple drones (simple assignment).
        
        Returns:
            Dictionary mapping drone name to Route
        """
        routes = {}
        
        if not self.mission.drones or not self.mission.target_points:
            return routes
        
        if use_vrp and len(self.mission.drones) > 1:
            # Use VRP solver for optimal assignment
            from app.optimization.vrp_solver import VRPSolver
            vrp_solver = VRPSolver(self.mission)
            assignments = vrp_solver.solve()
            
            # Plan routes for each drone based on VRP assignment
            for drone in self.mission.drones:
                target_indices = assignments.get(drone.name, [])
                drone_targets = [self.mission.target_points[i] for i in target_indices if i < len(self.mission.target_points)]
                
                if drone_targets:
                    temp_mission = Mission(
                        name=f"{self.mission.name}_drone_{drone.name}",
                        drones=[drone],
                        target_points=drone_targets,
                        depot=self.mission.depot,
                        constraints=self.mission.constraints
                    )
                    
                    planner = RoutePlanner(temp_mission, self.weather_data)
                    route = planner.plan_single_drone_route(drone)
                    
                    if route:
                        routes[drone.name] = route
        else:
            # Simple assignment: divide targets evenly among drones
            targets_per_drone = len(self.mission.target_points) // len(self.mission.drones)
            remainder = len(self.mission.target_points) % len(self.mission.drones)
            
            target_idx = 0
            for drone_idx, drone in enumerate(self.mission.drones):
                num_targets = targets_per_drone + (1 if drone_idx < remainder else 0)
                drone_targets = self.mission.target_points[target_idx:target_idx + num_targets]
                
                temp_mission = Mission(
                    name=f"{self.mission.name}_drone_{drone.name}",
                    drones=[drone],
                    target_points=drone_targets,
                    depot=self.mission.depot,
                    constraints=self.mission.constraints
                )
                
                planner = RoutePlanner(temp_mission, self.weather_data)
                route = planner.plan_single_drone_route(drone)
                
                if route:
                    routes[drone.name] = route
                
                target_idx += num_targets
        
        return routes

