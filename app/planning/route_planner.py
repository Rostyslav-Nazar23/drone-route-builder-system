"""Route planner for single and multi-drone missions."""
from typing import List, Optional, Dict
from datetime import datetime
from app.domain.mission import Mission
from app.domain.route import Route
from app.domain.drone import Drone
from app.domain.waypoint import Waypoint
from app.environment.graph_builder import GraphBuilder
from app.environment.navigation_graph import NavigationGraph
from app.planning.a_star import AStar
from app.planning.theta_star import ThetaStar
from app.planning.d_star import DStar
from app.weather.weather_provider import WeatherConditions, WeatherProvider
from app.weather.weather_manager import WeatherManager


class RoutePlanner:
    """Planner for generating routes."""
    
    def __init__(self, mission: Mission, 
                 weather_data: Optional[Dict[tuple[float, float], WeatherConditions]] = None,
                 use_weather: bool = True,
                 weather_timestamp: Optional[datetime] = None):
        """Initialize route planner with mission.
        
        Args:
            mission: Mission object
            weather_data: Dictionary mapping (lat, lon) to WeatherConditions (optional, used as initial cache)
            use_weather: Whether to fetch and use weather data during route planning
            weather_timestamp: Timestamp for weather data (default: current time)
        """
        self.mission = mission
        self.use_weather = use_weather
        self.weather_timestamp = weather_timestamp or datetime.now()
        
        # Initialize weather manager for integrated weather fetching
        self.weather_manager = WeatherManager(
            weather_provider=WeatherProvider(),
            initial_weather_data=weather_data,
            use_weather=use_weather
        )
        
        # Pre-fetch weather for mission waypoints
        if use_weather:
            self.weather_manager.pre_fetch_weather_for_mission(mission, self.weather_timestamp)
        
        # Keep weather_data for backward compatibility (now uses weather_manager)
        self.weather_data = self.weather_manager.get_all_weather_data()
        self.current_graph: Optional[NavigationGraph] = None  # Store graph for visualization
    
    def plan_single_drone_route(self, drone: Drone, 
                               algorithm: str = "astar",
                               optimization_metric: str = "distance") -> Optional[Route]:
        """Plan route for a single drone.
        
        Args:
            drone: Drone to plan route for
            algorithm: Pathfinding algorithm to use ("astar", "thetastar", "dstar")
            optimization_metric: Optimization metric ("distance", "energy", "time")
        
        Returns:
            Route object, or None if planning fails
        """
        if not self.mission.target_points:
            return None
        
        # Build navigation graph with weather manager for dynamic weather fetching
        # Pass weather_manager to GraphBuilder so it can fetch weather during graph building
        graph_builder = GraphBuilder(
            drone, 
            self.mission.constraints, 
            self.weather_manager.get_all_weather_data(),  # Initial weather cache
            weather_manager=self.weather_manager  # For dynamic fetching
        )
        
        # Determine finish point early (before building graph)
        finish_point = None
        finish_node = None
        
        if self.mission.finish_point_type == "depot" and self.mission.depot:
            finish_point = self.mission.depot
        elif self.mission.finish_point_type == "custom" and self.mission.finish_point:
            finish_point = self.mission.finish_point
        elif self.mission.constraints and self.mission.constraints.require_return_to_depot and self.mission.depot:
            # Fallback to old behavior
            finish_point = self.mission.depot
        
        # Build waypoint graph
        all_waypoints = [self.mission.depot] if self.mission.depot else []
        all_waypoints.extend(self.mission.target_points)
        
        # Add custom finish point to waypoints if needed
        if finish_point and finish_point not in all_waypoints:
            all_waypoints.append(finish_point)
        
        graph = graph_builder.build_waypoint_graph(
            all_waypoints,
            connect_all=True,
            max_distance=drone.max_range
        )
        self.current_graph = graph  # Store for visualization
        
        start_node = "wp_0" if self.mission.depot else "wp_0"
        target_nodes = [f"wp_{i+1}" for i in range(len(self.mission.target_points))]
        
        # Determine finish node for waypoint graph
        if finish_point:
            # Find finish point index in all_waypoints
            finish_idx = all_waypoints.index(finish_point)
            finish_node = f"wp_{finish_idx}"
        
        if not start_node or not target_nodes:
            return None
        
        # Select pathfinding algorithm
        if algorithm == "thetastar":
            pathfinder = ThetaStar(graph)
        elif algorithm == "dstar":
            pathfinder = DStar(graph)
        else:  # default to astar
            pathfinder = AStar(graph)
        
        # Optimize target order based on optimization metric (before adding return to depot)
        if len(target_nodes) > 1:
            target_nodes = self._optimize_waypoint_order(
                graph, start_node, target_nodes, drone, optimization_metric
            )
        
        # Handle finish point based on type
        if self.mission.finish_point_type == "last_target":
            # The last target in the optimized order will be the finish
            # No need to add additional finish node - route ends at last target
            pass
        elif finish_node:
            # Add finish node to the end (for "depot" or "custom")
            target_nodes.append(finish_node)
        
        # Find path visiting all targets in optimized order
        path_nodes = pathfinder.find_path_to_waypoints(start_node, target_nodes)
        
        if not path_nodes:
            return None
        
        # Convert to route
        waypoints = pathfinder.path_to_waypoints(path_nodes)
        
        # Ensure first waypoint (depot) has correct type
        if waypoints and self.mission.depot:
            waypoints[0].waypoint_type = "depot"
        
        # Handle landing approach for finish point (for all finish types)
        if waypoints:
            # Find last target point (before finish) - needed for both landing modes
            last_target_idx = None
            for i in range(len(waypoints) - 1, -1, -1):
                if waypoints[i].waypoint_type == "target":
                    last_target_idx = i
                    break
            
            # Handle landing mode
            landing_mode = getattr(self.mission, 'landing_mode', 'vertical')
            
            if self.mission.finish_point_type == "last_target":
                # For last_target, the last target IS the finish point
                if waypoints and last_target_idx is not None:
                    # Mark the last target as finish
                    waypoints[last_target_idx].waypoint_type = "target"  # Keep as target, but will be treated as finish
                    
                    # Handle landing mode for last_target finish
                    if landing_mode == "vertical":
                        # Vertical landing: fly to last target at min flight altitude, then land vertically
                        last_target = waypoints[last_target_idx]
                        # Get the last target altitude (we'll maintain this or use min altitude)
                        last_target_alt = last_target.altitude
                        # Use the higher of last target altitude or min flight altitude
                        approach_altitude = max(drone.min_altitude, last_target_alt)
                        
                        # Set all intermediate waypoints between previous target and last target to approach altitude
                        # This ensures they don't gradually descend
                        if last_target_idx > 0:
                            for i in range(last_target_idx, len(waypoints)):
                                if waypoints[i].waypoint_type not in ["depot", "target"]:
                                    # Set altitude to approach altitude (maintain flight altitude)
                                    waypoints[i].altitude = approach_altitude
                                    waypoints[i].waypoint_type = "landing_segment"
                        
                        # Add waypoint at last target location but at min flight altitude
                        target_at_min_alt = Waypoint(
                            latitude=last_target.latitude,
                            longitude=last_target.longitude,
                            altitude=drone.min_altitude,
                            waypoint_type="landing_approach"
                        )
                        # Insert before last target
                        waypoints.insert(last_target_idx, target_at_min_alt)
                        # Final target point is on ground (at its original altitude)
                        waypoints[-1].waypoint_type = "finish"
                    elif landing_mode == "gradual":
                        # Gradual landing: descend from previous target to last target (may go below min alt)
                        # Mark waypoints between previous target and last target as landing_segment
                        if last_target_idx > 0:
                            for i in range(last_target_idx, len(waypoints)):
                                if waypoints[i].waypoint_type not in ["depot", "target"]:
                                    waypoints[i].waypoint_type = "landing_segment"
                        # Mark last target as finish
                        waypoints[-1].waypoint_type = "finish"
            else:
                # For "depot" or "custom" finish types
                if self.mission.finish_point_type == "depot":
                    waypoints[-1].waypoint_type = "depot"
                elif self.mission.finish_point_type == "custom" and self.mission.finish_point:
                    waypoints[-1].waypoint_type = "finish"
                
                if landing_mode == "vertical":
                    # Vertical landing: fly to finish at min flight altitude, then land vertically
                    if last_target_idx is not None:
                        # For vertical landing, we need to:
                        # 1. Set all intermediate waypoints between last target and finish to min flight altitude
                        # 2. Add landing_approach waypoint at min altitude
                        # 3. Add final finish point on ground
                        
                        # Get the last target altitude (we'll maintain this or use min altitude)
                        last_target_alt = waypoints[last_target_idx].altitude
                        # Use the higher of last target altitude or min flight altitude
                        approach_altitude = max(drone.min_altitude, last_target_alt)
                        
                        # Set all intermediate waypoints between last target and finish to approach altitude
                        # This ensures they don't gradually descend
                        for i in range(last_target_idx + 1, len(waypoints)):
                            if waypoints[i].waypoint_type not in ["depot", "finish"]:
                                # Set altitude to approach altitude (maintain flight altitude)
                                waypoints[i].altitude = approach_altitude
                                waypoints[i].waypoint_type = "landing_segment"
                        
                        # Add waypoint at finish location but at min flight altitude
                        finish_location = self.mission.finish_point if self.mission.finish_point else self.mission.depot
                        if finish_location:
                            finish_at_min_alt = Waypoint(
                                latitude=finish_location.latitude,
                                longitude=finish_location.longitude,
                                altitude=drone.min_altitude,
                                waypoint_type="landing_approach"
                            )
                            # Insert before final finish point
                            waypoints.insert(-1, finish_at_min_alt)
                            # Final point is on ground
                            if self.mission.finish_point_type == "depot":
                                waypoints[-1].waypoint_type = "depot"
                                waypoints[-1].altitude = self.mission.depot.altitude
                            elif self.mission.finish_point_type == "custom":
                                waypoints[-1].waypoint_type = "finish"
                                waypoints[-1].altitude = self.mission.finish_point.altitude
                            
                            # After inserting landing_approach, ensure all waypoints between last target and landing_approach
                            # maintain approach altitude (not descending)
                            for i in range(last_target_idx + 1, len(waypoints) - 2):  # -2 to exclude landing_approach and finish
                                if waypoints[i].waypoint_type not in ["depot", "finish", "landing_approach"]:
                                    waypoints[i].altitude = approach_altitude
                                    waypoints[i].waypoint_type = "landing_segment"
                elif landing_mode == "gradual":
                    # Gradual landing: descend from last target to finish (may go below min alt)
                    if last_target_idx is not None:
                        # Mark ALL waypoints between last target and finish as landing segment
                        # This includes intermediate waypoints added by algorithms (e.g., Theta*)
                        for i in range(last_target_idx + 1, len(waypoints) - 1):
                            # Mark all waypoints (including intermediate) as landing_segment
                            waypoints[i].waypoint_type = "landing_segment"
                        
                        # Ensure finish point has correct type
                        if self.mission.finish_point_type == "depot":
                            waypoints[-1].waypoint_type = "depot"
                        elif self.mission.finish_point_type == "custom" and self.mission.finish_point:
                            waypoints[-1].waypoint_type = "finish"
        
        route = Route(waypoints=waypoints, drone_name=drone.name)
        route.calculate_metrics(drone, self.weather_data)
        
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
    
    def _optimize_waypoint_order(self, graph: NavigationGraph, start_node: str,
                                target_nodes: List[str], drone: Drone,
                                optimization_metric: str = "distance") -> List[str]:
        """Optimize the order of waypoints using the specified metric.
        
        Uses a greedy nearest-neighbor approach with the selected optimization metric.
        
        Args:
            graph: Navigation graph
            start_node: Starting node
            target_nodes: List of target node IDs to visit
            drone: Drone capabilities
            optimization_metric: "distance", "energy", or "time"
        
        Returns:
            Optimized list of target node IDs
        """
        if len(target_nodes) <= 1:
            return target_nodes
        
        # Build cost matrix between all nodes
        all_nodes = [start_node] + target_nodes
        cost_matrix = {}
        
        for i, node1 in enumerate(all_nodes):
            for j, node2 in enumerate(all_nodes):
                if i != j:
                    pos1 = graph.get_node_position(node1)
                    pos2 = graph.get_node_position(node2)
                    
                    if optimization_metric == "energy":
                        # Calculate energy cost
                        horizontal_dist = self._haversine_distance(
                            pos1[1], pos1[0], pos2[1], pos2[0]
                        )
                        altitude_change = pos2[2] - pos1[2]
                        cost = drone.estimate_energy_consumption(horizontal_dist, altitude_change)
                    elif optimization_metric == "time":
                        # Calculate time cost
                        distance = self._calculate_distance(
                            pos1[1], pos1[0], pos1[2],
                            pos2[1], pos2[0], pos2[2]
                        )
                        cost = distance / drone.max_speed
                    else:  # distance (default)
                        # Calculate distance cost
                        cost = self._calculate_distance(
                            pos1[1], pos1[0], pos1[2],
                            pos2[1], pos2[0], pos2[2]
                        )
                    
                    cost_matrix[(node1, node2)] = cost
        
        # Greedy nearest-neighbor algorithm
        visited = set()
        current = start_node
        optimized_order = []
        
        while len(optimized_order) < len(target_nodes):
            best_node = None
            best_cost = float('inf')
            
            for target in target_nodes:
                if target not in visited:
                    cost = cost_matrix.get((current, target), float('inf'))
                    if cost < best_cost:
                        best_cost = cost
                        best_node = target
            
            if best_node:
                optimized_order.append(best_node)
                visited.add(best_node)
                current = best_node
            else:
                break
        
        return optimized_order
    
    @staticmethod
    def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate horizontal distance using Haversine formula."""
        from math import radians, sin, cos, sqrt, atan2
        
        R = 6371000  # Earth radius in meters
        
        lat1_rad = radians(lat1)
        lat2_rad = radians(lat2)
        delta_lat = radians(lat2 - lat1)
        delta_lon = radians(lon2 - lon1)
        
        a = sin(delta_lat / 2) ** 2 + cos(lat1_rad) * cos(lat2_rad) * sin(delta_lon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        
        return R * c
    
    @staticmethod
    def _calculate_distance(lat1: float, lon1: float, alt1: float,
                           lat2: float, lon2: float, alt2: float) -> float:
        """Calculate 3D distance between two points."""
        horizontal_dist = RoutePlanner._haversine_distance(lat1, lon1, lat2, lon2)
        vertical_dist = abs(alt2 - alt1)
        return (horizontal_dist ** 2 + vertical_dist ** 2) ** 0.5

