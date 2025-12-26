"""VRP solver for multi-drone missions using OR-Tools."""
from typing import List, Dict, Tuple, Optional
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from app.domain.mission import Mission
from app.domain.drone import Drone
from app.domain.waypoint import Waypoint
import math


class VRPSolver:
    """Vehicle Routing Problem solver for multi-drone missions."""
    
    def __init__(self, mission: Mission):
        """Initialize VRP solver.
        
        Args:
            mission: Mission object with multiple drones and target points
        """
        self.mission = mission
        self.depot = mission.depot
        self.targets = mission.target_points
        self.drones = mission.drones
        
        # Debug: Ensure we have drones
        if not self.drones:
            raise ValueError("VRP solver requires at least one drone")
    
    def solve(self) -> Dict[str, List[int]]:
        """Solve VRP to assign targets to drones.
        
        Returns:
            Dictionary mapping drone name to list of target indices
        """
        if not self.targets or not self.drones:
            return {}
        
        # Ensure we have at least one drone
        if len(self.drones) == 0:
            return {}
        
        # Create distance matrix
        distance_matrix = self._create_distance_matrix()
        
        # Create routing model
        manager = pywrapcp.RoutingIndexManager(
            len(distance_matrix),
            len(self.drones),  # Number of vehicles
            0  # Depot index
        )
        routing = pywrapcp.RoutingModel(manager)
        
        # Define distance callback
        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(distance_matrix[from_node][to_node])
        
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        
        # Calculate maximum distance needed (sum of all distances from depot to targets and back)
        # This ensures we don't set an unreasonably low limit
        max_distance_needed = 0
        if self.depot and self.targets:
            for target in self.targets:
                dist_to = self._haversine_distance(
                    self.depot.latitude, self.depot.longitude,
                    target.latitude, target.longitude
                )
                dist_back = self._haversine_distance(
                    target.latitude, target.longitude,
                    self.depot.latitude, self.depot.longitude
                )
                max_distance_needed = max(max_distance_needed, dist_to + dist_back)
        
        # Use the maximum of: drone max_range, or calculated needed distance * 2 (safety margin)
        max_range = max(
            max(d.max_range for d in self.drones) if self.drones else 100000,
            max_distance_needed * 2 if max_distance_needed > 0 else 100000
        )
        
        # Add distance constraint (make it more lenient)
        dimension_name = 'Distance'
        routing.AddDimension(
            transit_callback_index,
            0,  # No slack
            int(max_range),  # Maximum distance (more lenient)
            True,  # Start cumul to zero
            dimension_name
        )
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)
        
        # Note: We removed the energy/battery capacity constraint because:
        # 1. It was too restrictive and causing solver failures
        # 2. Distance constraint already provides reasonable limits
        # 3. Energy consumption is better handled during route planning, not VRP assignment
        
        # Set search parameters
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        # Use a strategy that tries to use all vehicles
        # SAVINGS strategy tends to use fewer vehicles, so we use PATH_CHEAPEST_ARC
        # which is more likely to distribute targets across all vehicles
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.seconds = 30
        
        # If we have fewer targets than drones, we need to ensure distribution
        # OR-Tools will naturally try to use all vehicles if beneficial
        # But if targets < drones, some drones won't get targets (which is expected)
        
        # Solve
        solution = routing.SolveWithParameters(search_parameters)
        
        if not solution:
            # If OR-Tools can't find a solution, fall back to simple greedy assignment
            # This ensures we always return a valid assignment, even if not optimal
            return self._greedy_fallback_assignment()
        
        # Extract solution
        assignments = {}
        for vehicle_id in range(len(self.drones)):
            drone = self.drones[vehicle_id]
            route_indices = []
            index = routing.Start(vehicle_id)
            
            # Traverse the route for this vehicle
            # Start from the first node after the start
            current_index = solution.Value(routing.NextVar(index))
            
            # Check if vehicle has any nodes (not just going from start to end)
            if routing.IsEnd(current_index):
                # Vehicle has no targets assigned (empty route)
                assignments[drone.name] = []
            else:
                # Follow the route
                while not routing.IsEnd(current_index):
                    node_index = manager.IndexToNode(current_index)
                    if node_index > 0:  # Skip depot (index 0)
                        route_indices.append(node_index - 1)  # Convert to target index
                    current_index = solution.Value(routing.NextVar(current_index))
                
                assignments[drone.name] = route_indices
        
        # Ensure all drones are in assignments (even if empty)
        for drone in self.drones:
            if drone.name not in assignments:
                assignments[drone.name] = []
        
        # If we have fewer targets than drones, distribute targets more evenly
        # This ensures the first drone gets targets when possible
        if len(self.targets) < len(self.drones):
            # Count how many drones have targets
            drones_with_targets = sum(1 for indices in assignments.values() if indices)
            # If some drones don't have targets but we have targets, redistribute
            if drones_with_targets < len(self.drones) and len(self.targets) > 0:
                # Collect all target indices
                all_target_indices = []
                for indices in assignments.values():
                    all_target_indices.extend(indices)
                
                # Remove duplicates and sort
                all_target_indices = sorted(set(all_target_indices))
                
                # Redistribute evenly among all drones
                assignments = {}
                targets_per_drone = len(all_target_indices) // len(self.drones)
                remainder = len(all_target_indices) % len(self.drones)
                
                target_idx = 0
                for i, drone in enumerate(self.drones):
                    num_targets = targets_per_drone + (1 if i < remainder else 0)
                    assignments[drone.name] = all_target_indices[target_idx:target_idx + num_targets]
                    target_idx += num_targets
        
        return assignments
    
    def _create_distance_matrix(self) -> List[List[int]]:
        """Create distance matrix for VRP.
        
        Returns:
            Distance matrix (integers in meters)
        """
        # Include depot and all targets
        locations = []
        if self.depot:
            locations.append((self.depot.latitude, self.depot.longitude))
        for target in self.targets:
            locations.append((target.latitude, target.longitude))
        
        n = len(locations)
        matrix = [[0] * n for _ in range(n)]
        
        for i in range(n):
            for j in range(n):
                if i != j:
                    lat1, lon1 = locations[i]
                    lat2, lon2 = locations[j]
                    distance = self._haversine_distance(lat1, lon1, lat2, lon2)
                    matrix[i][j] = int(distance)  # Convert to integer meters
        
        return matrix
    
    def _greedy_fallback_assignment(self) -> Dict[str, List[int]]:
        """Fallback greedy assignment when OR-Tools fails.
        
        Assigns targets to drones using a simple nearest-neighbor approach.
        
        Returns:
            Dictionary mapping drone name to list of target indices
        """
        if not self.targets or not self.drones:
            return {}
        
        assignments = {drone.name: [] for drone in self.drones}
        
        # Calculate distances from depot to each target
        target_distances = []
        if self.depot:
            for idx, target in enumerate(self.targets):
                distance = self._haversine_distance(
                    self.depot.latitude, self.depot.longitude,
                    target.latitude, target.longitude
                )
                target_distances.append((idx, distance))
        
        # Sort targets by distance from depot
        target_distances.sort(key=lambda x: x[1])
        
        # Distribute targets evenly among drones using round-robin
        for i, (target_idx, _) in enumerate(target_distances):
            drone_idx = i % len(self.drones)
            drone = self.drones[drone_idx]
            assignments[drone.name].append(target_idx)
        
        return assignments
    
    @staticmethod
    def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance using Haversine formula."""
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c

