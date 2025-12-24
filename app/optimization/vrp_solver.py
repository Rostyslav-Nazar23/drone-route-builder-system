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
    
    def solve(self) -> Dict[str, List[int]]:
        """Solve VRP to assign targets to drones.
        
        Returns:
            Dictionary mapping drone name to list of target indices
        """
        if not self.targets or not self.drones:
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
        
        # Add distance constraint
        dimension_name = 'Distance'
        routing.AddDimension(
            transit_callback_index,
            0,  # No slack
            int(max(d.max_range for d in self.drones) if self.drones else 100000),  # Maximum distance
            True,  # Start cumul to zero
            dimension_name
        )
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)
        
        # Set vehicle capacities (battery/energy)
        for vehicle_id in range(len(self.drones)):
            drone = self.drones[vehicle_id]
            # Use battery capacity as vehicle capacity
            capacity = int(drone.battery_capacity * 100)  # Convert to integer
            routing.AddDimension(
                transit_callback_index,
                0,
                capacity,
                True,
                'Energy'
            )
        
        # Set search parameters
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.seconds = 30
        
        # Solve
        solution = routing.SolveWithParameters(search_parameters)
        
        if not solution:
            return {}
        
        # Extract solution
        assignments = {}
        for vehicle_id in range(len(self.drones)):
            drone = self.drones[vehicle_id]
            route_indices = []
            index = routing.Start(vehicle_id)
            
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                if node_index > 0:  # Skip depot (index 0)
                    route_indices.append(node_index - 1)  # Convert to target index
                index = solution.Value(routing.NextVar(index))
            
            assignments[drone.name] = route_indices
        
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

