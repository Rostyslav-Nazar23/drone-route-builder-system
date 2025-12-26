"""Ant Colony Optimization (ACO) for route optimization."""
from typing import List, Dict, Optional
import random
import math
import copy
from shapely.geometry import LineString, Point
from app.domain.route import Route
from app.domain.waypoint import Waypoint
from app.domain.drone import Drone
from app.domain.constraints import MissionConstraints


class ACOOptimizer:
    """Ant Colony Optimization for optimizing routes."""
    
    def __init__(self, route: Route, drone: Drone,
                 num_ants: int = 30,
                 iterations: int = 100,
                 alpha: float = 1.0,  # Pheromone importance
                 beta: float = 2.0,    # Heuristic importance
                 evaporation: float = 0.1,  # Pheromone evaporation rate
                 q: float = 100.0,   # Pheromone deposit constant
                 constraints: Optional[MissionConstraints] = None):
        """Initialize ACO optimizer.
        
        Args:
            route: Initial route to optimize
            drone: Drone capabilities
            num_ants: Number of ants in colony
            iterations: Number of iterations
            alpha: Pheromone importance parameter
            beta: Heuristic importance parameter
            evaporation: Pheromone evaporation rate
            q: Pheromone deposit constant
            constraints: Mission constraints (for no-fly zone checking)
        """
        self.route = route
        self.drone = drone
        self.constraints = constraints
        self.num_ants = num_ants
        self.iterations = iterations
        self.alpha = alpha
        self.beta = beta
        self.evaporation = evaporation
        self.q = q
        
        # Extract waypoints (keep first and last fixed)
        self.waypoints = route.waypoints
        if len(self.waypoints) < 3:
            self.fixed_start = []
            self.fixed_end = []
            self.middle_waypoints = []
        else:
            self.fixed_start = [self.waypoints[0]]
            self.fixed_end = [self.waypoints[-1]]
            self.middle_waypoints = self.waypoints[1:-1]
        
        # Initialize pheromone matrix
        self.pheromone = {}
        self._initialize_pheromones()
    
    def optimize(self) -> Route:
        """Optimize route using ACO.
        
        Returns:
            Optimized route
        """
        if len(self.middle_waypoints) < 2:
            return self.route
        
        best_route = None
        best_cost = float('inf')
        
        for iteration in range(self.iterations):
            ant_routes = []
            
            # Each ant constructs a solution
            for ant in range(self.num_ants):
                route = self._construct_solution()
                cost = self._calculate_cost(route)
                ant_routes.append((route, cost))
                
                if cost < best_cost:
                    best_cost = cost
                    best_route = route
            
            # Update pheromones
            self._update_pheromones(ant_routes)
        
        if best_route:
            optimized_waypoints = self.fixed_start + best_route + self.fixed_end
            optimized_route = Route(waypoints=optimized_waypoints, drone_name=self.route.drone_name)
            optimized_route.calculate_metrics(self.drone, None)
            return optimized_route
        
        return self.route
    
    def _initialize_pheromones(self):
        """Initialize pheromone matrix."""
        n = len(self.middle_waypoints)
        initial_pheromone = 1.0
        
        for i in range(n):
            for j in range(n):
                if i != j:
                    self.pheromone[(i, j)] = initial_pheromone
    
    def _construct_solution(self) -> List[Waypoint]:
        """Construct a solution (route) for one ant."""
        if not self.middle_waypoints:
            return []
        
        unvisited = list(range(len(self.middle_waypoints)))
        route = []
        current = random.choice(unvisited)
        unvisited.remove(current)
        route.append(self.middle_waypoints[current])
        
        while unvisited:
            next_idx = self._select_next(current, unvisited)
            route.append(self.middle_waypoints[next_idx])
            current = next_idx
            unvisited.remove(next_idx)
        
        return route
    
    def _select_next(self, current: int, unvisited: List[int]) -> int:
        """Select next waypoint using probability based on pheromone and heuristic."""
        probabilities = []
        
        for next_idx in unvisited:
            pheromone = self.pheromone.get((current, next_idx), 1.0)
            heuristic = 1.0 / (self._distance(current, next_idx) + 0.001)
            
            prob = (pheromone ** self.alpha) * (heuristic ** self.beta)
            probabilities.append((next_idx, prob))
        
        # Normalize probabilities
        total = sum(prob for _, prob in probabilities)
        if total == 0:
            return random.choice(unvisited)
        
        probabilities = [(idx, prob / total) for idx, prob in probabilities]
        
        # Roulette wheel selection
        r = random.random()
        cumulative = 0.0
        for idx, prob in probabilities:
            cumulative += prob
            if r <= cumulative:
                return idx
        
        return probabilities[-1][0]
    
    def _distance(self, idx1: int, idx2: int) -> float:
        """Calculate distance between two waypoints."""
        wp1 = self.middle_waypoints[idx1]
        wp2 = self.middle_waypoints[idx2]
        return self._haversine_distance(wp1.latitude, wp1.longitude, wp2.latitude, wp2.longitude)
    
    def _calculate_cost(self, route: List[Waypoint]) -> float:
        """Calculate total cost of a route."""
        if not route:
            return float('inf')
        
        total_route = self.fixed_start + route + self.fixed_end
        
        # Check for no-fly zone violations first (heavily penalize)
        no_fly_penalty = self._check_no_fly_zones(total_route)
        if no_fly_penalty > 0:
            # Heavily penalize routes that cross no-fly zones
            return float('inf')
        
        cost = 0.0
        
        for i in range(len(total_route) - 1):
            wp1 = total_route[i]
            wp2 = total_route[i + 1]
            distance = self._haversine_distance(wp1.latitude, wp1.longitude, wp2.latitude, wp2.longitude)
            cost += distance
        
        return cost
    
    def _check_no_fly_zones(self, waypoints: List[Waypoint]) -> float:
        """Check if route intersects no-fly zones. Returns penalty value.
        
        Args:
            waypoints: List of waypoints to check
            
        Returns:
            Penalty value (0 if no violations, >0 if violations found)
        """
        if not self.constraints or not self.constraints.no_fly_zones:
            return 0.0
        
        penalty = 0.0
        
        # Check each waypoint
        for waypoint in waypoints:
            point = Point(waypoint.longitude, waypoint.latitude)
            for zone in self.constraints.no_fly_zones:
                if zone.contains(point, waypoint.altitude):
                    penalty += 10000.0  # Heavy penalty for waypoint in zone
        
        # Check route segments
        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            
            # Create 2D line segment for intersection check
            line_2d = LineString([
                (wp1.longitude, wp1.latitude),
                (wp2.longitude, wp2.latitude)
            ])
            
            for zone in self.constraints.no_fly_zones:
                # Check if 2D line intersects zone geometry
                if zone.geometry.intersects(line_2d):
                    # Check altitude range
                    min_alt = min(wp1.altitude, wp2.altitude)
                    max_alt = max(wp1.altitude, wp2.altitude)
                    
                    if zone.min_altitude <= max_alt and zone.max_altitude >= min_alt:
                        penalty += 10000.0  # Heavy penalty for segment crossing zone
        
        return penalty
    
    def _update_pheromones(self, ant_routes: List[tuple[List[Waypoint], float]]):
        """Update pheromone matrix based on ant solutions."""
        # Evaporate pheromones
        for key in self.pheromone:
            self.pheromone[key] *= (1.0 - self.evaporation)
        
        # Deposit pheromones based on solution quality
        for route, cost in ant_routes:
            if cost > 0:
                delta_pheromone = self.q / cost
                
                # Convert route to indices
                route_indices = []
                for wp in route:
                    if wp in self.middle_waypoints:
                        route_indices.append(self.middle_waypoints.index(wp))
                
                # Update pheromones along the route
                for i in range(len(route_indices) - 1):
                    idx1 = route_indices[i]
                    idx2 = route_indices[i + 1]
                    key = (idx1, idx2)
                    if key in self.pheromone:
                        self.pheromone[key] += delta_pheromone
    
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

