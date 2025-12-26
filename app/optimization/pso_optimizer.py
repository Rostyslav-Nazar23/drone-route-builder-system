"""Particle Swarm Optimization (PSO) for route optimization."""
from typing import List, Dict, Optional, Tuple
import random
import math
import copy
from shapely.geometry import LineString, Point
from app.domain.route import Route
from app.domain.waypoint import Waypoint
from app.domain.drone import Drone
from app.domain.constraints import MissionConstraints


class Particle:
    """Represents a particle in PSO."""
    
    def __init__(self, waypoints: List[Waypoint], drone: Drone, constraints: Optional[MissionConstraints] = None):
        """Initialize particle.
        
        Args:
            waypoints: List of waypoints (route)
            drone: Drone capabilities
            constraints: Mission constraints (for no-fly zone checking)
        """
        self.waypoints = copy.deepcopy(waypoints)
        self.velocity = [0.0] * len(waypoints)  # Velocity for each waypoint
        self.best_waypoints = copy.deepcopy(waypoints)
        self.best_cost = float('inf')
        self.drone = drone
        self.constraints = constraints
        self._calculate_cost()
    
    def _calculate_cost(self):
        """Calculate and update particle cost."""
        # Check for no-fly zone violations first (heavily penalize)
        no_fly_penalty = self._check_no_fly_zones(self.waypoints)
        if no_fly_penalty > 0:
            # Heavily penalize routes that cross no-fly zones
            self.cost = float('inf')
            return
        
        cost = 0.0
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            distance = self._haversine_distance(wp1.latitude, wp1.longitude, wp2.latitude, wp2.longitude)
            cost += distance
        
        self.cost = cost
        
        if cost < self.best_cost:
            self.best_cost = cost
            self.best_waypoints = copy.deepcopy(self.waypoints)
    
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
    
    def update_velocity(self, global_best: List[Waypoint], w: float = 0.5, c1: float = 1.5, c2: float = 1.5):
        """Update particle velocity.
        
        Args:
            global_best: Global best solution
            w: Inertia weight
            c1: Cognitive coefficient
            c2: Social coefficient
        """
        for i in range(len(self.waypoints)):
            if i == 0 or i == len(self.waypoints) - 1:
                continue  # Keep start and end fixed
            
            r1 = random.random()
            r2 = random.random()
            
            # Calculate distance-based velocity (for waypoint order)
            # For PSO on route optimization, we use swap-based velocity
            cognitive = c1 * r1 * self._swap_distance(self.waypoints, self.best_waypoints, i)
            social = c2 * r2 * self._swap_distance(self.waypoints, global_best, i)
            
            self.velocity[i] = w * self.velocity[i] + cognitive + social
    
    def update_position(self):
        """Update particle position based on velocity."""
        # For route optimization, velocity represents probability of swapping
        for i in range(1, len(self.waypoints) - 1):
            if abs(self.velocity[i]) > 0.5:  # Threshold for swap
                # Find best swap position
                best_swap = i
                best_improvement = 0
                
                for j in range(1, len(self.waypoints) - 1):
                    if i != j:
                        # Try swap
                        self.waypoints[i], self.waypoints[j] = self.waypoints[j], self.waypoints[i]
                        old_cost = self.cost
                        self._calculate_cost()
                        improvement = old_cost - self.cost
                        
                        if improvement > best_improvement:
                            best_improvement = improvement
                            best_swap = j
                        
                        # Swap back
                        self.waypoints[i], self.waypoints[j] = self.waypoints[j], self.waypoints[i]
                        self.cost = old_cost
                
                if best_improvement > 0:
                    self.waypoints[i], self.waypoints[best_swap] = self.waypoints[best_swap], self.waypoints[i]
                    self._calculate_cost()
    
    def _swap_distance(self, waypoints1: List[Waypoint], waypoints2: List[Waypoint], idx: int) -> float:
        """Calculate swap distance (how far a waypoint is from its position in best solution)."""
        if idx >= len(waypoints1) or idx >= len(waypoints2):
            return 0.0
        
        wp = waypoints1[idx]
        try:
            best_idx = waypoints2.index(wp)
            return abs(idx - best_idx) / len(waypoints1)
        except ValueError:
            return 1.0
    
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


class PSOOptimizer:
    """Particle Swarm Optimization for optimizing routes."""
    
    def __init__(self, route: Route, drone: Drone,
                 num_particles: int = 30,
                 iterations: int = 100,
                 w: float = 0.5,      # Inertia weight
                 c1: float = 1.5,    # Cognitive coefficient
                 c2: float = 1.5,   # Social coefficient
                 constraints: Optional[MissionConstraints] = None):
        """Initialize PSO optimizer.
        
        Args:
            route: Initial route to optimize
            drone: Drone capabilities
            num_particles: Number of particles in swarm
            iterations: Number of iterations
            w: Inertia weight
            c1: Cognitive coefficient
            c2: Social coefficient
            constraints: Mission constraints (for no-fly zone checking)
        """
        self.route = route
        self.drone = drone
        self.constraints = constraints
        self.num_particles = num_particles
        self.iterations = iterations
        self.w = w
        self.c1 = c1
        self.c2 = c2
    
    def optimize(self) -> Route:
        """Optimize route using PSO.
        
        Returns:
            Optimized route
        """
        if len(self.route.waypoints) < 3:
            return self.route
        
        # Initialize particles
        particles = []
        for _ in range(self.num_particles):
            # Create random permutation of middle waypoints
            waypoints = copy.deepcopy(self.route.waypoints)
            if len(waypoints) > 2:
                middle = waypoints[1:-1]
                random.shuffle(middle)
                waypoints[1:-1] = middle
            
            particle = Particle(waypoints, self.drone, constraints=self.constraints)
            particles.append(particle)
        
        # Find global best
        global_best = min(particles, key=lambda p: p.cost)
        global_best_waypoints = copy.deepcopy(global_best.waypoints)
        
        # Iterate
        for iteration in range(self.iterations):
            for particle in particles:
                # Update velocity
                particle.update_velocity(global_best_waypoints, self.w, self.c1, self.c2)
                
                # Update position
                particle.update_position()
                
                # Update global best
                if particle.cost < global_best.cost:
                    global_best = particle
                    global_best_waypoints = copy.deepcopy(particle.waypoints)
        
        # Return best route
        optimized_route = Route(waypoints=global_best_waypoints, drone_name=self.route.drone_name)
        optimized_route.calculate_metrics(self.drone, None)
        return optimized_route

