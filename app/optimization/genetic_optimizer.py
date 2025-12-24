"""Genetic algorithm optimizer for route optimization."""
from typing import List, Dict, Optional, Tuple
import random
import copy
from app.domain.route import Route
from app.domain.waypoint import Waypoint
from app.domain.drone import Drone


class GeneticOptimizer:
    """Genetic algorithm for optimizing routes."""
    
    def __init__(self, route: Route, drone: Drone, 
                 population_size: int = 50,
                 generations: int = 100,
                 mutation_rate: float = 0.1,
                 crossover_rate: float = 0.7):
        """Initialize genetic optimizer.
        
        Args:
            route: Initial route to optimize
            drone: Drone capabilities
            population_size: Size of population
            generations: Number of generations
            mutation_rate: Probability of mutation
            crossover_rate: Probability of crossover
        """
        self.route = route
        self.drone = drone
        self.population_size = population_size
        self.generations = generations
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
    
    def optimize(self) -> Route:
        """Optimize route using genetic algorithm.
        
        Returns:
            Optimized route
        """
        if len(self.route.waypoints) < 3:
            return self.route  # Too few waypoints to optimize
        
        # Initialize population
        population = self._initialize_population()
        
        # Evolve
        for generation in range(self.generations):
            # Evaluate fitness
            fitness_scores = [self._fitness(individual) for individual in population]
            
            # Select parents
            parents = self._select_parents(population, fitness_scores)
            
            # Create new generation
            new_population = []
            for i in range(0, len(parents) - 1, 2):
                if random.random() < self.crossover_rate:
                    child1, child2 = self._crossover(parents[i], parents[i + 1])
                    new_population.extend([child1, child2])
                else:
                    new_population.extend([parents[i], parents[i + 1]])
            
            # Mutate
            for individual in new_population:
                if random.random() < self.mutation_rate:
                    self._mutate(individual)
            
            # Elitism: keep best individual
            best_idx = max(range(len(population)), key=lambda i: fitness_scores[i])
            new_population[0] = population[best_idx]
            
            population = new_population[:self.population_size]
        
        # Return best individual
        final_fitness = [self._fitness(individual) for individual in population]
        best_idx = max(range(len(population)), key=lambda i: final_fitness[i])
        best_route = Route(waypoints=population[best_idx], drone_name=self.route.drone_name)
        best_route.calculate_metrics(self.drone)
        
        return best_route
    
    def _initialize_population(self) -> List[List[Waypoint]]:
        """Initialize population with random route variations."""
        population = []
        
        # Original route
        population.append(copy.deepcopy(self.route.waypoints))
        
        # Random permutations
        for _ in range(self.population_size - 1):
            waypoints = copy.deepcopy(self.route.waypoints)
            # Keep first and last waypoint fixed (depot and return)
            if len(waypoints) > 2:
                middle = waypoints[1:-1]
                random.shuffle(middle)
                waypoints[1:-1] = middle
            population.append(waypoints)
        
        return population
    
    def _fitness(self, waypoints: List[Waypoint]) -> float:
        """Calculate fitness of a route (higher is better).
        
        Fitness considers:
        - Total distance (shorter is better)
        - Energy consumption (lower is better)
        - Smoothness (fewer sharp turns is better)
        """
        if len(waypoints) < 2:
            return 0.0
        
        # Calculate total distance
        total_distance = 0.0
        total_energy = 0.0
        turn_penalty = 0.0
        
        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            
            distance = self._haversine_distance(
                wp1.latitude, wp1.longitude,
                wp2.latitude, wp2.longitude
            )
            total_distance += distance
            
            altitude_change = wp2.altitude - wp1.altitude
            total_energy += self.drone.estimate_energy_consumption(distance, altitude_change)
            
            # Calculate turn angle penalty
            if i > 0:
                wp0 = waypoints[i - 1]
                angle = self._calculate_turn_angle(wp0, wp1, wp2)
                if angle > 45:  # Sharp turn
                    turn_penalty += (angle - 45) * 10
        
        # Fitness = 1 / (distance + energy + turn_penalty)
        # Normalize
        distance_norm = total_distance / 10000.0  # Normalize to 10km
        energy_norm = total_energy / 100.0  # Normalize to 100Wh
        turn_norm = turn_penalty / 1000.0
        
        fitness = 1.0 / (1.0 + distance_norm + energy_norm + turn_norm)
        return fitness
    
    def _select_parents(self, population: List[List[Waypoint]], 
                        fitness_scores: List[float]) -> List[List[Waypoint]]:
        """Select parents using tournament selection."""
        parents = []
        
        for _ in range(self.population_size):
            # Tournament of size 3
            tournament = random.sample(list(zip(population, fitness_scores)), 3)
            winner = max(tournament, key=lambda x: x[1])[0]
            parents.append(copy.deepcopy(winner))
        
        return parents
    
    def _crossover(self, parent1: List[Waypoint], parent2: List[Waypoint]) -> Tuple[List[Waypoint], List[Waypoint]]:
        """Perform order crossover (OX) for route optimization."""
        if len(parent1) < 3:
            return parent1, parent2
        
        # Keep first and last waypoint fixed
        start = parent1[0]
        end = parent1[-1]
        
        # Select random segment from parent1
        if len(parent1) > 2:
            i = random.randint(1, len(parent1) - 2)
            j = random.randint(i, len(parent1) - 2)
            
            segment = parent1[i:j+1]
            
            # Create child1: segment from parent1, rest from parent2
            child1 = [start]
            for wp in parent2[1:-1]:
                if wp not in segment:
                    child1.append(wp)
            child1.extend(segment)
            for wp in parent2[1:-1]:
                if wp not in segment and wp not in child1[1:-1]:
                    child1.insert(-1, wp)
            child1.append(end)
            
            # Create child2: segment from parent2, rest from parent1
            segment2 = parent2[i:j+1] if len(parent2) > j else []
            child2 = [start]
            for wp in parent1[1:-1]:
                if wp not in segment2:
                    child2.append(wp)
            child2.extend(segment2)
            for wp in parent1[1:-1]:
                if wp not in segment2 and wp not in child2[1:-1]:
                    child2.insert(-1, wp)
            child2.append(end)
            
            return child1, child2
        
        return parent1, parent2
    
    def _mutate(self, waypoints: List[Waypoint]):
        """Mutate route by swapping two random waypoints."""
        if len(waypoints) < 3:
            return
        
        # Don't mutate first and last waypoint
        i = random.randint(1, len(waypoints) - 2)
        j = random.randint(1, len(waypoints) - 2)
        
        if i != j:
            waypoints[i], waypoints[j] = waypoints[j], waypoints[i]
    
    def _calculate_turn_angle(self, wp1: Waypoint, wp2: Waypoint, wp3: Waypoint) -> float:
        """Calculate turn angle at waypoint wp2."""
        import math
        
        # Calculate bearings
        bearing1 = self._bearing(wp1.latitude, wp1.longitude, wp2.latitude, wp2.longitude)
        bearing2 = self._bearing(wp2.latitude, wp2.longitude, wp3.latitude, wp3.longitude)
        
        # Calculate angle difference
        angle = abs(bearing2 - bearing1)
        if angle > 180:
            angle = 360 - angle
        
        return angle
    
    @staticmethod
    def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate bearing between two points."""
        import math
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)
        
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
        
        bearing = math.atan2(y, x)
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360
        
        return bearing
    
    @staticmethod
    def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance using Haversine formula."""
        import math
        
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c

