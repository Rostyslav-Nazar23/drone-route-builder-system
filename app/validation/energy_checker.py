"""Energy consumption checker."""
from typing import Dict
from app.domain.route import Route
from app.domain.drone import Drone


class EnergyChecker:
    """Checks energy consumption constraints."""
    
    def check_route(self, route: Route, drone: Drone) -> Dict:
        """Check if route is feasible given drone energy capacity.
        
        Args:
            route: Route to check
            drone: Drone capabilities
        
        Returns:
            Dictionary with validation result
        """
        if not route.waypoints:
            return {"is_valid": False, "message": "Route has no waypoints"}
        
        # Calculate metrics if not already calculated
        if route.metrics is None:
            route.calculate_metrics(drone)
        
        total_energy = route.metrics.total_energy
        battery_capacity = drone.battery_capacity
        
        result = {"is_valid": True}
        
        if total_energy > battery_capacity:
            result["is_valid"] = False
            result["message"] = f"Route requires {total_energy:.2f}Wh, exceeds battery capacity {battery_capacity:.2f}Wh"
        elif total_energy > battery_capacity * 0.9:
            result["warning"] = f"Route uses {total_energy:.2f}Wh ({total_energy/battery_capacity*100:.1f}%), close to battery limit"
        else:
            result["message"] = f"Route uses {total_energy:.2f}Wh ({total_energy/battery_capacity*100:.1f}% of capacity)"
        
        return result

