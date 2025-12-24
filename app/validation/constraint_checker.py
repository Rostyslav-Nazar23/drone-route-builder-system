"""Main constraint checker for routes."""
from typing import List, Dict, Optional
from app.domain.route import Route
from app.domain.drone import Drone
from app.domain.constraints import MissionConstraints
from app.validation.zone_checker import ZoneChecker
from app.validation.altitude_checker import AltitudeChecker
from app.validation.energy_checker import EnergyChecker
from app.validation.kinematics_checker import KinematicsChecker


class ValidationResult:
    """Result of route validation."""
    
    def __init__(self):
        self.is_valid = True
        self.violations: List[Dict] = []
        self.warnings: List[Dict] = []
    
    def add_violation(self, type: str, message: str, waypoint_index: Optional[int] = None):
        """Add a constraint violation."""
        self.is_valid = False
        self.violations.append({
            "type": type,
            "message": message,
            "waypoint_index": waypoint_index
        })
    
    def add_warning(self, type: str, message: str, waypoint_index: Optional[int] = None):
        """Add a warning."""
        self.warnings.append({
            "type": type,
            "message": message,
            "waypoint_index": waypoint_index
        })
    
    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            "is_valid": self.is_valid,
            "violations": self.violations,
            "warnings": self.warnings
        }


class ConstraintChecker:
    """Main constraint checker."""
    
    def __init__(self):
        """Initialize constraint checker."""
        self.zone_checker = ZoneChecker()
        self.altitude_checker = AltitudeChecker()
        self.energy_checker = EnergyChecker()
        self.kinematics_checker = KinematicsChecker()
    
    def validate_route(self, route: Route, drone: Drone, 
                      constraints: Optional[MissionConstraints] = None) -> ValidationResult:
        """Validate a route against all constraints.
        
        Args:
            route: Route to validate
            drone: Drone capabilities
            constraints: Mission constraints
        
        Returns:
            ValidationResult object
        """
        result = ValidationResult()
        
        if not route.waypoints:
            result.add_violation("empty_route", "Route has no waypoints")
            return result
        
        # Check zones
        if constraints:
            zone_violations = self.zone_checker.check_route(route, constraints)
            for violation in zone_violations:
                result.add_violation("no_fly_zone", violation["message"], violation.get("waypoint_index"))
        
        # Check altitude
        altitude_violations = self.altitude_checker.check_route(route, drone, constraints)
        for violation in altitude_violations:
            result.add_violation("altitude", violation["message"], violation.get("waypoint_index"))
        
        # Check energy
        energy_result = self.energy_checker.check_route(route, drone)
        if not energy_result["is_valid"]:
            result.add_violation("energy", energy_result["message"])
        if energy_result.get("warning"):
            result.add_warning("energy", energy_result["warning"])
        
        # Check kinematics (Dubins Airplane)
        kinematics_result = self.kinematics_checker.check_route(route, drone)
        if not kinematics_result["is_valid"]:
            for violation in kinematics_result.get("violations", []):
                result.add_violation("kinematics", violation["message"], None)
        
        return result

