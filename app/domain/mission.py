"""Mission domain model."""
from dataclasses import dataclass, field
from typing import List, Optional, Dict
from datetime import datetime
from .drone import Drone
from .waypoint import Waypoint
from .route import Route
from .constraints import MissionConstraints


@dataclass
class Mission:
    """Represents a complete mission with drones, targets, and routes."""
    name: str
    drones: List[Drone] = field(default_factory=list)
    target_points: List[Waypoint] = field(default_factory=list)
    depot: Optional[Waypoint] = None
    constraints: Optional[MissionConstraints] = None
    routes: Dict[str, Route] = field(default_factory=dict)  # drone_name -> Route
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None
    
    def __post_init__(self):
        """Initialize timestamps."""
        if self.created_at is None:
            self.created_at = datetime.now()
        if self.updated_at is None:
            self.updated_at = datetime.now()
        if self.constraints is None:
            self.constraints = MissionConstraints()
    
    def add_drone(self, drone: Drone):
        """Add a drone to the mission."""
        self.drones.append(drone)
        self.updated_at = datetime.now()
    
    def add_target_point(self, waypoint: Waypoint):
        """Add a target point to the mission."""
        self.target_points.append(waypoint)
        self.updated_at = datetime.now()
    
    def set_depot(self, waypoint: Waypoint):
        """Set the depot/start point."""
        waypoint.waypoint_type = "depot"
        self.depot = waypoint
        self.updated_at = datetime.now()
    
    def add_route(self, drone_name: str, route: Route):
        """Add a route for a specific drone."""
        route.drone_name = drone_name
        self.routes[drone_name] = route
        self.updated_at = datetime.now()
    
    def get_route(self, drone_name: str) -> Optional[Route]:
        """Get route for a specific drone."""
        return self.routes.get(drone_name)
    
    def to_dict(self) -> dict:
        """Convert mission to dictionary."""
        return {
            "name": self.name,
            "drones": [drone.to_dict() for drone in self.drones],
            "target_points": [tp.to_dict() for tp in self.target_points],
            "depot": self.depot.to_dict() if self.depot else None,
            "constraints": self.constraints.to_dict() if self.constraints else None,
            "routes": {name: route.to_dict() for name, route in self.routes.items()},
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> "Mission":
        """Create mission from dictionary."""
        from .constraints import NoFlyZone
        from shapely.geometry import shape
        
        drones = [Drone.from_dict(d) for d in data.get("drones", [])]
        target_points = [Waypoint.from_dict(tp) for tp in data.get("target_points", [])]
        depot = Waypoint.from_dict(data["depot"]) if data.get("depot") else None
        
        constraints = None
        if data.get("constraints"):
            constraints_data = data["constraints"]
            constraints = MissionConstraints(
                max_altitude=constraints_data.get("max_altitude"),
                min_altitude=constraints_data.get("min_altitude"),
                max_distance=constraints_data.get("max_distance"),
                max_flight_time=constraints_data.get("max_flight_time"),
                require_return_to_depot=constraints_data.get("require_return_to_depot", True)
            )
            for zone_data in constraints_data.get("no_fly_zones", []):
                geometry = shape(zone_data["geometry"])
                zone = NoFlyZone(
                    geometry=geometry,
                    min_altitude=zone_data.get("min_altitude", 0.0),
                    max_altitude=zone_data.get("max_altitude", 1000.0),
                    name=zone_data.get("name")
                )
                constraints.add_no_fly_zone(zone)
        
        routes = {}
        for name, route_data in data.get("routes", {}).items():
            routes[name] = Route.from_dict(route_data)
        
        created_at = datetime.fromisoformat(data["created_at"]) if data.get("created_at") else None
        updated_at = datetime.fromisoformat(data["updated_at"]) if data.get("updated_at") else None
        
        mission = cls(
            name=data["name"],
            drones=drones,
            target_points=target_points,
            depot=depot,
            constraints=constraints,
            routes=routes,
            created_at=created_at,
            updated_at=updated_at
        )
        return mission

