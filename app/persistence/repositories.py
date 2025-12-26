"""Repository pattern for database operations."""
from typing import List, Optional
from sqlalchemy.orm import Session
from sqlalchemy import func
from datetime import datetime
from app.persistence.models import (
    MissionModel, DroneModel, TargetPointModel, RouteModel,
    RouteWaypointModel, ConstraintsModel, NoFlyZoneModel
)
from app.domain.mission import Mission
from app.domain.drone import Drone
from app.domain.waypoint import Waypoint
from app.domain.route import Route, RouteMetrics
from app.domain.constraints import MissionConstraints, NoFlyZone
from shapely.geometry import Point, shape
from geoalchemy2.shape import to_shape, from_shape
import uuid


class MissionRepository:
    """Repository for mission operations."""
    
    def __init__(self, db: Session):
        """Initialize repository.
        
        Args:
            db: Database session
        """
        self.db = db
    
    def create(self, mission: Mission) -> MissionModel:
        """Create mission in database.
        
        Args:
            mission: Mission domain object
        
        Returns:
            MissionModel instance
        """
        mission_model = MissionModel(
            id=uuid.uuid4(),
            name=mission.name,
            created_at=mission.created_at,
            updated_at=mission.updated_at
        )
        self.db.add(mission_model)
        
        # Add drones
        for drone in mission.drones:
            drone_model = DroneModel(
                id=uuid.uuid4(),
                mission_id=mission_model.id,
                name=drone.name,
                max_speed=drone.max_speed,
                max_altitude=drone.max_altitude,
                min_altitude=drone.min_altitude,
                battery_capacity=drone.battery_capacity,
                power_consumption=drone.power_consumption,
                max_flight_time=drone.max_flight_time,
                max_range=drone.max_range,
                turn_radius=drone.turn_radius,
                climb_rate=drone.climb_rate,
                descent_rate=drone.descent_rate
            )
            self.db.add(drone_model)
        
        # Add target points
        for idx, target in enumerate(mission.target_points):
            point = Point(target.longitude, target.latitude)
            target_model = TargetPointModel(
                id=uuid.uuid4(),
                mission_id=mission_model.id,
                name=target.name,
                latitude=target.latitude,
                longitude=target.longitude,
                altitude=target.altitude,
                waypoint_type=target.waypoint_type,
                location=from_shape(point, srid=4326)
            )
            self.db.add(target_model)
        
        # Add depot if exists
        if mission.depot:
            point = Point(mission.depot.longitude, mission.depot.latitude)
            depot_model = TargetPointModel(
                id=uuid.uuid4(),
                mission_id=mission_model.id,
                name=mission.depot.name,
                latitude=mission.depot.latitude,
                longitude=mission.depot.longitude,
                altitude=mission.depot.altitude,
                waypoint_type="depot",
                location=from_shape(point, srid=4326)
            )
            self.db.add(depot_model)
        
        # Add finish point if exists (as target point with type "finish")
        if mission.finish_point:
            point = Point(mission.finish_point.longitude, mission.finish_point.latitude)
            finish_model = TargetPointModel(
                id=uuid.uuid4(),
                mission_id=mission_model.id,
                name=mission.finish_point.name,
                latitude=mission.finish_point.latitude,
                longitude=mission.finish_point.longitude,
                altitude=mission.finish_point.altitude,
                waypoint_type="finish",
                location=from_shape(point, srid=4326)
            )
            self.db.add(finish_model)
        
        # Set finish_point_type and landing_mode
        mission_model.finish_point_type = mission.finish_point_type
        mission_model.landing_mode = mission.landing_mode
        
        # Add constraints
        if mission.constraints:
            constraints_model = ConstraintsModel(
                id=uuid.uuid4(),
                mission_id=mission_model.id,
                max_altitude=mission.constraints.max_altitude,
                min_altitude=mission.constraints.min_altitude,
                max_distance=mission.constraints.max_distance,
                max_flight_time=mission.constraints.max_flight_time,
                require_return_to_depot=mission.constraints.require_return_to_depot
            )
            self.db.add(constraints_model)
            
            # Add no-fly zones
            for zone in mission.constraints.no_fly_zones:
                zone_model = NoFlyZoneModel(
                    id=uuid.uuid4(),
                    constraints_id=constraints_model.id,
                    name=zone.name,
                    min_altitude=zone.min_altitude,
                    max_altitude=zone.max_altitude,
                    geometry=from_shape(zone.geometry, srid=4326)
                )
                self.db.add(zone_model)
        
        # Save routes if they exist
        if hasattr(mission, 'routes') and mission.routes:
            for drone_name, route in mission.routes.items():
                self.save_route(mission_model.id, route, drone_name)
        
        # Commit all changes (mission, drones, targets, constraints, routes)
        self.db.commit()
        self.db.refresh(mission_model)
        
        return mission_model
    
    def get_by_id(self, mission_id: uuid.UUID) -> Optional[MissionModel]:
        """Get mission by ID.
        
        Args:
            mission_id: Mission UUID
        
        Returns:
            MissionModel or None
        """
        return self.db.query(MissionModel).filter(MissionModel.id == mission_id).first()
    
    def get_by_name(self, name: str) -> Optional[MissionModel]:
        """Get mission by name.
        
        Args:
            name: Mission name
        
        Returns:
            MissionModel or None
        """
        return self.db.query(MissionModel).filter(MissionModel.name == name).first()
    
    def list_all(self) -> List[MissionModel]:
        """List all missions.
        
        Returns:
            List of MissionModel instances
        """
        return self.db.query(MissionModel).all()
    
    def to_domain(self, mission_model: MissionModel) -> Mission:
        """Convert database model to domain object.
        
        Args:
            mission_model: MissionModel instance
        
        Returns:
            Mission domain object
        """
        # Load relationships
        drones = [self._drone_to_domain(d) for d in mission_model.drones]
        
        # Get target points, depot, and finish point
        target_points = []
        depot = None
        finish_point = None
        for tp in mission_model.target_points:
            wp = Waypoint(
                latitude=tp.latitude,
                longitude=tp.longitude,
                altitude=tp.altitude,
                name=tp.name,
                waypoint_type=tp.waypoint_type
            )
            if tp.waypoint_type == "depot":
                depot = wp
            elif tp.waypoint_type == "finish":
                finish_point = wp
            else:
                target_points.append(wp)
        
        # Get constraints
        constraints = None
        if mission_model.constraints:
            constraints = MissionConstraints(
                max_altitude=mission_model.constraints.max_altitude,
                min_altitude=mission_model.constraints.min_altitude,
                max_distance=mission_model.constraints.max_distance,
                max_flight_time=mission_model.constraints.max_flight_time,
                require_return_to_depot=mission_model.constraints.require_return_to_depot
            )
            
            for zone_model in mission_model.constraints.no_fly_zones:
                geom = to_shape(zone_model.geometry)
                zone = NoFlyZone(
                    geometry=geom,
                    min_altitude=zone_model.min_altitude,
                    max_altitude=zone_model.max_altitude,
                    name=zone_model.name
                )
                constraints.add_no_fly_zone(zone)
        
        mission = Mission(
            name=mission_model.name,
            drones=drones,
            target_points=target_points,
            depot=depot,
            finish_point=finish_point,
            finish_point_type=getattr(mission_model, 'finish_point_type', 'depot'),
            landing_mode=getattr(mission_model, 'landing_mode', 'vertical'),
            constraints=constraints,
            created_at=mission_model.created_at,
            updated_at=mission_model.updated_at
        )
        
        # Load routes
        for route_model in mission_model.routes:
            route = self._route_to_domain(route_model)
            mission.add_route(route_model.drone_name, route)
        
        return mission
    
    def _drone_to_domain(self, drone_model: DroneModel) -> Drone:
        """Convert DroneModel to Drone domain object."""
        return Drone(
            name=drone_model.name,
            max_speed=drone_model.max_speed,
            max_altitude=drone_model.max_altitude,
            min_altitude=drone_model.min_altitude,
            battery_capacity=drone_model.battery_capacity,
            power_consumption=drone_model.power_consumption,
            max_flight_time=drone_model.max_flight_time,
            max_range=drone_model.max_range,
            turn_radius=drone_model.turn_radius,
            climb_rate=drone_model.climb_rate,
            descent_rate=drone_model.descent_rate
        )
    
    def _route_to_domain(self, route_model: RouteModel) -> Route:
        """Convert RouteModel to Route domain object."""
        waypoints = []
        for wp_model in sorted(route_model.waypoints, key=lambda x: x.sequence):
            waypoint = Waypoint(
                latitude=wp_model.latitude,
                longitude=wp_model.longitude,
                altitude=wp_model.altitude,
                name=wp_model.name,
                waypoint_type=getattr(wp_model, 'waypoint_type', 'intermediate')
            )
            waypoints.append(waypoint)
        
        route = Route(
            waypoints=waypoints,
            drone_name=route_model.drone_name,
            validation_result=route_model.validation_result
        )
        
        if route_model.total_distance is not None:
            route.metrics = RouteMetrics(
                total_distance=route_model.total_distance,
                total_time=route_model.total_time or 0.0,
                total_energy=route_model.total_energy or 0.0,
                max_altitude=route_model.max_altitude or 0.0,
                min_altitude=route_model.min_altitude or 0.0,
                waypoint_count=route_model.waypoint_count or 0,
                risk_score=getattr(route_model, 'risk_score', None) or 0.0,
                avg_speed=getattr(route_model, 'avg_speed', None) or 0.0
            )
        
        return route
    
    def save_route(self, mission_id: uuid.UUID, route: Route, drone_name: str):
        """Save route to database.
        
        Args:
            mission_id: Mission UUID
            route: Route domain object
            drone_name: Drone name
        """
        # Convert validation_result to dict if it's an object
        validation_result = route.validation_result
        if validation_result and not isinstance(validation_result, dict):
            if hasattr(validation_result, 'to_dict'):
                validation_result = validation_result.to_dict()
            elif hasattr(validation_result, '__dict__'):
                validation_result = validation_result.__dict__
        
        route_model = RouteModel(
            id=uuid.uuid4(),
            mission_id=mission_id,
            drone_name=drone_name,
            total_distance=route.metrics.total_distance if route.metrics else None,
            total_time=route.metrics.total_time if route.metrics else None,
            total_energy=route.metrics.total_energy if route.metrics else None,
            max_altitude=route.metrics.max_altitude if route.metrics else None,
            min_altitude=route.metrics.min_altitude if route.metrics else None,
            waypoint_count=route.metrics.waypoint_count if route.metrics else None,
            risk_score=route.metrics.risk_score if route.metrics else None,
            avg_speed=route.metrics.avg_speed if route.metrics else None,
            validation_result=validation_result
        )
        self.db.add(route_model)
        self.db.flush()  # Flush to get route_model.id for waypoints
        
        # Add waypoints
        for idx, waypoint in enumerate(route.waypoints):
            point = Point(waypoint.longitude, waypoint.latitude)
            wp_model = RouteWaypointModel(
                id=uuid.uuid4(),
                route_id=route_model.id,
                sequence=idx,
                latitude=waypoint.latitude,
                longitude=waypoint.longitude,
                altitude=waypoint.altitude,
                name=waypoint.name,
                waypoint_type=getattr(waypoint, 'waypoint_type', 'intermediate'),
                location=from_shape(point, srid=4326)
            )
            self.db.add(wp_model)
        
        # Note: Don't commit here - let the caller (create/update) handle commits
        # This allows multiple routes to be saved in a single transaction
    
    def update(self, mission: Mission, mission_id: Optional[uuid.UUID] = None) -> MissionModel:
        """Update existing mission in database.
        
        Args:
            mission: Mission domain object
            mission_id: Mission UUID (if None, will try to find by name)
        
        Returns:
            Updated MissionModel instance
        """
        # Find existing mission
        if mission_id:
            mission_model = self.get_by_id(mission_id)
        else:
            mission_model = self.get_by_name(mission.name)
        
        if not mission_model:
            # Mission doesn't exist, create new one
            return self.create(mission)
        
        # Update mission fields
        mission_model.name = mission.name
        mission_model.finish_point_type = mission.finish_point_type
        mission_model.landing_mode = mission.landing_mode
        mission_model.updated_at = datetime.now()
        
        # Delete existing routes and create new ones
        for route_model in mission_model.routes:
            self.db.delete(route_model)
        
        # Update or recreate drones, target points, constraints
        # For simplicity, delete and recreate (could be optimized)
        for drone_model in mission_model.drones:
            self.db.delete(drone_model)
        for target_model in mission_model.target_points:
            self.db.delete(target_model)
        
        # Recreate all data
        # Add drones
        for drone in mission.drones:
            drone_model = DroneModel(
                id=uuid.uuid4(),
                mission_id=mission_model.id,
                name=drone.name,
                max_speed=drone.max_speed,
                max_altitude=drone.max_altitude,
                min_altitude=drone.min_altitude,
                battery_capacity=drone.battery_capacity,
                power_consumption=drone.power_consumption,
                max_flight_time=drone.max_flight_time,
                max_range=drone.max_range,
                turn_radius=drone.turn_radius,
                climb_rate=drone.climb_rate,
                descent_rate=drone.descent_rate
            )
            self.db.add(drone_model)
        
        # Add target points
        for target in mission.target_points:
            point = Point(target.longitude, target.latitude)
            target_model = TargetPointModel(
                id=uuid.uuid4(),
                mission_id=mission_model.id,
                name=target.name,
                latitude=target.latitude,
                longitude=target.longitude,
                altitude=target.altitude,
                waypoint_type=target.waypoint_type,
                location=from_shape(point, srid=4326)
            )
            self.db.add(target_model)
        
        # Add depot if exists
        if mission.depot:
            point = Point(mission.depot.longitude, mission.depot.latitude)
            depot_model = TargetPointModel(
                id=uuid.uuid4(),
                mission_id=mission_model.id,
                name=mission.depot.name,
                latitude=mission.depot.latitude,
                longitude=mission.depot.longitude,
                altitude=mission.depot.altitude,
                waypoint_type="depot",
                location=from_shape(point, srid=4326)
            )
            self.db.add(depot_model)
        
        # Add finish point if exists
        if mission.finish_point:
            point = Point(mission.finish_point.longitude, mission.finish_point.latitude)
            finish_model = TargetPointModel(
                id=uuid.uuid4(),
                mission_id=mission_model.id,
                name=mission.finish_point.name,
                latitude=mission.finish_point.latitude,
                longitude=mission.finish_point.longitude,
                altitude=mission.finish_point.altitude,
                waypoint_type="finish",
                location=from_shape(point, srid=4326)
            )
            self.db.add(finish_model)
        
        # Handle constraints: update if exists, create if not
        existing_constraints = mission_model.constraints
        if mission.constraints:
            if existing_constraints:
                # Update existing constraints
                existing_constraints.max_altitude = mission.constraints.max_altitude
                existing_constraints.min_altitude = mission.constraints.min_altitude
                existing_constraints.max_distance = mission.constraints.max_distance
                existing_constraints.max_flight_time = mission.constraints.max_flight_time
                existing_constraints.require_return_to_depot = mission.constraints.require_return_to_depot
                
                # Delete old no-fly zones
                for zone_model in existing_constraints.no_fly_zones:
                    self.db.delete(zone_model)
                
                constraints_model = existing_constraints
            else:
                # Create new constraints
                constraints_model = ConstraintsModel(
                    id=uuid.uuid4(),
                    mission_id=mission_model.id,
                    max_altitude=mission.constraints.max_altitude,
                    min_altitude=mission.constraints.min_altitude,
                    max_distance=mission.constraints.max_distance,
                    max_flight_time=mission.constraints.max_flight_time,
                    require_return_to_depot=mission.constraints.require_return_to_depot
                )
                self.db.add(constraints_model)
            
            # Add no-fly zones
            for zone in mission.constraints.no_fly_zones:
                zone_model = NoFlyZoneModel(
                    id=uuid.uuid4(),
                    constraints_id=constraints_model.id,
                    name=zone.name,
                    min_altitude=zone.min_altitude,
                    max_altitude=zone.max_altitude,
                    geometry=from_shape(zone.geometry, srid=4326)
                )
                self.db.add(zone_model)
        elif existing_constraints:
            # Mission has no constraints, but database has - delete them
            for zone_model in existing_constraints.no_fly_zones:
                self.db.delete(zone_model)
            self.db.delete(existing_constraints)
        
        # Save routes if they exist
        if hasattr(mission, 'routes') and mission.routes:
            for drone_name, route in mission.routes.items():
                self.save_route(mission_model.id, route, drone_name)
        
        # Commit all changes (mission, drones, targets, constraints, routes)
        self.db.commit()
        self.db.refresh(mission_model)
        
        return mission_model
    
    def save_or_create(self, mission: Mission) -> MissionModel:
        """Save mission to database (always create new mission, even if name exists).
        
        Each mission is saved as a new record with a unique ID, allowing multiple
        missions with the same name to coexist in the database.
        
        Args:
            mission: Mission domain object
        
        Returns:
            MissionModel instance (newly created)
        """
        # Always create a new mission, even if one with the same name exists
        # This allows multiple missions with the same name but different IDs
        return self.create(mission)
    
    def delete(self, mission_id: uuid.UUID):
        """Delete mission.
        
        Args:
            mission_id: Mission UUID
        """
        mission = self.get_by_id(mission_id)
        if mission:
            self.db.delete(mission)
            self.db.commit()

