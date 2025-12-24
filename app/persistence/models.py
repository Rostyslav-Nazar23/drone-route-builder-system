"""SQLAlchemy models for database persistence."""
from sqlalchemy import Column, Integer, String, Float, Boolean, DateTime, ForeignKey, Text, JSON
from sqlalchemy.orm import relationship
from sqlalchemy.dialects.postgresql import UUID
from geoalchemy2 import Geometry
from app.persistence.db import Base
import uuid
from datetime import datetime


class MissionModel(Base):
    """Mission database model."""
    __tablename__ = "missions"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String(255), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    drones = relationship("DroneModel", back_populates="mission", cascade="all, delete-orphan")
    target_points = relationship("TargetPointModel", back_populates="mission", cascade="all, delete-orphan")
    routes = relationship("RouteModel", back_populates="mission", cascade="all, delete-orphan")
    constraints = relationship("ConstraintsModel", back_populates="mission", uselist=False, cascade="all, delete-orphan")


class DroneModel(Base):
    """Drone database model."""
    __tablename__ = "drones"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    mission_id = Column(UUID(as_uuid=True), ForeignKey("missions.id"), nullable=False)
    name = Column(String(255), nullable=False)
    max_speed = Column(Float, nullable=False)
    max_altitude = Column(Float, nullable=False)
    min_altitude = Column(Float, nullable=False)
    battery_capacity = Column(Float, nullable=False)
    power_consumption = Column(Float, nullable=False)
    max_flight_time = Column(Float, nullable=True)
    max_range = Column(Float, nullable=True)
    turn_radius = Column(Float, default=50.0)
    climb_rate = Column(Float, default=5.0)
    descent_rate = Column(Float, default=5.0)
    
    # Relationships
    mission = relationship("MissionModel", back_populates="drones")


class TargetPointModel(Base):
    """Target point database model."""
    __tablename__ = "target_points"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    mission_id = Column(UUID(as_uuid=True), ForeignKey("missions.id"), nullable=False)
    name = Column(String(255), nullable=True)
    latitude = Column(Float, nullable=False)
    longitude = Column(Float, nullable=False)
    altitude = Column(Float, nullable=False)
    waypoint_type = Column(String(50), default="target")
    
    # Spatial index
    location = Column(Geometry('POINT', srid=4326), nullable=True)
    
    # Relationships
    mission = relationship("MissionModel", back_populates="target_points")


class RouteModel(Base):
    """Route database model."""
    __tablename__ = "routes"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    mission_id = Column(UUID(as_uuid=True), ForeignKey("missions.id"), nullable=False)
    drone_name = Column(String(255), nullable=False)
    total_distance = Column(Float, nullable=True)
    total_time = Column(Float, nullable=True)
    total_energy = Column(Float, nullable=True)
    max_altitude = Column(Float, nullable=True)
    min_altitude = Column(Float, nullable=True)
    waypoint_count = Column(Integer, nullable=True)
    validation_result = Column(JSON, nullable=True)
    
    # Relationships
    mission = relationship("MissionModel", back_populates="routes")
    waypoints = relationship("RouteWaypointModel", back_populates="route", cascade="all, delete-orphan")


class RouteWaypointModel(Base):
    """Route waypoint database model."""
    __tablename__ = "route_waypoints"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    route_id = Column(UUID(as_uuid=True), ForeignKey("routes.id"), nullable=False)
    sequence = Column(Integer, nullable=False)
    latitude = Column(Float, nullable=False)
    longitude = Column(Float, nullable=False)
    altitude = Column(Float, nullable=False)
    name = Column(String(255), nullable=True)
    
    # Spatial index
    location = Column(Geometry('POINT', srid=4326), nullable=True)
    
    # Relationships
    route = relationship("RouteModel", back_populates="waypoints")


class ConstraintsModel(Base):
    """Mission constraints database model."""
    __tablename__ = "constraints"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    mission_id = Column(UUID(as_uuid=True), ForeignKey("missions.id"), nullable=False, unique=True)
    max_altitude = Column(Float, nullable=True)
    min_altitude = Column(Float, nullable=True)
    max_distance = Column(Float, nullable=True)
    max_flight_time = Column(Float, nullable=True)
    require_return_to_depot = Column(Boolean, default=True)
    
    # Relationships
    mission = relationship("MissionModel", back_populates="constraints")
    no_fly_zones = relationship("NoFlyZoneModel", back_populates="constraints", cascade="all, delete-orphan")


class NoFlyZoneModel(Base):
    """No-fly zone database model."""
    __tablename__ = "no_fly_zones"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    constraints_id = Column(UUID(as_uuid=True), ForeignKey("constraints.id"), nullable=False)
    name = Column(String(255), nullable=True)
    min_altitude = Column(Float, default=0.0)
    max_altitude = Column(Float, default=1000.0)
    
    # Spatial geometry (Polygon or MultiPolygon)
    geometry = Column(Geometry('GEOMETRY', srid=4326), nullable=False)
    
    # Relationships
    constraints = relationship("ConstraintsModel", back_populates="no_fly_zones")

