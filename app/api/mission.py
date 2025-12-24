"""Mission API endpoints."""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
from app.domain.mission import Mission
from app.domain.drone import Drone
from app.domain.waypoint import Waypoint
from app.domain.constraints import MissionConstraints

router = APIRouter(prefix="/api/missions", tags=["missions"])


class WaypointDTO(BaseModel):
    latitude: float
    longitude: float
    altitude: float
    name: Optional[str] = None
    waypoint_type: str = "target"


class DroneDTO(BaseModel):
    name: str
    max_speed: float
    max_altitude: float
    min_altitude: float
    battery_capacity: float
    power_consumption: float
    max_flight_time: Optional[float] = None
    max_range: Optional[float] = None
    turn_radius: float = 50.0
    climb_rate: float = 5.0
    descent_rate: float = 5.0


class MissionDTO(BaseModel):
    name: str
    drones: List[DroneDTO]
    target_points: List[WaypointDTO]
    depot: Optional[WaypointDTO] = None


# In-memory storage (replace with DB in production)
missions_store: dict[str, Mission] = {}


@router.post("/", response_model=dict)
async def create_mission(mission_dto: MissionDTO):
    """Create a new mission."""
    # Convert DTOs to domain objects
    drones = [Drone.from_dict(drone.dict()) for drone in mission_dto.drones]
    target_points = [Waypoint.from_dict(tp.dict()) for tp in mission_dto.target_points]
    depot = Waypoint.from_dict(mission_dto.depot.dict()) if mission_dto.depot else None
    
    mission = Mission(
        name=mission_dto.name,
        drones=drones,
        target_points=target_points,
        depot=depot
    )
    
    missions_store[mission.name] = mission
    
    return {"message": "Mission created", "mission_name": mission.name}


@router.get("/", response_model=List[str])
async def list_missions():
    """List all mission names."""
    return list(missions_store.keys())


@router.get("/{mission_name}", response_model=dict)
async def get_mission(mission_name: str):
    """Get mission by name."""
    if mission_name not in missions_store:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    return missions_store[mission_name].to_dict()


@router.delete("/{mission_name}", response_model=dict)
async def delete_mission(mission_name: str):
    """Delete a mission."""
    if mission_name not in missions_store:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    del missions_store[mission_name]
    return {"message": "Mission deleted"}

