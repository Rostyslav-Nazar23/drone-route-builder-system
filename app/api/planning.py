"""Planning API endpoints."""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional
from app.orchestrator.mission_orchestrator import MissionOrchestrator
from app.api.mission import missions_store

router = APIRouter(prefix="/api/planning", tags=["planning"])


class PlanningRequest(BaseModel):
    mission_name: str


@router.post("/plan", response_model=dict)
async def plan_mission(request: PlanningRequest):
    """Plan routes for a mission."""
    if request.mission_name not in missions_store:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    mission = missions_store[request.mission_name]
    orchestrator = MissionOrchestrator(mission)
    
    routes, error_message = orchestrator.plan_mission()
    
    if error_message:
        raise HTTPException(status_code=400, detail=error_message)
    
    return {
        "mission_name": request.mission_name,
        "routes": {name: route.to_dict() for name, route in routes.items()}
    }


@router.post("/replan", response_model=dict)
async def replan_route(mission_name: str, drone_name: str):
    """Replan route for a specific drone."""
    if mission_name not in missions_store:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    mission = missions_store[mission_name]
    orchestrator = MissionOrchestrator(mission)
    
    route = orchestrator.replan_route(drone_name)
    
    if not route:
        raise HTTPException(status_code=400, detail="Failed to plan route")
    
    return {
        "mission_name": mission_name,
        "drone_name": drone_name,
        "route": route.to_dict()
    }

