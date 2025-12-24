"""Export API endpoints."""
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
from pathlib import Path
import tempfile
from app.api.mission import missions_store
from app.export.plan_exporter import PlanExporter
from app.export.json_exporter import JSONExporter

router = APIRouter(prefix="/api/export", tags=["export"])


@router.get("/plan/{mission_name}/{drone_name}")
async def export_route_plan(mission_name: str, drone_name: str):
    """Export route as .plan file."""
    if mission_name not in missions_store:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    mission = missions_store[mission_name]
    route = mission.get_route(drone_name)
    
    if not route:
        raise HTTPException(status_code=404, detail="Route not found")
    
    # Create temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.plan', delete=False, encoding='utf-8') as f:
        temp_path = f.name
        PlanExporter.export_route(route, temp_path)
    
    return FileResponse(
        temp_path,
        media_type='application/octet-stream',
        filename=f"{mission_name}_{drone_name}.plan"
    )


@router.get("/json/{mission_name}")
async def export_mission_json(mission_name: str):
    """Export mission as JSON file."""
    if mission_name not in missions_store:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    mission = missions_store[mission_name]
    
    # Create temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False, encoding='utf-8') as f:
        temp_path = f.name
        JSONExporter.export_mission(mission, temp_path)
    
    return FileResponse(
        temp_path,
        media_type='application/json',
        filename=f"{mission_name}.json"
    )

