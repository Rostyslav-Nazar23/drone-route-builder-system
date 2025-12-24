"""Visualization API endpoints."""
from fastapi import APIRouter, HTTPException
from fastapi.responses import HTMLResponse
from app.api.mission import missions_store
from app.visualization.map_renderer import MapRenderer

router = APIRouter(prefix="/api/visualization", tags=["visualization"])


@router.get("/mission/{mission_name}", response_class=HTMLResponse)
async def visualize_mission(mission_name: str):
    """Get HTML map visualization for a mission."""
    if mission_name not in missions_store:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    mission = missions_store[mission_name]
    renderer = MapRenderer()
    map_obj = renderer.render_mission(mission)
    
    return map_obj._repr_html_()


@router.get("/route/{mission_name}/{drone_name}", response_class=HTMLResponse)
async def visualize_route(mission_name: str, drone_name: str):
    """Get HTML map visualization for a specific route."""
    if mission_name not in missions_store:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    mission = missions_store[mission_name]
    route = mission.get_route(drone_name)
    
    if not route:
        raise HTTPException(status_code=404, detail="Route not found")
    
    renderer = MapRenderer()
    map_obj = renderer.render_route(route)
    
    return map_obj._repr_html_()

