"""Simple test script to verify the system works."""
from app.domain.mission import Mission
from app.domain.drone import Drone
from app.domain.waypoint import Waypoint
from app.domain.constraints import MissionConstraints
from app.orchestrator.mission_orchestrator import MissionOrchestrator

def test_basic_planning():
    """Test basic route planning."""
    print("Testing basic route planning...")
    
    # Create a drone
    drone = Drone(
        name="Test Drone",
        max_speed=15.0,
        max_altitude=120.0,
        min_altitude=10.0,
        battery_capacity=100.0,
        power_consumption=50.0
    )
    
    # Create mission
    mission = Mission(
        name="Test Mission",
        drones=[drone],
        target_points=[
            Waypoint(50.0, 30.0, 50.0, "Target 1"),
            Waypoint(50.01, 30.01, 60.0, "Target 2"),
        ],
        depot=Waypoint(49.99, 29.99, 0.0, "Depot"),
        constraints=MissionConstraints()
    )
    
    # Plan route
    orchestrator = MissionOrchestrator(mission)
    routes = orchestrator.plan_mission(use_grid=False)  # Use waypoint graph for faster testing
    
    if routes:
        print(f"✓ Successfully planned {len(routes)} route(s)")
        for drone_name, route in routes.items():
            print(f"  - {drone_name}: {len(route.waypoints)} waypoints")
            if route.metrics:
                print(f"    Distance: {route.metrics.total_distance/1000:.2f} km")
                print(f"    Energy: {route.metrics.total_energy:.2f} Wh")
    else:
        print("✗ Failed to plan route")
    
    return routes is not None

if __name__ == "__main__":
    try:
        success = test_basic_planning()
        if success:
            print("\n✓ System test passed!")
        else:
            print("\n✗ System test failed!")
    except Exception as e:
        print(f"\n✗ Error during test: {e}")
        import traceback
        traceback.print_exc()

