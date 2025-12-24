"""Streamlit UI for drone route planning."""
import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from app.domain.mission import Mission
from app.domain.drone import Drone
from app.domain.waypoint import Waypoint
from app.domain.constraints import MissionConstraints, NoFlyZone
from app.orchestrator.mission_orchestrator import MissionOrchestrator
from app.visualization.map_renderer import MapRenderer
from app.data_import.importer import DataImporter
from app.export.plan_exporter import PlanExporter
from app.export.json_exporter import JSONExporter
from app.weather.weather_provider import WeatherProvider, WeatherConditions
from shapely.geometry import shape
from datetime import datetime


st.set_page_config(page_title="Drone Route Builder", layout="wide")

# Initialize session state
if "mission" not in st.session_state:
    st.session_state.mission = None
if "routes" not in st.session_state:
    st.session_state.routes = {}
if "weather_data" not in st.session_state:
    st.session_state.weather_data = None
if "use_weather" not in st.session_state:
    st.session_state.use_weather = False


def create_default_drone() -> Drone:
    """Create a default drone."""
    return Drone(
        name="Default Drone",
        max_speed=15.0,  # m/s
        max_altitude=120.0,  # meters
        min_altitude=10.0,
        battery_capacity=100.0,  # Wh
        power_consumption=50.0  # W
    )


st.title("🚁 Drone Route Builder System")

# Sidebar for mission configuration
with st.sidebar:
    st.header("Mission Configuration")
    
    mission_name = st.text_input("Mission Name", value="My Mission")
    
    st.subheader("Drone Configuration")
    drone_name = st.text_input("Drone Name", value="Drone 1")
    max_speed = st.number_input("Max Speed (m/s)", min_value=1.0, max_value=50.0, value=15.0)
    max_altitude = st.number_input("Max Altitude (m)", min_value=10.0, max_value=500.0, value=120.0)
    min_altitude = st.number_input("Min Altitude (m)", min_value=0.0, max_value=100.0, value=10.0)
    battery_capacity = st.number_input("Battery Capacity (Wh)", min_value=10.0, max_value=500.0, value=100.0)
    power_consumption = st.number_input("Power Consumption (W)", min_value=10.0, max_value=200.0, value=50.0)
    
    drone = Drone(
        name=drone_name,
        max_speed=max_speed,
        max_altitude=max_altitude,
        min_altitude=min_altitude,
        battery_capacity=battery_capacity,
        power_consumption=power_consumption
    )
    
    st.subheader("Target Points")
    st.write("Add target points manually or import from file")
    
    # Manual waypoint input
    with st.expander("Add Waypoint Manually"):
        wp_lat = st.number_input("Latitude", value=50.0, format="%.6f")
        wp_lon = st.number_input("Longitude", value=30.0, format="%.6f")
        wp_alt = st.number_input("Altitude (m)", value=50.0)
        wp_name = st.text_input("Name (optional)")
        
        if st.button("Add Waypoint"):
            if st.session_state.mission is None:
                st.session_state.mission = Mission(
                    name=mission_name,
                    drones=[drone],
                    target_points=[],
                    constraints=MissionConstraints()
                )
            
            waypoint = Waypoint(
                latitude=wp_lat,
                longitude=wp_lon,
                altitude=wp_alt,
                name=wp_name if wp_name else None
            )
            st.session_state.mission.add_target_point(waypoint)
            st.success(f"Added waypoint: {wp_name or 'Unnamed'}")
            st.rerun()
    
    # File import
    with st.expander("Import from File"):
        uploaded_file = st.file_uploader("Upload CSV or GeoJSON", type=["csv", "geojson", "json"])
        if uploaded_file:
            try:
                # Save to temp file
                import tempfile
                with tempfile.NamedTemporaryFile(delete=False, suffix=uploaded_file.name) as tmp:
                    tmp.write(uploaded_file.read())
                    tmp_path = tmp.name
                
                importer = DataImporter()
                waypoints = importer.import_waypoints(tmp_path)
                
                if st.session_state.mission is None:
                    st.session_state.mission = Mission(
                        name=mission_name,
                        drones=[drone],
                        target_points=[],
                        constraints=MissionConstraints()
                    )
                
                for wp in waypoints:
                    st.session_state.mission.add_target_point(wp)
                
                st.success(f"Imported {len(waypoints)} waypoints")
                st.rerun()
            except Exception as e:
                st.error(f"Import error: {str(e)}")
    
    # Depot configuration
    st.subheader("Depot (Start Point)")
    use_depot = st.checkbox("Use Depot", value=True)
    if use_depot:
        depot_lat = st.number_input("Depot Latitude", value=50.0, format="%.6f", key="depot_lat")
        depot_lon = st.number_input("Depot Longitude", value=30.0, format="%.6f", key="depot_lon")
        depot_alt = st.number_input("Depot Altitude (m)", value=0.0, key="depot_alt")
        
        if st.session_state.mission:
            depot = Waypoint(
                latitude=depot_lat,
                longitude=depot_lon,
                altitude=depot_alt,
                waypoint_type="depot"
            )
            st.session_state.mission.set_depot(depot)
    
    # Weather configuration
    st.subheader("Weather Data")
    use_weather = st.checkbox("Use Weather Data", value=False, key="use_weather_checkbox")
    st.session_state.use_weather = use_weather
    
    if use_weather:
        st.info("🌤️ Weather data will be fetched from Open Meteo API to optimize route planning")
        
        # Weather time selection
        weather_time = st.datetime_input(
            "Weather Forecast Time",
            value=datetime.now(),
            help="Select the time for weather forecast"
        )
        
        if st.button("Fetch Weather Data", key="fetch_weather"):
            if st.session_state.mission and (st.session_state.mission.target_points or st.session_state.mission.depot):
                with st.spinner("Fetching weather data from Open Meteo..."):
                    try:
                        weather_provider = WeatherProvider()
                        weather_data = {}
                        
                        # Get weather for depot
                        if st.session_state.mission.depot:
                            weather = weather_provider.get_weather(
                                st.session_state.mission.depot.latitude,
                                st.session_state.mission.depot.longitude,
                                st.session_state.mission.depot.altitude,
                                weather_time
                            )
                            if weather:
                                key = (st.session_state.mission.depot.latitude, st.session_state.mission.depot.longitude)
                                weather_data[key] = weather
                        
                        # Get weather for target points
                        for target in st.session_state.mission.target_points:
                            key = (target.latitude, target.longitude)
                            if key not in weather_data:
                                weather = weather_provider.get_weather(
                                    target.latitude,
                                    target.longitude,
                                    target.altitude,
                                    weather_time
                                )
                                if weather:
                                    weather_data[key] = weather
                        
                        st.session_state.weather_data = weather_data
                        st.success(f"Fetched weather data for {len(weather_data)} locations")
                        
                        # Display weather summary
                        if weather_data:
                            with st.expander("Weather Summary"):
                                for (lat, lon), weather in list(weather_data.items())[:5]:  # Show first 5
                                    st.write(f"**Location ({lat:.4f}, {lon:.4f})**")
                                    st.write(f"- Wind: {weather.wind_speed_10m:.1f} m/s @ {weather.wind_direction_10m:.0f}°")
                                    st.write(f"- Temperature: {weather.temperature_2m:.1f}°C")
                                    st.write(f"- Precipitation: {weather.precipitation:.1f} mm")
                                    st.write(f"- Cloud Cover: {weather.cloud_cover:.0f}%")
                    except Exception as e:
                        st.error(f"Error fetching weather data: {str(e)}")
                        st.session_state.weather_data = None
            else:
                st.warning("Please add target points or depot first")
    
    # Planning options
    st.subheader("Planning Options")
    use_grid = st.checkbox("Use Grid Graph", value=True)
    
    # Algorithm selection
    algorithm = st.selectbox(
        "Pathfinding Algorithm",
        ["astar", "thetastar", "dstar"],
        index=0,
        help="A*: Standard pathfinding\nTheta*: Any-angle pathfinding\nD*: Dynamic replanning"
    )
    
    # Optimization options
    use_vrp = st.checkbox("Use VRP for Multi-Drone", value=True, 
                         help="Use Vehicle Routing Problem solver for optimal target assignment")
    use_genetic = st.checkbox("Use Genetic Algorithm Optimization", value=False,
                              help="Optimize routes using genetic algorithm (slower but better results)")
    
    if st.button("Plan Route", type="primary"):
        if st.session_state.mission is None or not st.session_state.mission.target_points:
            st.error("Please add target points first")
        else:
            with st.spinner("Planning route..."):
                weather_data = st.session_state.weather_data if st.session_state.use_weather else None
                orchestrator = MissionOrchestrator(st.session_state.mission, weather_data)
                routes = orchestrator.plan_mission(
                    use_grid=use_grid, 
                    use_weather=st.session_state.use_weather,
                    algorithm=algorithm,
                    use_vrp=use_vrp,
                    use_genetic=use_genetic
                )
                st.session_state.routes = routes
                st.success("Route planned successfully!")
                st.rerun()

# Main area
if st.session_state.mission:
    mission = st.session_state.mission
    
    # Display mission info
    col1, col2, col3 = st.columns(3)
    with col1:
        st.metric("Target Points", len(mission.target_points))
    with col2:
        st.metric("Drones", len(mission.drones))
    with col3:
        st.metric("Routes", len(mission.routes))
    
    # Map visualization
    st.subheader("Mission Map")
    renderer = MapRenderer()
    weather_data = st.session_state.weather_data if st.session_state.use_weather else None
    map_obj = renderer.render_mission(mission, weather_data=weather_data)
    
    # Display map
    map_data = st_folium(map_obj, width=1200, height=600)
    
    # Display routes
    if st.session_state.routes:
        st.subheader("Planned Routes")
        for drone_name, route in st.session_state.routes.items():
            with st.expander(f"Route for {drone_name}"):
                if route.metrics:
                    col1, col2, col3, col4 = st.columns(4)
                    with col1:
                        st.metric("Distance", f"{route.metrics.total_distance/1000:.2f} km")
                    with col2:
                        st.metric("Time", f"{route.metrics.total_time/60:.1f} min")
                    with col3:
                        st.metric("Energy", f"{route.metrics.total_energy:.2f} Wh")
                    with col4:
                        st.metric("Waypoints", route.metrics.waypoint_count)
                
                # Validation results
                if route.validation_result:
                    if route.validation_result.get("is_valid"):
                        st.success("✓ Route is valid")
                    else:
                        st.error("✗ Route has violations")
                        for violation in route.validation_result.get("violations", []):
                            st.error(f"- {violation['message']}")
                
                # Export buttons
                col1, col2 = st.columns(2)
                with col1:
                    if st.button(f"Export .plan", key=f"plan_{drone_name}"):
                        with tempfile.NamedTemporaryFile(delete=False, suffix='.plan') as tmp:
                            PlanExporter.export_route(route, tmp.name)
                            with open(tmp.name, 'rb') as f:
                                st.download_button(
                                    "Download .plan file",
                                    f.read(),
                                    file_name=f"{mission_name}_{drone_name}.plan",
                                    mime="application/octet-stream"
                                )
                
                with col2:
                    if st.button(f"Export JSON", key=f"json_{drone_name}"):
                        with tempfile.NamedTemporaryFile(delete=False, suffix='.json') as tmp:
                            JSONExporter.export_route(route, tmp.name)
                            with open(tmp.name, 'rb') as f:
                                st.download_button(
                                    "Download JSON file",
                                    f.read(),
                                    file_name=f"{mission_name}_{drone_name}.json",
                                    mime="application/json"
                                )
    
    # Waypoint list
    if mission.target_points:
        st.subheader("Target Points")
        waypoint_data = []
        for idx, wp in enumerate(mission.target_points):
            waypoint_data.append({
                "Index": idx + 1,
                "Name": wp.name or f"Target {idx+1}",
                "Latitude": f"{wp.latitude:.6f}",
                "Longitude": f"{wp.longitude:.6f}",
                "Altitude (m)": f"{wp.altitude:.1f}"
            })
        st.dataframe(waypoint_data, use_container_width=True)
        
        if st.button("Clear All Waypoints"):
            st.session_state.mission.target_points = []
            st.session_state.routes = {}
            st.rerun()

else:
    st.info("👈 Configure your mission in the sidebar to get started")
    
    # Example mission
    st.subheader("Quick Start Example")
    if st.button("Load Example Mission"):
        example_drone = create_default_drone()
        example_mission = Mission(
            name="Example Mission",
            drones=[example_drone],
            target_points=[
                Waypoint(50.0, 30.0, 50.0, "Target 1"),
                Waypoint(50.01, 30.01, 60.0, "Target 2"),
                Waypoint(50.02, 30.0, 55.0, "Target 3"),
            ],
            depot=Waypoint(49.99, 29.99, 0.0, "Depot"),
            constraints=MissionConstraints()
        )
        st.session_state.mission = example_mission
        st.success("Example mission loaded!")
        st.rerun()

