"""Streamlit UI for drone route planning."""
import sys
import tempfile
from pathlib import Path

# Додайте кореневу папку проєкту до PYTHONPATH
project_root = Path(__file__).parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

# Load .env file if it exists
try:
    from dotenv import load_dotenv
    env_path = project_root / '.env'
    if env_path.exists():
        load_dotenv(dotenv_path=env_path)
except ImportError:
    # python-dotenv not installed, skip
    pass

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
from app.persistence.db import init_db, get_db, engine, SessionLocal
from app.persistence.repositories import MissionRepository


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
if "orchestrator" not in st.session_state:
    st.session_state.orchestrator = None
if "db_connected" not in st.session_state:
    st.session_state.db_connected = None  # None = not checked, True = connected, False = not connected
if "db_error" not in st.session_state:
    st.session_state.db_error = None
if "show_close_warning" not in st.session_state:
    st.session_state.show_close_warning = False
if "mission_to_delete" not in st.session_state:
    st.session_state.mission_to_delete = None


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

# Function to check database connection
def check_database_connection():
    """Check if database is available and can be connected."""
    try:
        if SessionLocal is None:
            init_db()
        
        # Try to create a session and query
        db = next(get_db())
        # Simple query to test connection
        from sqlalchemy import text
        db.execute(text("SELECT 1"))
        db.close()
        return True, None
    except Exception as e:
        return False, str(e)

# Check database connection on app start (only if not checked yet)
if st.session_state.db_connected is None:
    st.session_state.db_connected, st.session_state.db_error = check_database_connection()

# Sidebar for mission configuration
with st.sidebar:
    st.header("Mission Configuration")
    
    # Close mission button (only show if mission is loaded)
    if st.session_state.mission:
        st.divider()
        st.subheader("Mission Management")
        
        if not st.session_state.show_close_warning:
            if st.button("❌ Close Mission", type="secondary", use_container_width=True):
                st.session_state.show_close_warning = True
                st.rerun()
        else:
            # Show warning dialog
            st.warning("⚠️ **Warning:** Closing the mission will discard all unsaved changes. Make sure to save your mission if needed.")
            col1, col2 = st.columns(2)
            with col1:
                if st.button("✅ Confirm Close", type="primary", use_container_width=True, key="confirm_close"):
                    st.session_state.mission = None
                    st.session_state.routes = {}
                    st.session_state.weather_data = None
                    st.session_state.orchestrator = None
                    st.session_state.show_close_warning = False
                    st.success("Mission closed")
                    st.rerun()
            with col2:
                if st.button("❌ Cancel", use_container_width=True, key="cancel_close"):
                    st.session_state.show_close_warning = False
                    st.rerun()
        st.divider()
    
    mission_name = st.text_input("Mission Name", value="My Mission")
    
    st.subheader("Mission Type")
    is_multi_drone = st.checkbox("Multi-Drone Mission", value=False, 
                                 help="Enable multi-drone mission planning. VRP will be used automatically for target assignment.")
    
    # Option to select number of drones to use (only for multi-drone missions)
    num_drones_to_use = None
    if is_multi_drone:
        st.subheader("Drone Selection")
        use_all_drones = st.checkbox("Use All Drones", value=True, key="use_all_drones",
                                     help="If checked, all configured drones will be used. If unchecked, select specific number.")
        if not use_all_drones:
            num_drones_to_use = st.number_input("Number of Drones to Use", 
                                               min_value=1, 
                                               max_value=10, 
                                               value=2,
                                               key="num_drones_to_use",
                                               help="Select how many drones to use for the mission")
    
    st.subheader("Drone Configuration")
    
    if is_multi_drone:
        # Multi-drone: show list of drones and allow adding more
        if "drone_list" not in st.session_state:
            st.session_state.drone_list = [{
                "name": "Drone 1",
                "max_speed": 15.0,
                "max_altitude": 120.0,
                "min_altitude": 10.0,
                "battery_capacity": 100.0,
                "power_consumption": 50.0
            }]
        
        st.write(f"**{len(st.session_state.drone_list)} drone(s) configured:**")
        for idx, drone_config in enumerate(st.session_state.drone_list):
            with st.expander(f"Drone {idx + 1}: {drone_config['name']}", expanded=idx == 0):
                drone_config["name"] = st.text_input("Drone Name", value=drone_config["name"], key=f"drone_name_{idx}")
                col1, col2 = st.columns(2)
                with col1:
                    drone_config["max_speed"] = st.number_input("Max Speed (m/s)", min_value=1.0, max_value=50.0, 
                                                               value=drone_config["max_speed"], key=f"max_speed_{idx}")
                    drone_config["max_altitude"] = st.number_input("Max Altitude (m)", min_value=10.0, max_value=500.0, 
                                                                   value=drone_config["max_altitude"], key=f"max_alt_{idx}")
                    drone_config["min_altitude"] = st.number_input("Min Altitude (m)", min_value=0.0, max_value=100.0, 
                                                                   value=drone_config["min_altitude"], key=f"min_alt_{idx}")
                with col2:
                    drone_config["battery_capacity"] = st.number_input("Battery Capacity (Wh)", min_value=10.0, max_value=500.0, 
                                                                       value=drone_config["battery_capacity"], key=f"battery_{idx}")
                    drone_config["power_consumption"] = st.number_input("Power Consumption (W)", min_value=10.0, max_value=200.0, 
                                                                        value=drone_config["power_consumption"], key=f"power_{idx}")
                
                if st.button("🗑️ Remove", key=f"remove_drone_{idx}"):
                    st.session_state.drone_list.pop(idx)
                    st.rerun()
        
        if st.button("➕ Add Another Drone"):
            st.session_state.drone_list.append({
                "name": f"Drone {len(st.session_state.drone_list) + 1}",
                "max_speed": 15.0,
                "max_altitude": 120.0,
                "min_altitude": 10.0,
                "battery_capacity": 100.0,
                "power_consumption": 50.0,
                "turn_radius": 50.0,
                "climb_rate": 5.0,
                "descent_rate": 5.0
            })
            st.rerun()
        
        # Create drone objects
        all_drones = []
        for drone_config in st.session_state.drone_list:
            all_drones.append(Drone(
                name=drone_config["name"],
                max_speed=drone_config["max_speed"],
                max_altitude=drone_config["max_altitude"],
                min_altitude=drone_config["min_altitude"],
                battery_capacity=drone_config["battery_capacity"],
                power_consumption=drone_config["power_consumption"],
                turn_radius=drone_config.get("turn_radius", 50.0),
                climb_rate=drone_config.get("climb_rate", 5.0),
                descent_rate=drone_config.get("descent_rate", 5.0)
            ))
        
        # Select drones to use based on user selection
        if num_drones_to_use is not None and num_drones_to_use < len(all_drones):
            drones = all_drones[:num_drones_to_use]
        else:
            drones = all_drones
        
        drone = drones[0] if drones else create_default_drone()
    else:
        # Single drone: simple configuration
        drone_name = st.text_input("Drone Name", value="Drone 1")
        max_speed = st.number_input("Max Speed (m/s)", min_value=1.0, max_value=50.0, value=15.0)
        max_altitude = st.number_input("Max Altitude (m)", min_value=10.0, max_value=500.0, value=120.0)
        min_altitude = st.number_input("Min Altitude (m)", min_value=0.0, max_value=100.0, value=10.0)
        battery_capacity = st.number_input("Battery Capacity (Wh)", min_value=10.0, max_value=500.0, value=100.0)
        power_consumption = st.number_input("Power Consumption (W)", min_value=10.0, max_value=200.0, value=50.0)
        
        # Dubins parameters for single drone
        turn_radius = st.number_input("Turn Radius (m)", min_value=10.0, max_value=200.0, value=50.0, key="single_turn_radius")
        climb_rate = st.number_input("Climb Rate (m/s)", min_value=1.0, max_value=20.0, value=5.0, key="single_climb_rate")
        descent_rate = st.number_input("Descent Rate (m/s)", min_value=1.0, max_value=20.0, value=5.0, key="single_descent_rate")
        
        drone = Drone(
            name=drone_name,
            max_speed=max_speed,
            max_altitude=max_altitude,
            min_altitude=min_altitude,
            battery_capacity=battery_capacity,
            power_consumption=power_consumption,
            turn_radius=turn_radius,
            climb_rate=climb_rate,
            descent_rate=descent_rate
        )
        drones = [drone]
    
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
                    drones=drones,
                    target_points=[],
                    constraints=MissionConstraints()
                )
            else:
                # Update drones list if mission already exists
                st.session_state.mission.drones = drones
            
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
                        drones=drones,
                        target_points=[],
                        constraints=MissionConstraints()
                    )
                else:
                    # Update drones list if mission already exists
                    st.session_state.mission.drones = drones
                
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
    
    # Finish point configuration
    st.subheader("Finish Point")
    if st.session_state.mission:
        finish_type = st.radio(
            "Finish Point Type",
            ["depot", "last_target", "custom"],
            index=0 if st.session_state.mission.finish_point_type == "depot" 
                 else 1 if st.session_state.mission.finish_point_type == "last_target" else 2,
            key="finish_type",
            help="Depot: Return to start point\nLast Target: Finish at last visited target\nCustom: Set custom finish point"
        )
        st.session_state.mission.finish_point_type = finish_type
        
        if finish_type == "custom":
            finish_lat = st.number_input("Finish Latitude", value=50.0, format="%.6f", key="finish_lat")
            finish_lon = st.number_input("Finish Longitude", value=30.0, format="%.6f", key="finish_lon")
            finish_alt = st.number_input("Finish Altitude (m)", value=0.0, key="finish_alt",
                                         help="Altitude for finish point. Must meet drone minimum altitude requirements.")
            
            finish = Waypoint(
                latitude=finish_lat,
                longitude=finish_lon,
                altitude=finish_alt,
                waypoint_type="finish"
            )
            st.session_state.mission.set_finish_point(finish)
        elif finish_type == "last_target":
            st.info("ℹ️ Finish point will be the last target point in the optimized route. The finish altitude will be the same as that target's altitude.")
            if st.session_state.mission.target_points:
                st.write("**Target points:**")
                for idx, target in enumerate(st.session_state.mission.target_points):
                    st.write(f"- Target {idx + 1}: {target.name or 'Unnamed'} at altitude {target.altitude:.0f}m")
        elif finish_type == "depot":
            if st.session_state.mission.depot:
                st.info(f"ℹ️ Finish point is the same as depot: {st.session_state.mission.depot.altitude:.0f}m altitude")
            else:
                st.warning("⚠️ Please set depot first")
        
        # Landing mode selection (available for all finish point types)
        if finish_type in ["depot", "last_target", "custom"]:
            st.subheader("Landing Approach")
            landing_mode = st.radio(
                "Landing Mode",
                ["vertical", "gradual"],
                index=0 if getattr(st.session_state.mission, 'landing_mode', 'vertical') == 'vertical' else 1,
                key="landing_mode",
                help="Vertical: Fly to finish at min flight altitude, then land vertically\nGradual: Descend from last target to finish (may go below min altitude)"
            )
            st.session_state.mission.landing_mode = landing_mode
            
            if landing_mode == "gradual":
                st.warning("⚠️ **Warning:** With gradual landing, the drone may fly below minimum flight altitude while descending from the last target to the finish point. This is expected behavior.")
            
            # Show additional info based on finish type
            if finish_type == "depot":
                st.info("ℹ️ For depot finish, vertical landing will fly to depot at min altitude then land. Gradual landing will descend from last target to depot altitude.")
            elif finish_type == "last_target":
                st.info("ℹ️ For last target finish, vertical landing will fly to last target at min altitude then land. Gradual landing will descend from previous target to last target altitude.")
    
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
    
    # No-Fly Zones configuration
    st.subheader("No-Fly Zones")
    if st.session_state.mission:
        if not st.session_state.mission.constraints:
            st.session_state.mission.constraints = MissionConstraints()
        
        # Check if target points are in no-fly zones
        if st.session_state.mission.constraints.no_fly_zones and st.session_state.mission.target_points:
            from shapely.geometry import Point
            
            conflicts = []
            for idx, target in enumerate(st.session_state.mission.target_points):
                point = Point(target.longitude, target.latitude)
                for zone in st.session_state.mission.constraints.no_fly_zones:
                    if zone.contains(point, target.altitude):
                        zone_name = zone.name or "unnamed"
                        conflicts.append(f"Target {idx + 1} ({target.name or 'Unnamed'}) is in no-fly zone: {zone_name}")
            
            if st.session_state.mission.depot:
                depot_point = Point(st.session_state.mission.depot.longitude, st.session_state.mission.depot.latitude)
                for zone in st.session_state.mission.constraints.no_fly_zones:
                    if zone.contains(depot_point, st.session_state.mission.depot.altitude):
                        zone_name = zone.name or "unnamed"
                        conflicts.append(f"Depot is in no-fly zone: {zone_name}")
            
            if conflicts:
                st.warning("⚠️ **Warning:** Some waypoints are in no-fly zones:\n" + "\n".join(f"- {c}" for c in conflicts))
        
        # Display existing zones
        if st.session_state.mission.constraints.no_fly_zones:
            st.write(f"**{len(st.session_state.mission.constraints.no_fly_zones)} zone(s) defined:**")
            for idx, zone in enumerate(st.session_state.mission.constraints.no_fly_zones):
                # Determine shape type
                geom_type = zone.geometry.geom_type if hasattr(zone.geometry, 'geom_type') else "Unknown"
                shape_display = {
                    "Polygon": "Polygon",
                    "MultiPolygon": "Multi-Polygon",
                    "Point": "Point",
                    "LineString": "Line"
                }.get(geom_type, geom_type)
                
                with st.expander(f"Zone {idx + 1}: {zone.name or 'Unnamed'} ({shape_display})"):
                    st.write(f"**Shape:** {shape_display}")
                    st.write(f"**Altitude:** {zone.min_altitude:.0f}m - {zone.max_altitude:.0f}m")
                    
                    # Show bounds for reference
                    if hasattr(zone.geometry, 'bounds'):
                        bounds = zone.geometry.bounds
                        st.write(f"**Bounds:** Lat {bounds[1]:.6f} to {bounds[3]:.6f}, Lon {bounds[0]:.6f} to {bounds[2]:.6f}")
                    
                    if st.button(f"Remove Zone {idx + 1}", key=f"remove_zone_{idx}"):
                        st.session_state.mission.constraints.no_fly_zones.pop(idx)
                        st.rerun()
        
        # Add new zone
        with st.expander("Add No-Fly Zone"):
            zone_name = st.text_input("Zone Name", key="zone_name", placeholder="e.g., Airport, City Center")
            
            # Shape type selection
            shape_type = st.selectbox(
                "Zone Shape",
                ["Rectangle", "Polygon", "Circle"],
                key="zone_shape_type",
                help="Rectangle: Simple rectangular zone\nPolygon: Custom polygon with multiple points\nCircle: Circular zone with center and radius"
            )
            
            geometry = None
            
            if shape_type == "Rectangle":
                st.write("**Zone Boundaries (Rectangle):**")
                col1, col2 = st.columns(2)
                with col1:
                    zone_min_lat = st.number_input("Min Latitude", value=50.0, format="%.6f", key="zone_min_lat")
                    zone_min_lon = st.number_input("Min Longitude", value=30.0, format="%.6f", key="zone_min_lon")
                with col2:
                    zone_max_lat = st.number_input("Max Latitude", value=50.1, format="%.6f", key="zone_max_lat")
                    zone_max_lon = st.number_input("Max Longitude", value=30.1, format="%.6f", key="zone_max_lon")
                
                if st.button("Create Rectangle", key="create_rect"):
                    from shapely.geometry import Polygon
                    rect_geometry = Polygon([
                        (zone_min_lon, zone_min_lat),  # Bottom-left
                        (zone_max_lon, zone_min_lat),  # Bottom-right
                        (zone_max_lon, zone_max_lat),  # Top-right
                        (zone_min_lon, zone_max_lat),  # Top-left
                        (zone_min_lon, zone_min_lat)   # Close polygon
                    ])
                    # Validate geometry
                    if not rect_geometry.is_valid:
                        from shapely.validation import make_valid
                        rect_geometry = make_valid(rect_geometry)
                    st.session_state.pending_zone_geometry = rect_geometry
                    st.success("Rectangle created!")
                
                # Use geometry from session state if available
                if "pending_zone_geometry" in st.session_state and st.session_state.pending_zone_geometry is not None:
                    geometry = st.session_state.pending_zone_geometry
                else:
                    geometry = None
            
            elif shape_type == "Polygon":
                st.write("**Polygon Points (enter coordinates, one per line):**")
                st.info("Format: latitude,longitude (e.g., 50.0,30.0). Minimum 3 points required.")
                
                polygon_points_text = st.text_area(
                    "Polygon Coordinates",
                    value="50.0,30.0\n50.01,30.0\n50.01,30.01\n50.0,30.01",
                    key="polygon_points",
                    height=150,
                    help="Enter coordinates as: lat,lon (one per line). The polygon will be automatically closed."
                )
                
                if st.button("Create Polygon", key="create_polygon"):
                    from shapely.geometry import Polygon
                    try:
                        points = []
                        lines = polygon_points_text.strip().split('\n')
                        for line in lines:
                            line = line.strip()
                            if line:
                                parts = line.split(',')
                                if len(parts) != 2:
                                    st.error(f"Invalid format in line: {line}. Use format: lat,lon")
                                    st.session_state.pending_zone_geometry = None
                                    break
                                try:
                                    lat = float(parts[0].strip())
                                    lon = float(parts[1].strip())
                                    points.append((lon, lat))  # Shapely uses (lon, lat)
                                except ValueError:
                                    st.error(f"Invalid coordinates in line: {line}")
                                    st.session_state.pending_zone_geometry = None
                                    break
                        
                        if points and len(points) >= 3:
                            # Close polygon if not already closed
                            if points[0] != points[-1]:
                                points.append(points[0])
                            poly_geometry = Polygon(points)
                            
                            # Validate geometry
                            if not poly_geometry.is_valid:
                                from shapely.validation import make_valid
                                poly_geometry = make_valid(poly_geometry)
                            
                            st.session_state.pending_zone_geometry = poly_geometry
                            st.success(f"Polygon created with {len(points)-1} points!")
                        elif points:
                            st.error("Polygon needs at least 3 points")
                            st.session_state.pending_zone_geometry = None
                    except Exception as e:
                        st.error(f"Error creating polygon: {e}")
                        st.session_state.pending_zone_geometry = None
                
                # Use geometry from session state if available
                if "pending_zone_geometry" in st.session_state and st.session_state.pending_zone_geometry is not None:
                    geometry = st.session_state.pending_zone_geometry
                else:
                    geometry = None
            
            elif shape_type == "Circle":
                st.write("**Circle Center and Radius:**")
                col1, col2, col3 = st.columns(3)
                with col1:
                    circle_center_lat = st.number_input("Center Latitude", value=50.005, format="%.6f", key="circle_center_lat")
                with col2:
                    circle_center_lon = st.number_input("Center Longitude", value=30.005, format="%.6f", key="circle_center_lon")
                with col3:
                    circle_radius = st.number_input("Radius (degrees)", value=0.005, format="%.6f", key="circle_radius", 
                                                    help="Radius in degrees. Approx: 0.001° ≈ 111m, 0.01° ≈ 1.1km")
                
                # Store geometry in session state to persist between reruns
                if "pending_zone_geometry" not in st.session_state:
                    st.session_state.pending_zone_geometry = None
                
                if st.button("Create Circle", key="create_circle"):
                    from shapely.geometry import Point
                    import math
                    
                    try:
                        # Create circle as a polygon approximation
                        center = Point(circle_center_lon, circle_center_lat)
                        # Number of segments for circle approximation
                        num_segments = 64
                        
                        # Calculate radius in degrees (approximate)
                        # At the given latitude, adjust for longitude
                        lat_rad = math.radians(circle_center_lat)
                        radius_lat = circle_radius
                        radius_lon = circle_radius / math.cos(lat_rad)
                        
                        # Generate circle points
                        points = []
                        for i in range(num_segments + 1):
                            angle = 2 * math.pi * i / num_segments
                            lon = circle_center_lon + radius_lon * math.cos(angle)
                            lat = circle_center_lat + radius_lat * math.sin(angle)
                            points.append((lon, lat))
                        
                        from shapely.geometry import Polygon
                        circle_geometry = Polygon(points)
                        
                        # Validate geometry
                        if not circle_geometry.is_valid:
                            from shapely.validation import make_valid
                            circle_geometry = make_valid(circle_geometry)
                        
                        st.session_state.pending_zone_geometry = circle_geometry
                        st.success(f"Circle created! Radius: {circle_radius:.6f}° (~{circle_radius * 111000:.0f}m)")
                    except Exception as e:
                        st.error(f"Error creating circle: {e}")
                        st.session_state.pending_zone_geometry = None
                
                # Use geometry from session state if available
                if st.session_state.pending_zone_geometry is not None:
                    geometry = st.session_state.pending_zone_geometry
                else:
                    geometry = None
            
            # Altitude range (common for all shapes)
            st.write("**Altitude Range:**")
            col3, col4 = st.columns(2)
            with col3:
                zone_min_alt = st.number_input("Min Altitude (m)", value=0.0, key="zone_min_alt")
            with col4:
                zone_max_alt = st.number_input("Max Altitude (m)", value=1000.0, key="zone_max_alt")
            
            # Add zone button
            if geometry is not None:
                if st.button("Add No-Fly Zone", key="add_zone"):
                    try:
                        # Validate geometry
                        if not geometry.is_valid:
                            from shapely.validation import make_valid
                            geometry = make_valid(geometry)
                        
                        zone = NoFlyZone(
                            geometry=geometry,
                            min_altitude=zone_min_alt,
                            max_altitude=zone_max_alt,
                            name=zone_name if zone_name else None
                        )
                        
                        st.session_state.mission.constraints.add_no_fly_zone(zone)
                        st.success(f"Added no-fly zone: {zone_name or 'Unnamed'} ({shape_type})")
                        # Clear pending geometry after adding
                        st.session_state.pending_zone_geometry = None
                        st.rerun()
                    except Exception as e:
                        st.error(f"Error adding zone: {e}")
            elif shape_type in ["Polygon", "Circle"]:
                if "pending_zone_geometry" not in st.session_state or st.session_state.pending_zone_geometry is None:
                    st.info("Click 'Create Polygon' or 'Create Circle' first to generate the geometry")
                else:
                    st.success("Geometry created! Click 'Add No-Fly Zone' to add it to the mission.")
            
            # Option to load from GeoJSON
            st.write("**Or load from GeoJSON file:**")
            uploaded_file = st.file_uploader("Upload GeoJSON", type=["geojson", "json"], key="geojson_upload")
            if uploaded_file is not None:
                try:
                    from app.data_import.geojson_loader import load_no_fly_zones_from_geojson
                    import os
                    
                    # Read file content
                    content = uploaded_file.read()
                    if isinstance(content, bytes):
                        content = content.decode('utf-8')
                    
                    with tempfile.NamedTemporaryFile(delete=False, suffix='.geojson', mode='w', encoding='utf-8') as tmp:
                        tmp.write(content)
                        tmp_path = tmp.name
                    
                    zones = load_no_fly_zones_from_geojson(tmp_path)
                    for zone in zones:
                        st.session_state.mission.constraints.add_no_fly_zone(zone)
                    
                    os.unlink(tmp_path)
                    st.success(f"Loaded {len(zones)} no-fly zone(s) from GeoJSON")
                    st.rerun()
                except Exception as e:
                    st.error(f"Error loading GeoJSON: {e}")
    
    # Planning options
    st.subheader("Planning Options")
    
    # Optimization metric selection
    optimization_metric = st.selectbox(
        "Optimization Metric",
        ["distance", "energy", "time"],
        index=0,
        help="Distance: Minimize route length\nEnergy: Minimize energy consumption\nTime: Minimize flight time"
    )
    
    # Algorithm selection
    algorithm = st.selectbox(
        "Pathfinding Algorithm",
        ["astar", "thetastar", "dstar"],
        index=0,
        help="A*: Standard pathfinding\nTheta*: Any-angle pathfinding\nD*: Dynamic replanning"
    )
    
    # Optimization options (VRP is automatic for multi-drone missions)
    if st.session_state.mission and len(st.session_state.mission.drones) > 1:
        st.info("ℹ️ Multi-drone mission: VRP will be used automatically for target assignment")
    
    st.subheader("Route Optimization")
    optimization_algorithm = st.selectbox(
        "Optimization Algorithm",
        ["None", "genetic", "aco", "pso"],
        index=0,
        help="None: No optimization\nGenetic: Genetic algorithm\nACO: Ant Colony Optimization\nPSO: Particle Swarm Optimization"
    )
    
    if st.button("Plan Route", type="primary"):
        if st.session_state.mission is None or not st.session_state.mission.target_points:
            st.error("Please add target points first")
        else:
            # Ensure mission has the latest drones list
            st.session_state.mission.drones = drones
            
            with st.spinner("Planning route..."):
                # Pass initial weather data as cache, but weather will be fetched during planning
                weather_data = st.session_state.weather_data if st.session_state.use_weather else None
                orchestrator = MissionOrchestrator(
                    st.session_state.mission, 
                    weather_data=weather_data,
                    use_weather=st.session_state.use_weather
                )
                # Get route options from mission
                landing_mode = getattr(st.session_state.mission, 'landing_mode', 'vertical')
                finish_point_type = st.session_state.mission.finish_point_type
                finish_point = st.session_state.mission.finish_point
                
                routes, error_message = orchestrator.plan_mission(
                    use_weather=st.session_state.use_weather,
                    algorithm=algorithm,
                    optimization_algorithm=optimization_algorithm if optimization_algorithm != "None" else None,
                    optimization_metric=optimization_metric,
                    landing_mode=landing_mode,
                    finish_point_type=finish_point_type,
                    finish_point=finish_point
                )
                
                if error_message:
                    st.error(error_message)
                elif routes:
                    st.session_state.routes = routes
                    st.session_state.orchestrator = orchestrator  # Store for grid visualization
                    # Update weather_data from orchestrator (includes weather fetched during planning)
                    if hasattr(orchestrator, 'weather_data') and orchestrator.weather_data:
                        st.session_state.weather_data = orchestrator.weather_data
                    if len(routes) > 1:
                        st.success(f"Routes planned successfully for {len(routes)} drones!")
                    else:
                        st.success("Route planned successfully!")
                    st.rerun()
                else:
                    st.error("No route found. Possible reasons:\n"
                           "- Target points are unreachable\n"
                           "- No-fly zones block all possible paths\n"
                           "- Constraints are too restrictive\n"
                           "- Try adjusting target points or no-fly zones")

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
    # Get weather data from orchestrator if available (includes weather fetched during planning)
    weather_data = None
    if st.session_state.use_weather:
        if hasattr(st.session_state, 'orchestrator') and st.session_state.orchestrator:
            if hasattr(st.session_state.orchestrator, 'weather_data'):
                weather_data = st.session_state.orchestrator.weather_data
        if not weather_data and st.session_state.weather_data:
            weather_data = st.session_state.weather_data
    
    # Update mission.routes with current routes from session state
    if st.session_state.routes:
        mission.routes = st.session_state.routes
    
    map_obj = renderer.render_mission(
        mission, 
        weather_data=weather_data
    )
    
    # Display map with coordinate tracking
    map_data = st_folium(map_obj, width=1200, height=600, returned_objects=["last_object_clicked", "last_clicked"])
    
    # Display coordinates info below map
    if map_data and map_data.get("last_clicked"):
        clicked_lat = map_data["last_clicked"]["lat"]
        clicked_lon = map_data["last_clicked"]["lng"]
        st.info(f"📍 **Last clicked coordinates:** Lat: {clicked_lat:.6f}, Lon: {clicked_lon:.6f}")
    else:
        st.info("📍 **Mouse coordinates:** Move your mouse over the map to see coordinates in the bottom-left corner and near the cursor. Click on the map to see coordinates here.")
    
    # Save to database section
    if st.session_state.routes and st.session_state.db_connected:
        st.divider()
        st.subheader("💾 Save to Database")
        
        col1, col2 = st.columns([3, 1])
        with col1:
            st.info(f"Save {len(st.session_state.routes)} route(s) for mission '{mission.name}' to database")
        with col2:
            if st.button("💾 Save Mission", type="primary", use_container_width=True, key="save_mission_db"):
                try:
                    db = next(get_db())
                    repo = MissionRepository(db)
                    
                    # Update mission with current routes
                    mission.routes = st.session_state.routes
                    
                    # Save or update mission
                    mission_model = repo.save_or_create(mission)
                    db.close()
                    
                    st.success(f"✅ Mission '{mission.name}' saved successfully to database!")
                    st.balloons()
                except Exception as e:
                    st.error(f"❌ Error saving mission: {str(e)}")
                    if 'db' in locals():
                        db.close()
        st.divider()
    
    # Display routes
    if st.session_state.routes:
        route_count = len(st.session_state.routes)
        st.subheader(f"Planned Routes ({route_count} drone{'s' if route_count > 1 else ''})")
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
                    
                    # Additional metrics row
                    col5, col6, col7, col8 = st.columns(4)
                    with col5:
                        st.metric("Avg Speed", f"{route.metrics.avg_speed:.1f} m/s")
                    with col6:
                        # Risk score with color coding
                        risk_value = route.metrics.risk_score
                        risk_color = "🟢" if risk_value < 0.3 else "🟡" if risk_value < 0.6 else "🔴"
                        st.metric("Risk Score", f"{risk_color} {risk_value:.2f}")
                    with col7:
                        st.metric("Max Altitude", f"{route.metrics.max_altitude:.0f} m")
                    with col8:
                        st.metric("Min Altitude", f"{route.metrics.min_altitude:.0f} m")
                
                # Validation results
                if route.validation_result:
                    # Handle both ValidationResult object and dict
                    if isinstance(route.validation_result, dict):
                        is_valid = route.validation_result.get("is_valid", True)
                        violations = route.validation_result.get("violations", [])
                        warnings = route.validation_result.get("warnings", [])
                    else:
                        # ValidationResult object
                        is_valid = route.validation_result.is_valid
                        violations = route.validation_result.violations
                        warnings = getattr(route.validation_result, 'warnings', [])
                    
                    if is_valid:
                        st.success("✓ Route is valid")
                    else:
                        st.error("✗ Route has violations")
                        for violation in violations:
                            if isinstance(violation, dict):
                                st.error(f"- {violation.get('message', 'Unknown violation')}")
                            else:
                                st.error(f"- {violation}")
                    
                    if warnings:
                        st.warning("⚠️ **Warnings:**")
                        for warning in warnings:
                            if isinstance(warning, dict):
                                st.warning(f"- {warning.get('message', 'Unknown warning')}")
                            else:
                                st.warning(f"- {warning}")
                
                # Export buttons
                col1, col2 = st.columns(2)
                with col1:
                    if st.button(f"Export .plan", key=f"plan_{drone_name}"):
                        with tempfile.NamedTemporaryFile(delete=False, suffix='.plan') as tmp:
                            # Find drone for this route
                            drone = next((d for d in mission.drones if d.name == drone_name), None)
                            PlanExporter.export_route(route, tmp.name, drone=drone, mission=mission)
                            with open(tmp.name, 'rb') as f:
                                st.download_button(
                                    "Download .plan file",
                                    f.read(),
                                    file_name=f"{mission_name}_{drone_name}.plan",
                                    mime="application/octet-stream",
                                    key=f"download_plan_{drone_name}"
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
                                    mime="application/json",
                                    key=f"download_json_{drone_name}"
                                )
        
        # Batch export for all drones (multi-drone missions)
        if st.session_state.routes and len(st.session_state.routes) > 1:
            st.subheader("Batch Export (All Drones)")
            st.info(f"Export routes for all {len(st.session_state.routes)} drones at once")
            
            col1, col2 = st.columns(2)
            with col1:
                if st.button("Export All .plan Files", key="export_all_plan"):
                    import zipfile
                    import io
                    
                    zip_buffer = io.BytesIO()
                    with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
                        for drone_name, route in st.session_state.routes.items():
                            with tempfile.NamedTemporaryFile(delete=False, suffix='.plan') as tmp:
                                # Find drone for this route
                                drone = next((d for d in mission.drones if d.name == drone_name), None)
                                PlanExporter.export_route(route, tmp.name, drone=drone, mission=mission)
                                zip_file.write(tmp.name, f"{mission_name}_{drone_name}.plan")
                    
                    zip_buffer.seek(0)
                    st.download_button(
                        "Download All .plan Files (ZIP)",
                        zip_buffer.read(),
                        file_name=f"{mission_name}_all_drones.plan.zip",
                        mime="application/zip",
                        key="download_all_plan"
                    )
            
            with col2:
                if st.button("Export All JSON Files", key="export_all_json"):
                    import zipfile
                    import io
                    
                    zip_buffer = io.BytesIO()
                    with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
                        for drone_name, route in st.session_state.routes.items():
                            with tempfile.NamedTemporaryFile(delete=False, suffix='.json') as tmp:
                                JSONExporter.export_route(route, tmp.name)
                                zip_file.write(tmp.name, f"{mission_name}_{drone_name}.json")
                    
                    zip_buffer.seek(0)
                    st.download_button(
                        "Download All JSON Files (ZIP)",
                        zip_buffer.read(),
                        file_name=f"{mission_name}_all_drones.json.zip",
                        mime="application/json",
                        key="download_all_json"
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
    # Main screen - no mission selected
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.info("👈 Configure your mission in the sidebar to get started")
    
    # Database connection status
    if st.session_state.db_connected:
        st.success("✅ Database connected")
        
        # Load saved missions
        try:
            db = next(get_db())
            repo = MissionRepository(db)
            saved_missions = repo.list_all()
            db.close()
            
            if saved_missions:
                st.subheader("📁 Saved Missions")
                st.write(f"Found {len(saved_missions)} saved mission(s):")
                
                # Display missions in a selectbox or buttons
                mission_names = [m.name for m in saved_missions]
                selected_mission_name = st.selectbox(
                    "Select a mission to open:",
                    options=[""] + mission_names,
                    format_func=lambda x: "Choose a mission..." if x == "" else x,
                    key="saved_mission_select"
                )
                
                if selected_mission_name and selected_mission_name != "":
                    if st.button("Open Mission", key="open_saved_mission"):
                        try:
                            db = next(get_db())
                            repo = MissionRepository(db)
                            mission_model = repo.get_by_name(selected_mission_name)
                            if mission_model:
                                domain_mission = repo.to_domain(mission_model)
                                st.session_state.mission = domain_mission
                                st.session_state.routes = domain_mission.routes
                                st.success(f"Mission '{selected_mission_name}' loaded successfully!")
                                st.rerun()
                            else:
                                st.error(f"Mission '{selected_mission_name}' not found in database")
                            db.close()
                        except Exception as e:
                            st.error(f"Error loading mission: {str(e)}")
                            if 'db' in locals():
                                db.close()
                
                # Show mission details in expander
                if saved_missions:
                    with st.expander("View All Saved Missions", expanded=False):
                        for mission in saved_missions:
                            col1, col2 = st.columns([3, 1])
                            with col1:
                                st.write(f"**{mission.name}**")
                                st.caption(f"Created: {mission.created_at.strftime('%Y-%m-%d %H:%M:%S') if mission.created_at else 'Unknown'}")
                                st.caption(f"Updated: {mission.updated_at.strftime('%Y-%m-%d %H:%M:%S') if mission.updated_at else 'Unknown'}")
                            with col2:
                                # Delete button for each mission
                                delete_key = f"delete_mission_{mission.id}"
                                if st.button("🗑️ Delete", key=delete_key, type="secondary", use_container_width=True):
                                    st.session_state.mission_to_delete = mission
                                    st.rerun()
                            
                            # Show warning if this mission is selected for deletion
                            if st.session_state.mission_to_delete and st.session_state.mission_to_delete.id == mission.id:
                                st.warning("⚠️ **Warning:** Are you sure you want to delete this mission? This action cannot be undone!")
                                col_confirm, col_cancel = st.columns(2)
                                with col_confirm:
                                    if st.button("✅ Confirm Delete", key=f"confirm_delete_{mission.id}", type="primary", use_container_width=True):
                                        try:
                                            db = next(get_db())
                                            repo = MissionRepository(db)
                                            repo.delete(mission.id)
                                            db.close()
                                            st.success(f"✅ Mission '{mission.name}' deleted successfully!")
                                            st.session_state.mission_to_delete = None
                                            st.rerun()
                                        except Exception as e:
                                            st.error(f"❌ Error deleting mission: {str(e)}")
                                            if 'db' in locals():
                                                db.close()
                                with col_cancel:
                                    if st.button("❌ Cancel", key=f"cancel_delete_{mission.id}", use_container_width=True):
                                        st.session_state.mission_to_delete = None
                                        st.rerun()
                            
                            st.divider()
            else:
                st.info("📭 No saved missions found. Create a new mission to get started.")
        except Exception as e:
            st.error(f"Error loading missions from database: {str(e)}")
            st.session_state.db_connected = False
            st.session_state.db_error = str(e)
    else:
        # Database not connected
        st.warning("⚠️ Database not connected")
        if st.session_state.db_error:
            st.caption(f"Error: {st.session_state.db_error}")
        st.info("💡 You can still create and plan missions, but they won't be saved to the database.")
        st.caption("To enable database features, configure DATABASE_URL environment variable or ensure PostgreSQL is running.")
    
    st.divider()
    
    # Example mission button (always available)
    st.subheader("Quick Start Example")
    if st.button("Load Example Mission", type="primary", use_container_width=True):
        example_drone = create_default_drone()
        example_mission = Mission(
            name="Example Mission",
            drones=[example_drone],
            target_points=[
                Waypoint(50.0, 30.0, 50.0, "Target 1"),
                Waypoint(50.01, 30.01, 60.0, "Target 2"),
                Waypoint(50.02, 30.0, 55.0, "Target 3"),
                Waypoint(50.03, 30.02, 65.0, "Target 4"),
                Waypoint(50.04, 29.98, 70.0, "Target 5"),
                Waypoint(50.005, 30.005, 58.0, "Target 6"),
            ],
            depot=Waypoint(49.99, 29.99, 0.0, "Depot"),
            constraints=MissionConstraints()
        )
        st.session_state.mission = example_mission
        st.success("Example mission loaded!")
        st.rerun()

