from flask import Flask, request, jsonify, send_file
import os
import sys
import io
sys.stdout.reconfigure(line_buffering=True)
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon, LineString
from pymavlink import mavutil
import time
import threading
import serial.tools.list_ports
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024
app.config['UPLOAD_FOLDER'] = 'uploads'

os.makedirs(app.config['UPLOAD_FOLDER'], exist_ok=True)

mav_connection = None
connection_lock = threading.Lock()
mission_monitor_thread = None
mission_monitoring = False
last_waypoint = -1
last_altitude = -1
mission_start_time = None
mission_altitude = 10.0
mission_speed = 5.0

gps_status = {
    'fix_type': 'UNKNOWN',
    'satellites': 0,
    'hdop': 9999,
    'lat': 0,
    'lon': 0,
    'alt': 0,
    'ekf_ok': False,
    'home_locked': False,
    'home_lat': None,
    'home_lon': None,
    'home_alt': None,
    'home_status': 'NOT_SET'
}

def get_available_ports():
    ports = []
    for port in serial.tools.list_ports.comports():
        ports.append({
            'device': port.device,
            'description': port.description,
            'hwid': port.hwid
        })
    return ports

def connect_pixhawk(connection_string='/dev/ttyACM0', baud=115200):
    global mav_connection
    try:
        with connection_lock:
            if mav_connection:
                try:
                    print("[CONNECTION] Closing existing connection...")
                    mav_connection.close()
                except Exception:
                    pass
            
            print(f"\n{'='*60}")
            print(f"[CONNECTION] Connecting to ArduPilot")
            print(f"[CONNECTION] Port: {connection_string}")
            print(f"[CONNECTION] Baud: {baud}")
            print(f"{'='*60}")
            
            mav_connection = mavutil.mavlink_connection(
                connection_string, 
                baud=baud,
                source_system=255,
                source_component=0,
                autoreconnect=True
            )
            
            print("[CONNECTION] Waiting for heartbeat...")
            mav_connection.wait_heartbeat(timeout=10)
            print(f"[CONNECTION] ✓ Heartbeat received!")
            print(f"[CONNECTION]   System ID: {mav_connection.target_system}")
            print(f"[CONNECTION]   Component: {mav_connection.target_component}")
            
            count = 0
            while mav_connection.recv_match(blocking=False):
                count += 1
            print(f"[CONNECTION] Flushed {count} old messages")
            
            start_gps_monitoring()
            
            print(f"[CONNECTION] ✓✓ CONNECTION SUCCESSFUL ✓✓\n")
            
            return True, "Connected to ArduPilot!"
            
    except PermissionError as e:
        error_msg = f"Permission denied: {connection_string}. Close Mission Planner/QGC."
        print(f"[CONNECTION] ✗ ERROR: {error_msg}")
        mav_connection = None
        return False, error_msg
    except Exception as e:
        error_msg = str(e)
        print(f"[CONNECTION] ✗ ERROR: {error_msg}")
        mav_connection = None
        return False, error_msg


def disconnect_pixhawk():
    global mav_connection
    try:
        with connection_lock:
            if mav_connection:
                print("[CONNECTION] Disconnecting...")
                stop_mission_monitoring()
                mav_connection.close()
                mav_connection = None
                print("[CONNECTION] Disconnected")
                return True, "Disconnected"
            return False, "Not connected"
    except Exception as e:
        mav_connection = None
        return False, str(e)


def gps_monitor():
    global mav_connection, gps_status
    
    print("[GPS_MONITOR] 📡 GPS monitoring started")
    
    while mav_connection:
        try:
            gps_msg = mav_connection.recv_match(type='GPS_RAW_INT', blocking=False, timeout=0.1)
            if gps_msg:
                fix_types = ['NO_GPS', 'NO_FIX', '2D_FIX', '3D_FIX', 'DGPS', 'RTK_FLOAT', 'RTK_FIXED']
                gps_status['fix_type'] = fix_types[gps_msg.fix_type] if gps_msg.fix_type < len(fix_types) else 'UNKNOWN'
                gps_status['satellites'] = gps_msg.satellites_visible
                gps_status['hdop'] = gps_msg.eph / 100.0 if gps_msg.eph < 65535 else 9999
                gps_status['lat'] = gps_msg.lat / 1e7
                gps_status['lon'] = gps_msg.lon / 1e7
                gps_status['alt'] = gps_msg.alt / 1000.0
            
            ekf_msg = mav_connection.recv_match(type='EKF_STATUS_REPORT', blocking=False, timeout=0.1)
            if ekf_msg:
                gps_status['ekf_ok'] = (ekf_msg.flags & 0x0C) == 0x0C
            
            home_msg = mav_connection.recv_match(type='HOME_POSITION', blocking=False, timeout=0.1)
            if home_msg:
                gps_status['home_lat'] = home_msg.latitude / 1e7
                gps_status['home_lon'] = home_msg.longitude / 1e7
                gps_status['home_alt'] = home_msg.altitude / 1000.0
                if gps_status['home_status'] == 'NOT_SET':
                    gps_status['home_status'] = 'ESTIMATED'
            
            time.sleep(1)
            
        except Exception as e:
            time.sleep(1)
    
    print("[GPS_MONITOR] 📡 GPS monitoring stopped")

def start_gps_monitoring():
    gps_thread = threading.Thread(target=gps_monitor, daemon=True)
    gps_thread.start()
    print("[GPS_MONITOR] ✓ GPS monitoring thread started")

def lock_home_position():
    global mav_connection, gps_status
    
    if not mav_connection:
        return False, "Not connected"
    
    try:
        print(f"\n{'='*60}")
        print(f"[HOME_LOCK] Evaluating GPS for HOME lock")
        print(f"{'='*60}")
        
        print(f"[HOME_LOCK] GPS Status:")
        print(f"[HOME_LOCK]   Fix Type: {gps_status['fix_type']}")
        print(f"[HOME_LOCK]   Satellites: {gps_status['satellites']}")
        print(f"[HOME_LOCK]   HDOP: {gps_status['hdop']:.2f}")
        print(f"[HOME_LOCK]   EKF OK: {gps_status['ekf_ok']}")
        print(f"[HOME_LOCK]   Position: {gps_status['lat']:.6f}, {gps_status['lon']:.6f}")
        
        has_3d_fix = gps_status['fix_type'] in ['3D_FIX', 'DGPS', 'RTK_FLOAT', 'RTK_FIXED']
        has_enough_sats = gps_status['satellites'] >= 6
        has_good_hdop = gps_status['hdop'] < 2.0
        ekf_ok = gps_status['ekf_ok']
        
        if has_3d_fix and has_enough_sats and has_good_hdop and ekf_ok:
            print(f"[HOME_LOCK] ✓ GPS quality sufficient for HOME lock")
            
            print(f"[HOME_LOCK] Setting HOME to current position...")
            
            mav_connection.mav.command_long_send(
                mav_connection.target_system,
                mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            
            time.sleep(0.5)
            
            mav_connection.mav.command_long_send(
                mav_connection.target_system,
                mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
                0, 0, 0, 0, 0, 0
            )
            
            time.sleep(0.5)
            
            gps_status['home_locked'] = True
            gps_status['home_status'] = 'LOCKED'
            gps_status['home_lat'] = gps_status['lat']
            gps_status['home_lon'] = gps_status['lon']
            gps_status['home_alt'] = gps_status['alt']
            
            print(f"[HOME_LOCK] ✓✓ HOME LOCKED to {gps_status['home_lat']:.6f}, {gps_status['home_lon']:.6f}")
            print(f"[HOME_LOCK] RTL will return to this exact position")
            print(f"{'='*60}\n")
            
            return True, f"✓ HOME locked at {gps_status['satellites']} sats, HDOP {gps_status['hdop']:.2f}"
            
        else:
            print(f"[HOME_LOCK] ⚠️ GPS not sufficient for HOME lock")
            print(f"[HOME_LOCK]   3D Fix: {has_3d_fix}")
            print(f"[HOME_LOCK]   Satellites ≥6: {has_enough_sats}")
            print(f"[HOME_LOCK]   HDOP <2.0: {has_good_hdop}")
            print(f"[HOME_LOCK]   EKF OK: {ekf_ok}")
            print(f"[HOME_LOCK] Using ArduPilot's estimated HOME (may drift)")
            print(f"[HOME_LOCK] RTL accuracy may vary - monitor drone closely")
            print(f"{'='*60}\n")
            
            gps_status['home_locked'] = False
            gps_status['home_status'] = 'ESTIMATED'
            
            return False, f"⚠️ GPS insufficient - HOME estimated (Sats: {gps_status['satellites']}, HDOP: {gps_status['hdop']:.2f})"
            
    except Exception as e:
        print(f"[HOME_LOCK] ✗ ERROR: {e}")
        return False, f"Error: {str(e)}"

def upload_mission_to_pixhawk(waypoints, altitude=10.0, speed=5.0):
    global mav_connection, mission_altitude, mission_speed
    if not mav_connection:
        print("[MISSION] ✗ Not connected to Pixhawk")
        return False, "Not connected"
    
    try:
        mission_altitude = altitude
        mission_speed = speed
        
        acceptance_radius = max(5.0, speed * 1.0)
        
        print(f"\n{'='*60}")
        print(f"[MISSION] LARGE DRONE MISSION UPLOAD")
        print(f"[MISSION] Waypoints: {len(waypoints)}")
        print(f"[MISSION] Altitude: {altitude}m AGL")
        print(f"[MISSION] Speed: {speed}m/s")
        print(f"[MISSION] Acceptance Radius: {acceptance_radius}m")
        print(f"{'='*60}")
        
        print("[MISSION] Step 1: Clearing existing mission...")
        clear_attempts = 0
        clear_success = False
        
        while clear_attempts < 3 and not clear_success:
            clear_attempts += 1
            mav_connection.mav.mission_clear_all_send(
                mav_connection.target_system,
                mav_connection.target_component
            )
            
            ack = mav_connection.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
            if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                clear_success = True
                print(f"[MISSION] ✓ Mission cleared (attempt {clear_attempts})")
            else:
                print(f"[MISSION] Retry clear (attempt {clear_attempts})")
                time.sleep(0.5)
        
        if not clear_success:
            print("[MISSION] ✗ Failed to clear mission")
            return False, "Failed to clear mission"
        
        mission_items = []
        
        mission_items.append({
            'seq': 0,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0,
            'autocontinue': 1,
            'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
            'x': 0, 'y': 0, 'z': 0
        })
        
        mission_items.append({
            'seq': 1,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            'current': 0,
            'autocontinue': 1,
            'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
            'x': 0, 'y': 0, 'z': altitude
        })
        
        mission_items.append({
            'seq': 2,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            'current': 0,
            'autocontinue': 1,
            'param1': 1, 'param2': speed, 'param3': -1, 'param4': 0,
            'x': 0, 'y': 0, 'z': 0
        })
        
        seq = 3
        for idx, (lon, lat) in enumerate(waypoints):
            mission_items.append({
                'seq': seq,
                'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                'current': 0,
                'autocontinue': 1,
                'param1': 0.0,
                'param2': acceptance_radius,
                'param3': 0,
                'param4': 0,
                'x': lat, 'y': lon, 'z': altitude
            })
            seq += 1
        
        mission_items.append({
            'seq': seq,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            'current': 0,
            'autocontinue': 1,
            'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
            'x': 0, 'y': 0, 'z': 0
        })
        
        print(f"[MISSION] Total mission items: {len(mission_items)}")
        
        print(f"\n[MISSION] Step 2: Sending mission count...")
        first_request = None
        for count_attempt in range(3):
            mav_connection.mav.mission_count_send(
                mav_connection.target_system,
                mav_connection.target_component,
                len(mission_items),
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            first_request = mav_connection.recv_match(type='MISSION_REQUEST', blocking=True, timeout=3)
            if first_request:
                print(f"[MISSION] ✓ Mission count accepted")
                break
            time.sleep(0.3)
        
        if not first_request:
            return False, "Failed to send mission count"
        
        print("[MISSION] Step 3: Uploading mission items...")
        pending_request = first_request
        uploaded_count = 0
        
        while uploaded_count < len(mission_items):
            if pending_request and pending_request.seq < len(mission_items):
                item = mission_items[pending_request.seq]
                mav_connection.mav.mission_item_send(
                    mav_connection.target_system,
                    mav_connection.target_component,
                    item['seq'], item['frame'], item['command'],
                    item['current'], item['autocontinue'],
                    item['param1'], item['param2'], item['param3'], item['param4'],
                    item['x'], item['y'], item['z']
                )
                uploaded_count += 1
                if uploaded_count % 10 == 0 or uploaded_count == len(mission_items):
                    print(f"[MISSION]   Uploaded {uploaded_count}/{len(mission_items)}...")
            
            msg = mav_connection.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'], blocking=True, timeout=5)
            if not msg:
                return False, f"Timeout at item {uploaded_count}/{len(mission_items)}"
            if msg.get_type() == 'MISSION_ACK':
                if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    return False, "Mission rejected"
                break
            pending_request = msg
        
        print(f"\n{'='*60}")
        print(f"[MISSION] ✓✓ MISSION UPLOAD SUCCESSFUL ✓✓")
        print(f"[MISSION] Survey waypoints: {len(waypoints)}")
        print(f"[MISSION] Flight altitude: {altitude}m AGL")
        print(f"[MISSION] Flight speed: {speed}m/s")
        print(f"[MISSION] Waypoint radius: {acceptance_radius}m")
        print(f"{'='*60}\n")
        
        return True, f"✓ Uploaded {len(waypoints)} waypoints"
        
    except Exception as e:
        print(f"[MISSION] ✗ ERROR: {e}")
        return False, str(e)

def parse_kml_polygon(kml_content):
    try:
        root = ET.fromstring(kml_content)
        coordinates_elements = root.findall('.//{http://www.opengis.net/kml/2.2}coordinates')
        
        if not coordinates_elements:
            return None
            
        coords_text = coordinates_elements[0].text.strip()
        coords_list = []
        coords_parts = coords_text.replace('\n', ' ').split()
        
        for coord_part in coords_parts:
            coord_part = coord_part.strip()
            if coord_part:
                parts = coord_part.split(',')
                if len(parts) >= 2:
                    coords_list.append([float(parts[0]), float(parts[1])])
        
        return coords_list if len(coords_list) >= 3 else None
        
    except Exception as e:
        print(f"KML parse error: {e}")
        return None

def generate_lawnmower_path(polygon_coords, spacing_meters=20, boundary_margin=10):
    if not polygon_coords or len(polygon_coords) < 3:
        return None
    
    try:
        if polygon_coords[0] != polygon_coords[-1]:
            polygon_coords.append(polygon_coords[0])
        
        poly = Polygon(polygon_coords)
        if not poly.is_valid:
            poly = poly.buffer(0)
        
        poly_safe = poly.buffer(-boundary_margin / 111000)
        
        if not poly_safe.is_valid or poly_safe.is_empty:
            poly_safe = poly.buffer(-5 / 111000)
        
        if not poly_safe.is_valid or poly_safe.is_empty:
            poly_safe = poly
        
        minx, miny, maxx, maxy = poly_safe.bounds
        spacing_deg_lat = spacing_meters / 111000
        
        lines = []
        current_y = miny
        
        while current_y <= maxy:
            line_geom = LineString([(minx, current_y), (maxx, current_y)])
            intersection = poly_safe.intersection(line_geom)
            
            if not intersection.is_empty:
                if intersection.geom_type == 'LineString':
                    coords = list(intersection.coords)
                    if len(coords) >= 2:
                        lines.append(coords)
                elif intersection.geom_type == 'MultiLineString':
                    for line_part in intersection.geoms:
                        coords = list(line_part.coords)
                        if len(coords) >= 2:
                            lines.append(coords)
            
            current_y += spacing_deg_lat
        
        if not lines and spacing_meters > 5:
            return generate_lawnmower_path(polygon_coords, spacing_meters/2, boundary_margin)
        
        path = []
        for i, line in enumerate(lines):
            if i % 2 == 1:
                line = line[::-1]
            path.extend(line)
        
        print(f"[PATH] ✓ Generated {len(path)} waypoints with {boundary_margin}m margin")
        
        return path
        
    except Exception as e:
        print(f"[PATH] Error: {e}")
        return None

def mission_monitor():
    global mav_connection, mission_monitoring, last_waypoint, last_altitude, mission_start_time
    
    print("\n[MONITOR] 📡 Mission monitoring started")
    
    takeoff_detected = False
    waypoint_times = {}
    
    while mission_monitoring and mav_connection:
        try:
            pos_msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
            if pos_msg:
                current_alt = pos_msg.relative_alt / 1000.0
                
                if not takeoff_detected and current_alt > 1.0:
                    takeoff_detected = True
                    print(f"\n[MONITOR] 🚁 TAKEOFF DETECTED at {current_alt:.1f}m")
                
                if takeoff_detected and abs(current_alt - last_altitude) > 0.5:
                    last_altitude = current_alt
                    print(f"[MONITOR] 📊 Altitude: {current_alt:.1f}m")
            
            mission_msg = mav_connection.recv_match(type='MISSION_CURRENT', blocking=False, timeout=0.1)
            if mission_msg:
                current_wp = mission_msg.seq
                
                if current_wp != last_waypoint:
                    last_waypoint = current_wp
                    waypoint_times[current_wp] = time.time()
                    
                    if current_wp == 1:
                        print(f"[MONITOR] 🛫 TAKEOFF")
                    elif current_wp == 2:
                        print(f"[MONITOR] ⚙️ Speed set to {mission_speed} m/s")
                    elif current_wp > 2:
                        waypoint_num = current_wp - 2
                        elapsed = time.time() - mission_start_time if mission_start_time else 0
                        print(f"\n[MONITOR] ✈️ Waypoint {waypoint_num} ({int(elapsed)}s)")
            
            heartbeat = mav_connection.recv_match(type='HEARTBEAT', blocking=False, timeout=0.1)
            if heartbeat:
                mode = heartbeat.custom_mode
                is_armed = (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                
                if mode == 6:
                    print(f"\n[MONITOR] 🏠 RTL - Returning to HOME")
                
                if not is_armed and takeoff_detected:
                    total_time = time.time() - mission_start_time if mission_start_time else 0
                    print(f"\n[MONITOR] ✅ MISSION COMPLETE ({int(total_time)}s)\n")
                    break
            
            time.sleep(0.5)
            
        except Exception as e:
            time.sleep(1)
    
    mission_monitoring = False

def start_mission_monitoring():
    global mission_monitor_thread, mission_monitoring, mission_start_time
    stop_mission_monitoring()
    mission_monitoring = True
    mission_start_time = time.time()
    mission_monitor_thread = threading.Thread(target=mission_monitor, daemon=True)
    mission_monitor_thread.start()

def stop_mission_monitoring():
    global mission_monitoring, mission_monitor_thread
    mission_monitoring = False
    if mission_monitor_thread and mission_monitor_thread.is_alive():
        mission_monitor_thread.join(timeout=2)

def generate_mission_file(waypoints, altitude, speed):
    lines = ["QGC WPL 110", "0\t1\t0\t16\t0\t0\t0\t0\t0\t0\t0\t1"]
    lines.append(f"1\t0\t3\t22\t0\t0\t0\t0\t0\t0\t{altitude}\t1")
    lines.append(f"2\t0\t3\t178\t1\t{speed}\t-1\t0\t0\t0\t0\t1")
    
    seq = 3
    for lon, lat in waypoints:
        lines.append(f"{seq}\t0\t3\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{altitude}\t1")
        seq += 1
    
    lines.append(f"{seq}\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1")
    return "\n".join(lines)

@app.route('/')
def index():
    return jsonify({'status': 'online', 'service': 'NidarDashboard Telemetry Server'})

@app.route('/ports', methods=['GET'])
def list_ports():
    try:
        return jsonify({'ports': get_available_ports()})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/connect', methods=['POST'])
def connect():
    data = request.get_json()
    connection_string = data.get('connection_string', '/dev/ttyACM0')
    baud = data.get('baud', 115200)
    
    success, message = connect_pixhawk(connection_string, baud)
    
    if success:
        return jsonify({'success': True, 'message': message})
    else:
        return jsonify({'success': False, 'message': message}), 500

@app.route('/disconnect', methods=['POST'])
def disconnect():
    success, message = disconnect_pixhawk()
    if success:
        return jsonify({'success': True, 'message': message})
    else:
        return jsonify({'success': False, 'message': message}), 500

@app.route('/upload', methods=['POST'])
def upload_file():
    if 'file' not in request.files:
        return jsonify({'error': 'No file'}), 400
    
    file = request.files['file']
    if not file.filename.lower().endswith('.kml'):
        return jsonify({'error': 'Need KML file'}), 400
    
    try:
        kml_content = file.read().decode('utf-8')
        polygon_coords = parse_kml_polygon(kml_content)
        
        if not polygon_coords:
            return jsonify({'error': 'Cannot parse KML'}), 400
        
        spacing = float(request.form.get('spacing', 20))
        path = generate_lawnmower_path(polygon_coords, spacing, boundary_margin=10)
        
        if not path or len(path) < 2:
            return jsonify({'error': 'Path generation failed'}), 400
        
        return jsonify({'polygon': polygon_coords, 'path': path, 'spacing': spacing})
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/upload_to_pixhawk', methods=['POST'])
def upload_to_pixhawk():
    try:
        data = request.get_json()
        waypoints = data.get('waypoints', [])
        altitude = data.get('altitude', 10.0)
        speed = data.get('speed', 5.0)
        
        if not waypoints or not mav_connection:
            return jsonify({'error': 'No waypoints or not connected'}), 400
        
        success, message = upload_mission_to_pixhawk(waypoints, altitude, speed)
        
        if success:
            return jsonify({
                'success': True,
                'message': message,
                'instructions': 'Mission uploaded! Verify GPS, then use START button.'
            })
        else:
            return jsonify({'error': message}), 500
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/download', methods=['POST'])
def download_mission():
    try:
        data = request.get_json()
        waypoints = data.get('waypoints', [])
        
        if not waypoints:
            return jsonify({'error': 'No waypoints'}), 400
        
        content = generate_mission_file(waypoints, 10.0, 5.0)
        buffer = io.BytesIO(content.encode('utf-8'))
        return send_file(buffer, as_attachment=True, download_name='mission.txt', mimetype='text/plain')
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/gps_status', methods=['GET'])
def get_gps_status():
    global gps_status
    
    try:
        return jsonify({
            'success': True,
            'gps': {
                'fix_type': gps_status['fix_type'],
                'satellites': gps_status['satellites'],
                'hdop': gps_status['hdop'],
                'lat': gps_status['lat'],
                'lon': gps_status['lon'],
                'alt': gps_status['alt'],
                'ekf_ok': gps_status['ekf_ok']
            },
            'home': {
                'status': gps_status['home_status'],
                'locked': gps_status['home_locked'],
                'lat': gps_status['home_lat'],
                'lon': gps_status['home_lon'],
                'alt': gps_status['home_alt']
            }
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/status', methods=['GET'])
def get_status():
    global mav_connection, gps_status
    
    if not mav_connection:
        return jsonify({'connected': False})
    
    try:
        if hasattr(mav_connection, 'target_system') and mav_connection.target_system:
            msg = mav_connection.recv_match(type='HEARTBEAT', blocking=False, timeout=0.1)
            
            status_info = {
                'connected': True,
                'system_id': mav_connection.target_system,
                'component_id': mav_connection.target_component,
                'port': str(mav_connection.port) if hasattr(mav_connection, 'port') else 'Unknown'
            }
            
            if msg:
                status_info['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                status_info['custom_mode'] = msg.custom_mode
                
                pos_msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
                if pos_msg:
                    status_info['altitude'] = pos_msg.relative_alt / 1000.0
                    status_info['lat'] = pos_msg.lat / 1e7
                    status_info['lon'] = pos_msg.lon / 1e7
                
                mission_msg = mav_connection.recv_match(type='MISSION_CURRENT', blocking=False, timeout=0.1)
                if mission_msg:
                    status_info['current_waypoint'] = mission_msg.seq
                
                bat_msg = mav_connection.recv_match(type='SYS_STATUS', blocking=False, timeout=0.1)
                if bat_msg:
                    status_info['battery_voltage'] = bat_msg.voltage_battery / 1000.0
                
                vfr_msg = mav_connection.recv_match(type='VFR_HUD', blocking=False, timeout=0.1)
                if vfr_msg:
                    status_info['ground_speed'] = vfr_msg.groundspeed
            
            status_info['gps'] = {
                'fix_type': gps_status['fix_type'],
                'satellites': gps_status['satellites'],
                'hdop': gps_status['hdop'],
                'ekf_ok': gps_status['ekf_ok']
            }
            status_info['home'] = {
                'status': gps_status['home_status'],
                'locked': gps_status['home_locked'],
                'lat': gps_status['home_lat'],
                'lon': gps_status['home_lon']
            }
            
            return jsonify(status_info)
        else:
            return jsonify({'connected': False})
    except Exception:
        return jsonify({'connected': False})

@app.route('/start_mission', methods=['POST'])
def start_mission():
    global mav_connection
    
    if not mav_connection:
        print("[START] ✗ Not connected")
        return jsonify({'success': False, 'error': 'Not connected'}), 400
    
    try:
        print(f"\n{'='*60}")
        print(f"[START] 🚀 MISSION START WITH GPS-AWARE HOME")
        print(f"{'='*60}")
        
        print("[START] Checking vehicle state...")
        msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if not msg:
            print("[START] ✗ No heartbeat")
            return jsonify({'success': False, 'error': 'No heartbeat'}), 400
        
        is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        current_mode = msg.custom_mode
        mode_names = {0:'STABILIZE', 1:'ACRO', 2:'ALT_HOLD', 3:'AUTO', 4:'GUIDED', 5:'LOITER', 6:'RTL'}
        mode_name = mode_names.get(current_mode, f'MODE_{current_mode}')
        
        print(f"[START]   Armed: {is_armed}, Mode: {mode_name}")
        
        if is_armed and current_mode == 3:
            print("[START] ✓ Already in AUTO and armed!")
            start_mission_monitoring()
            return jsonify({'success': True, 'message': 'Mission already running!'})
        
        if not is_armed:
            print("[START] ARMING vehicle...")
            print(f"[START] GPS: {gps_status['fix_type']}, {gps_status['satellites']} sats, HDOP {gps_status['hdop']:.2f}")
            
            mav_connection.mav.command_long_send(
                mav_connection.target_system,
                mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 2989, 0, 0, 0, 0, 0
            )
            time.sleep(3)
            
            msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if msg:
                is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                print(f"[START]   ARM result: {is_armed}")
                
                if is_armed:
                    print("\n[START] Vehicle armed - evaluating HOME lock...")
                    home_success, home_msg = lock_home_position()
                    
                    if home_success:
                        print(f"[START] ✓ HOME LOCKED - RTL will return to arming point")
                    else:
                        print(f"[START] ⚠️ HOME NOT LOCKED - {home_msg}")
                        print(f"[START] Mission can proceed but RTL accuracy may vary")
        
        print("\n[START] Switching to AUTO mode...")
        for i in range(3):
            mav_connection.mav.set_mode_send(
                mav_connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3
            )
            time.sleep(0.3)
        
        print("[START] Sending MISSION_START command...")
        mav_connection.mav.command_long_send(
            mav_connection.target_system,
            mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)
        
        msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg:
            final_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            final_mode = msg.custom_mode
            final_mode_name = mode_names.get(final_mode, f'MODE_{final_mode}')
            
            print(f"[START]   Final: Armed={final_armed}, Mode={final_mode_name}")
            
            if final_armed and final_mode == 3:
                print(f"\n{'='*60}")
                print(f"[START] ✓✓✓ MISSION STARTED ✓✓✓")
                print(f"[START] HOME Status: {gps_status['home_status']}")
                if gps_status['home_locked']:
                    print(f"[START] HOME Locked at: {gps_status['home_lat']:.6f}, {gps_status['home_lon']:.6f}")
                else:
                    print(f"[START] HOME Estimated - Monitor RTL closely")
                print(f"{'='*60}\n")
                
                start_mission_monitoring()
                
                home_info = ""
                if gps_status['home_locked']:
                    home_info = f" | HOME LOCKED at {gps_status['satellites']} sats"
                else:
                    home_info = f" | ⚠️ HOME ESTIMATED (weak GPS)"
                
                return jsonify({
                    'success': True,
                    'message': f'✅ MISSION STARTED!{home_info}',
                    'home_locked': gps_status['home_locked'],
                    'home_status': gps_status['home_status']
                })
            elif final_armed:
                return jsonify({
                    'success': True,
                    'message': f'⚠️ Armed but in {final_mode_name}. Switch to AUTO on RC!'
                })
        
        return jsonify({'success': True, 'message': '✅ Commands sent!'})
        
    except Exception as e:
        print(f"[START] ✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/pause_mission', methods=['POST'])
def pause_mission():
    global mav_connection
    
    if not mav_connection:
        return jsonify({'success': False, 'error': 'Not connected'}), 400
    
    try:
        print(f"\n{'='*60}")
        print(f"[PAUSE] ⏸️ PAUSING MISSION - LOITER")
        print(f"{'='*60}")
        
        for i in range(3):
            mav_connection.mav.set_mode_send(
                mav_connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 5
            )
            time.sleep(0.2)
        
        time.sleep(1)
        
        msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg and msg.custom_mode == 5:
            print(f"[PAUSE] ✓ LOITER activated - holding position")
        
        print(f"{'='*60}\n")
        
        return jsonify({'success': True, 'message': '⏸️ Mission paused - holding position'})
        
    except Exception as e:
        print(f"[PAUSE] ✗ ERROR: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/resume_mission', methods=['POST'])
def resume_mission():
    global mav_connection
    
    if not mav_connection:
        return jsonify({'success': False, 'error': 'Not connected'}), 400
    
    try:
        print(f"\n{'='*60}")
        print(f"[RESUME] ▶️ RESUMING MISSION - AUTO")
        print(f"{'='*60}")
        
        for i in range(3):
            mav_connection.mav.set_mode_send(
                mav_connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3
            )
            time.sleep(0.2)
        
        time.sleep(1)
        
        msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg and msg.custom_mode == 3:
            print(f"[RESUME] ✓ AUTO mode activated - continuing mission")
        
        print(f"{'='*60}\n")
        
        return jsonify({'success': True, 'message': '▶️ Mission resumed'})
        
    except Exception as e:
        print(f"[RESUME] ✗ ERROR: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/land_mission', methods=['POST'])
def land_mission():
    global mav_connection
    
    if not mav_connection:
        return jsonify({'success': False, 'error': 'Not connected'}), 400
    
    try:
        print(f"\n{'='*60}")
        print(f"[LAND] 🛬 LANDING AT CURRENT POSITION")
        print(f"{'='*60}")
        
        stop_mission_monitoring()
        
        for i in range(3):
            mav_connection.mav.set_mode_send(
                mav_connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
            )
            time.sleep(0.2)
        
        time.sleep(1)
        
        msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg and msg.custom_mode == 9:
            print(f"[LAND] ✓ LAND mode activated")
        
        print(f"{'='*60}\n")
        
        return jsonify({'success': True, 'message': '🛬 Landing at current position'})
        
    except Exception as e:
        print(f"[LAND] ✗ ERROR: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/stop_mission', methods=['POST'])
def stop_mission():
    global mav_connection, mission_altitude
    
    if not mav_connection:
        return jsonify({'success': False, 'error': 'Not connected'}), 400
    
    try:
        print(f"\n{'='*60}")
        print(f"[RTL] 🏠 RETURNING TO LAUNCH")
        print(f"{'='*60}")
        
        stop_mission_monitoring()
        
        for i in range(3):
            mav_connection.mav.set_mode_send(
                mav_connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6
            )
            time.sleep(0.2)
        
        mav_connection.mav.command_long_send(
            mav_connection.target_system,
            mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6,
            0, 0, 0, 0, 0
        )
        
        time.sleep(1)
        
        msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg and msg.custom_mode == 6:
            print(f"[RTL] ✓ RTL activated")
            if gps_status['home_locked']:
                print(f"[RTL] Returning to LOCKED HOME: {gps_status['home_lat']:.6f}, {gps_status['home_lon']:.6f}")
            else:
                print(f"[RTL] Returning to ESTIMATED HOME - position may vary")
        
        print(f"{'='*60}\n")
        
        rtl_msg = f"✓ RTL activated at {mission_altitude}m"
        if not gps_status['home_locked']:
            rtl_msg += " | ⚠️ HOME estimated - monitor position"
        
        return jsonify({'success': True, 'message': rtl_msg})
        
    except Exception as e:
        print(f"[RTL] ✗ ERROR: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/emergency_stop', methods=['POST'])
def emergency_stop():
    global mav_connection
    
    if not mav_connection:
        return jsonify({'success': False, 'error': 'Not connected'}), 400
    
    try:
        print(f"\n{'='*60}")
        print(f"[EMERGENCY] 🚨 EMERGENCY DISARM 🚨")
        print(f"{'='*60}")
        
        stop_mission_monitoring()
        
        mav_connection.mav.command_long_send(
            mav_connection.target_system,
            mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        
        time.sleep(0.5)
        print(f"[EMERGENCY] ✓ DISARM executed\n")
        
        return jsonify({'success': True, 'message': '🚨 EMERGENCY DISARM!'})
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/diagnostics', methods=['GET'])
def get_diagnostics():
    global mav_connection, gps_status
    
    if not mav_connection:
        return jsonify({'error': 'Not connected'}), 400
    
    try:
        print(f"\n{'='*60}")
        print(f"[DIAGNOSTICS] SYSTEM CHECK")
        print(f"{'='*60}")
        
        diag = {
            'connected': True,
            'armed': False,
            'mode': 'UNKNOWN',
            'gps_fix': gps_status['fix_type'],
            'satellites': gps_status['satellites'],
            'hdop': gps_status['hdop'],
            'ekf_ok': gps_status['ekf_ok'],
            'battery_voltage': 'N/A',
            'home_status': gps_status['home_status'],
            'home_locked': gps_status['home_locked'],
            'messages': []
        }
        
        msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg:
            diag['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            mode_map = {0:'STABILIZE', 1:'ACRO', 2:'ALT_HOLD', 3:'AUTO',
                       4:'GUIDED', 5:'LOITER', 6:'RTL', 7:'CIRCLE',
                       9:'LAND', 16:'POSHOLD', 17:'BRAKE'}
            diag['mode'] = mode_map.get(msg.custom_mode, f'MODE_{msg.custom_mode}')
            print(f"[DIAGNOSTICS]   Armed: {diag['armed']}, Mode: {diag['mode']}")
        
        print(f"[DIAGNOSTICS]   GPS: {diag['gps_fix']} ({diag['satellites']} sats, HDOP {diag['hdop']:.2f})")
        print(f"[DIAGNOSTICS]   EKF OK: {diag['ekf_ok']}")
        print(f"[DIAGNOSTICS]   HOME: {diag['home_status']} (Locked: {diag['home_locked']})")
        
        msg = mav_connection.recv_match(type='SYS_STATUS', blocking=False, timeout=1)
        if msg and msg.voltage_battery > 0:
            diag['battery_voltage'] = f"{msg.voltage_battery / 1000.0:.2f}V"
            print(f"[DIAGNOSTICS]   Battery: {diag['battery_voltage']}")
        
        timeout_time = time.time() + 1
        while time.time() < timeout_time:
            msg = mav_connection.recv_match(type='STATUSTEXT', blocking=False, timeout=0.1)
            if msg:
                diag['messages'].append(msg.text)
        
        print(f"{'='*60}\n")
        return jsonify(diag)
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    print("\n" + "="*70)
    print("LARGE DRONE MISSION PLANNER - GPS-AWARE HOME")
    print("="*70)
    app.run(debug=False, host='0.0.0.0', port=5001)