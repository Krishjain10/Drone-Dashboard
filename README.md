# NidarDashboard — Drone Ground Control Station

A full-stack web-based Ground Control Station (GCS) for planning and executing autonomous survey missions with ArduPilot-based drones. Upload a KML boundary, auto-generate a lawnmower flight path, push the mission to a Pixhawk flight controller over MAVLink, and monitor the drone in real time — all from a browser.

![Next.js](https://img.shields.io/badge/Next.js-16-black?logo=next.js)
![Flask](https://img.shields.io/badge/Flask-3.x-blue?logo=flask)
![MAVLink](https://img.shields.io/badge/Protocol-MAVLink-orange)
![License](https://img.shields.io/badge/License-MIT-green)

---

## Features

- **Mission Planning** — Upload a KML polygon → auto-generate a lawnmower survey path → visualize on an interactive Leaflet map
- **Configurable Flight Parameters** — Set altitude, speed, and line spacing from the dashboard
- **MAVLink Mission Upload** — Push waypoints directly to a Pixhawk flight controller over USB serial
- **Flight Controls** — Arm, Start (AUTO), Pause (LOITER), Resume, Land, RTL, and Emergency Disarm
- **GPS-Aware HOME Lock** — Evaluates GPS quality (fix type, satellite count, HDOP, EKF) before locking the RTL home position
- **Live Telemetry** — Real-time battery voltage, altitude, ground speed, GPS satellites, and current waypoint
- **Dual Camera Feeds** — MJPEG streaming from two USB cameras with auto-reconnect
- **Failsafe Configuration** — Altitude limits, geofence, low-battery RTL, telemetry loss, GPS failsafe
- **Mission File Export** — Download QGC-compatible `.txt` mission files

---

## Architecture

```
┌──────────────────────────────┐
│   Next.js Frontend (:3000)   │
│  ┌────────┐  ┌────────────┐  │
│  │ Mission │  │    Live    │  │
│  │Planning │  │ Monitoring │  │
│  └───┬────┘  └─────┬──────┘  │
└──────┼─────────────┼─────────┘
       │ REST API    │ MJPEG
┌──────▼──────┐ ┌────▼───────┐
│Flask Backend│ │Camera Srv  │
│  (:5001)    │ │  (:5000)   │
│  pymavlink  │ │  OpenCV    │
└──────┬──────┘ └────┬───────┘
       │ MAVLink     │ USB
   ┌───▼───┐    ┌────▼────┐
   │Pixhawk│    │USB Cams │
   │  FC   │    │  (x2)   │
   └───────┘    └─────────┘
```

| Layer | Stack |
|---|---|
| Frontend | Next.js 16, React 19, Tailwind CSS, Leaflet, Radix UI |
| Backend | Python, Flask, pymavlink, Shapely |
| Camera | Python, Flask, OpenCV |
| Protocol | MAVLink v2 (serial USB) |

---

## Getting Started

### Prerequisites

- **Node.js** ≥ 18
- **Python** ≥ 3.10
- A **Pixhawk**-based flight controller (ArduPilot firmware)
- USB cameras (optional, for video feeds)

### Installation

```bash
# Clone
git clone https://github.com/your-username/NidarDashboard.git
cd NidarDashboard

# Frontend
npm install

# Backend dependencies
pip install flask flask-cors pymavlink shapely pyserial opencv-python
```

### Running

```bash
# Terminal 1 — Frontend
npm run dev

# Terminal 2 — Telemetry Server
python telemetry_server.py

# Terminal 3 — Camera Server (optional)
python camera_server.py
```

Open [http://localhost:3000](http://localhost:3000) in your browser.

---

## Usage

1. **Connect** — Select the serial port and baud rate, click Connect
2. **Upload KML** — Upload a `.kml` file with a polygon boundary
3. **Configure** — Set altitude (m), speed (m/s), and line spacing (m)
4. **Generate Path** — Click to auto-generate the lawnmower survey path
5. **Upload to Drone** — Push the mission to the Pixhawk
6. **Start Mission** — Arm and start the autonomous flight
7. **Monitor** — Switch to Live Monitoring for camera feeds and telemetry

---

## Project Structure

```
NidarDashboard/
├── app/
│   ├── page.tsx              # Main dashboard page
│   ├── layout.tsx            # Root layout
│   └── globals.css           # Global styles
├── components/
│   ├── system-status.tsx     # Connection bar (port, baud, connect/disconnect)
│   ├── mission-planning.tsx  # Planning view container
│   ├── mission-workflow.tsx  # KML upload, path generation, drone upload
│   ├── mission-map.tsx       # Map wrapper (SSR-safe)
│   ├── map-component.tsx     # Leaflet map with polygon/path visualization
│   ├── flight-controls.tsx   # Arm, start, pause, resume, land, RTL, e-stop
│   ├── live-monitoring.tsx   # Monitoring view container
│   ├── drone-video-feed.tsx  # MJPEG camera feed component
│   ├── drone-telemetry.tsx   # Live telemetry display
│   ├── failsafe-info-modal.tsx # Failsafe configuration reference
│   └── ui/                   # Radix UI primitives
├── telemetry_server.py       # Flask + MAVLink backend
├── camera_server.py          # OpenCV MJPEG streaming server
└── package.json
```

---

## API Endpoints

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/ports` | List available serial ports |
| `POST` | `/connect` | Connect to flight controller |
| `POST` | `/disconnect` | Disconnect from flight controller |
| `GET` | `/status` | Get telemetry (armed, mode, altitude, battery, GPS) |
| `GET` | `/gps_status` | Get GPS fix and HOME position status |
| `POST` | `/upload` | Upload KML → generate survey path |
| `POST` | `/upload_to_pixhawk` | Push mission waypoints to Pixhawk |
| `POST` | `/download` | Download QGC-format mission file |
| `POST` | `/start_mission` | Arm + switch to AUTO mode |
| `POST` | `/pause_mission` | Switch to LOITER (hold position) |
| `POST` | `/resume_mission` | Switch back to AUTO |
| `POST` | `/land_mission` | Land at current position |
| `POST` | `/stop_mission` | Return to Launch (RTL) |
| `POST` | `/emergency_stop` | Emergency disarm (motors off immediately) |
| `GET` | `/diagnostics` | Full system diagnostic report |

---

## Safety

> ⚠️ **This software controls a real aircraft.** Always have a manual RC transmitter ready to take over. Test in simulation (SITL) before flying. Never fly over people or beyond visual line of sight without proper authorization.

---

## License

MIT
