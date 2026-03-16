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


MIT
