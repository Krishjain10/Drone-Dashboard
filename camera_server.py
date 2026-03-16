import cv2
import threading
import time
import signal
import sys
from flask import Flask, Response

app = Flask(__name__)

CAM_ID_1 = 0
CAM_ID_2 = 1


class Camera:
    def __init__(self, src):
        print(f"[INFO] Opening camera {src}...")
        self.src = src
        self.cap = None
        self.frame = None
        self.running = False
        self.lock = threading.Lock()
        self.last_frame_time = time.time()
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5

        if not self._connect():
            print(f"[ERROR] Failed to initialize camera {src}")
            return

        self.running = True
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def _connect(self):
        try:
            self.cap = cv2.VideoCapture(self.src, cv2.CAP_DSHOW)

            if not self.cap.isOpened():
                print(f"[WARN] CAP_DSHOW failed for camera {self.src}, trying default...")
                self.cap = cv2.VideoCapture(self.src)

            if not self.cap.isOpened():
                return False

            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            ret, frame = self.cap.read()
            if not ret or frame is None:
                print(f"[ERROR] Camera {self.src} opened but cannot read frames")
                self.cap.release()
                return False

            print(f"[SUCCESS] Camera {self.src} connected - Resolution: {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
            self.frame = frame
            return True

        except Exception as e:
            print(f"[ERROR] Exception connecting to camera {self.src}: {e}")
            return False

    def _reconnect(self):
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            print(f"[ERROR] Camera {self.src} max reconnection attempts reached")
            self.running = False
            return False

        self.reconnect_attempts += 1
        print(f"[INFO] Attempting to reconnect camera {self.src} (attempt {self.reconnect_attempts}/{self.max_reconnect_attempts})")

        if self.cap:
            self.cap.release()

        time.sleep(2)
        return self._connect()

    def update(self):
        consecutive_failures = 0
        max_consecutive_failures = 30

        while self.running:
            try:
                ret, frame = self.cap.read()

                if ret and frame is not None:
                    with self.lock:
                        self.frame = frame
                        self.last_frame_time = time.time()
                    consecutive_failures = 0
                    self.reconnect_attempts = 0
                else:
                    consecutive_failures += 1

                    if consecutive_failures >= max_consecutive_failures:
                        print(f"[WARN] Camera {self.src} stopped producing frames, attempting reconnect...")
                        if not self._reconnect():
                            break
                        consecutive_failures = 0

                    time.sleep(0.01)

            except Exception as e:
                print(f"[ERROR] Camera {self.src} update exception: {e}")
                time.sleep(0.1)

    def get_frame(self):
        with self.lock:
            if self.frame is None:
                return None

            if time.time() - self.last_frame_time > 2.0:
                print(f"[WARN] Camera {self.src} frame is stale")
                return None

            try:
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
                ret, jpeg = cv2.imencode('.jpg', self.frame, encode_param)
                return jpeg.tobytes() if ret else None
            except Exception as e:
                print(f"[ERROR] Camera {self.src} encode error: {e}")
                return None

    def stop(self):
        print(f"[INFO] Stopping camera {self.src}...")
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()
        print(f"[INFO] Camera {self.src} released")

    def is_alive(self):
        return self.running and hasattr(self, 'thread') and self.thread.is_alive()


cameras = {}
cameras_lock = threading.Lock()


def init_cameras():
    print("[INFO] Initializing cameras...")

    for cid in [CAM_ID_1, CAM_ID_2]:
        try:
            cam = Camera(cid)
            if cam.running and cam.is_alive():
                with cameras_lock:
                    cameras[cid] = cam
                print(f"[SUCCESS] Camera {cid} initialized")
            else:
                print(f"[ERROR] Camera {cid} failed to initialize")
        except Exception as e:
            print(f"[ERROR] Exception initializing camera {cid}: {e}")

    print(f"[INFO] {len(cameras)}/{len([CAM_ID_1, CAM_ID_2])} cameras initialized successfully")


def stream_gen(camera_id):
    frame_delay = 1.0 / 30

    while True:
        with cameras_lock:
            if camera_id not in cameras or not cameras[camera_id].is_alive():
                print(f"[WARN] Stream for camera {camera_id} ended")
                break

        try:
            frame = cameras[camera_id].get_frame()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(frame_delay)
        except Exception as e:
            print(f"[ERROR] Stream generation error for camera {camera_id}: {e}")
            time.sleep(0.1)


@app.route('/')
def index():
    with cameras_lock:
        available_cameras = list(cameras.keys())
        status_info = {
            cid: {
                'alive': cameras[cid].is_alive(),
                'last_frame_age': time.time() - cameras[cid].last_frame_time
            }
            for cid in available_cameras
        }
    return {
        "status": "online",
        "cameras": available_cameras,
        "details": status_info
    }


@app.route('/video_feed/<int:cam_id>')
def video_feed(cam_id):
    with cameras_lock:
        if cam_id in cameras and cameras[cam_id].is_alive():
            return Response(stream_gen(cam_id),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
    return f"Camera {cam_id} not available", 404


@app.route('/status')
def status():
    with cameras_lock:
        status_info = {
            cid: {
                'alive': cam.is_alive(),
                'last_frame': time.time() - cam.last_frame_time
            }
            for cid, cam in cameras.items()
        }
    return status_info


def cleanup(sig=None, frame=None):
    print("\n[INFO] Shutting down cameras...")
    with cameras_lock:
        for cam in cameras.values():
            cam.stop()
    cv2.destroyAllWindows()
    print("[INFO] Cleanup complete")
    if sig:
        sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    init_cameras()

    if len(cameras) == 0:
        print("[CRITICAL] No cameras initialized. Exiting.")
        sys.exit(1)

    print(f"[SUCCESS] Server starting with {len(cameras)} camera(s)")
    print("[INFO] Access dashboard at http://localhost:5000")
    print("[INFO] Press Ctrl+C to stop")

    try:
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    finally:
        cleanup()
