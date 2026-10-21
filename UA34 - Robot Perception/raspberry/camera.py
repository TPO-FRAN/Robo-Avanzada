#!/usr/bin/env python3
"""
raspberry/camera.py — Object-Oriented USB Webcam Manager & Vision Pipeline.

Classes:
  - CameraSnapshot  : Value object holding JPEG payload, dimensions, and byte size.
  - USBWebCamera    : Thread-safe zero-lag V4L2 camera controller.
  - VisionDetection : Value object representing a detected color/shape/centroid.
  - VisionProcessor : Computer vision pipeline prepared for future color, shape,
                      and centroid identification projects.
"""
import threading
import time
import argparse
from dataclasses import dataclass

try:
    import cv2
    import numpy as np
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False


@dataclass(frozen=True)
class CameraSnapshot:
    """Immutable value object representing a captured camera frame."""
    jpeg_bytes: bytes
    width: int
    height: int
    size_bytes: int

    @property
    def resolution_str(self) -> str:
        return f"{self.width}x{self.height}"


@dataclass(frozen=True)
class VisionDetection:
    """Value object for future color, geometric shape, and centroid detection."""
    color: str
    shape: str
    centroid: tuple[int, int]
    area: float


class USBWebCamera:
    """
    Zero-lag USB camera controller using a background grab thread so the
    V4L2 buffer always holds the freshest frame without double-retrieve bugs.
    """
    _MINIMAL_JPEG = (
        b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00"
        b"\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t"
        b"\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a"
        b"\x1f\x1e\x1d\x1a\x1c\x1c $.' \",#\x1c\x1c(7),01444\x1f'9=82<.342"
        b"\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00\xff\xc4"
        b"\x00\x1f\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00\x00"
        b"\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b"
        b"\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xfb\xd2\x8a(\x03\xff\xd9"
    )

    def __init__(self, camera_index: int = 0, width: int = 640, height: int = 480,
                 jpeg_quality: int = 85, mock: bool = False):
        self.camera_index = camera_index
        self.req_width    = width
        self.req_height   = height
        self.jpeg_quality = max(10, min(100, int(jpeg_quality)))
        self.mock         = mock or (not HAS_OPENCV)

        self._cap          = None
        self._lock         = threading.Lock()
        self._stop         = threading.Event()
        self._thread       = None
        self._frame_dirty  = False  # True when a new grab() happened since last retrieve()
        self._cached_frame = None

    def start(self):
        """Opens the USB webcam and starts the continuous buffer-draining thread."""
        if not self.mock:
            backend = getattr(cv2, "CAP_V4L2", cv2.CAP_ANY)
            cap = cv2.VideoCapture(self.camera_index, backend)
            if not cap.isOpened():
                cap = cv2.VideoCapture(self.camera_index)

            if cap.isOpened():
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.req_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.req_height)
                actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self._cap = cap
                print(f"[CAMERA] USB camera #{self.camera_index} active @ {actual_w}x{actual_h}")
            else:
                cap.release()
                print(f"[CAMERA WARNING] Could not open camera #{self.camera_index}. Falling back to mock mode.")
                self.mock = True

        if self.mock:
            print(f"[CAMERA] Running in MOCK mode ({self.req_width}x{self.req_height})")

        self._stop.clear()
        self._thread = threading.Thread(target=self._grab_worker, daemon=True, name="CameraGrabber")
        self._thread.start()

    def _grab_worker(self):
        """Continuously grabs frames without decoding them until requested."""
        while not self._stop.is_set():
            if not self.mock:
                with self._lock:
                    if self._cap is not None and self._cap.isOpened():
                        if self._cap.grab():
                            self._frame_dirty = True
                time.sleep(0.015)  # ~60 Hz max poll
            else:
                self._frame_dirty = True
                time.sleep(0.05)

    def get_raw_frame(self):
        """Thread-safely retrieves and decodes the latest BGR frame (numpy array)."""
        if not self.mock:
            with self._lock:
                if self._cap is not None and self._cap.isOpened():
                    if self._frame_dirty:
                        ok, frame = self._cap.retrieve()
                        if ok and frame is not None:
                            self._cached_frame = frame
                            self._frame_dirty = False
                    elif self._cached_frame is None:
                        ok, frame = self._cap.read()
                        if ok and frame is not None:
                            self._cached_frame = frame
                    if self._cached_frame is not None:
                        return self._cached_frame.copy()

        if HAS_OPENCV:
            frame = np.zeros((self.req_height, self.req_width, 3), dtype=np.uint8)
            frame[:] = (35, 35, 35)
            cv2.circle(frame, (self.req_width // 2, self.req_height // 2), 70, (0, 180, 255), -1)
            cv2.putText(frame, f"Rover Arm-Cam {time.strftime('%H:%M:%S')}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            return frame
        return None

    def get_snapshot(self) -> CameraSnapshot:
        """Captures the current frame and returns a CameraSnapshot object."""
        frame = self.get_raw_frame()
        if frame is not None and HAS_OPENCV:
            height, width = frame.shape[:2]
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            ok, buf = cv2.imencode(".jpg", frame, encode_params)
            if ok:
                jpeg_bytes = buf.tobytes()
                return CameraSnapshot(
                    jpeg_bytes=jpeg_bytes,
                    width=width,
                    height=height,
                    size_bytes=len(jpeg_bytes),
                )

        return CameraSnapshot(
            jpeg_bytes=self._MINIMAL_JPEG,
            width=self.req_width,
            height=self.req_height,
            size_bytes=len(self._MINIMAL_JPEG),
        )

    def stop(self):
        """Thread-safely stops the background thread and releases the USB webcam."""
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        with self._lock:
            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None
        print("[CAMERA] Stopped and released.")


class VisionProcessor:
    """
    [PREPARATION FOR FUTURE PROJECT]
    Object-oriented vision processor for color segmentation (HSV),
    geometric shape classification, and centroid computation.
    """
    def __init__(self, min_area: float = 400.0):
        self.min_area = min_area
        if HAS_OPENCV:
            self.color_ranges = {
                "RED":   [(np.array([0, 120, 70]),   np.array([10, 255, 255])),
                          (np.array([170, 120, 70]), np.array([180, 255, 255]))],
                "GREEN": [(np.array([36, 100, 70]),  np.array([86, 255, 255]))],
                "BLUE":  [(np.array([94, 80, 2]),    np.array([126, 255, 255]))],
            }
        else:
            self.color_ranges = {}

    def analyze(self, frame) -> tuple[object, list[VisionDetection]]:
        if frame is None or not HAS_OPENCV:
            return frame, []

        annotated = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections: list[VisionDetection] = []

        for color_name, ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = float(cv2.contourArea(cnt))
                if area < self.min_area:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                vertices = len(approx)
                if vertices == 3:
                    shape = "TRIANGLE"
                elif vertices == 4:
                    shape = "RECTANGLE"
                else:
                    shape = "CIRCLE"

                det = VisionDetection(
                    color=color_name,
                    shape=shape,
                    centroid=(cx, cy),
                    area=area,
                )
                detections.append(det)

                cv2.drawContours(annotated, [approx], -1, (0, 255, 0), 2)
                cv2.circle(annotated, (cx, cy), 5, (255, 255, 255), -1)
                cv2.putText(annotated, f"{color_name} {shape} ({cx},{cy})",
                            (cx - 40, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return annotated, detections


class StandaloneCameraApp:
    """Runner class when camera.py is executed directly."""
    def __init__(self, camera_index: int = 0, width: int = 640, height: int = 480,
                 headless: bool = False, enable_vision: bool = False, mock: bool = False):
        self.camera = USBWebCamera(camera_index, width, height, mock=mock)
        self.vision = VisionProcessor() if enable_vision else None
        self.headless = headless or (not HAS_OPENCV)

    def run(self):
        self.camera.start()
        print("[CAMERA STANDALONE] Active. Press Ctrl+C (or 'q' in window) to exit.")
        try:
            while True:
                if self.headless:
                    snap = self.camera.get_snapshot()
                    print(f"[SNAPSHOT] Resolution: {snap.resolution_str} | Size: {snap.size_bytes} bytes")
                    time.sleep(1.0)
                else:
                    frame = self.camera.get_raw_frame()
                    if frame is not None:
                        if self.vision is not None:
                            frame, detections = self.vision.analyze(frame)
                            if detections:
                                print(f"[VISION] {detections}")
                        cv2.imshow("Rover USB Camera", frame)
                        if (cv2.waitKey(1) & 0xFF) == ord("q"):
                            break
        except KeyboardInterrupt:
            pass
        finally:
            if HAS_OPENCV and not self.headless:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass
            self.camera.stop()


def main():
    p = argparse.ArgumentParser(description="Standalone USB Camera & Vision Test")
    p.add_argument("--camera",   type=int, default=0, help="USB camera index (default: 0)")
    p.add_argument("--width",    type=int, default=640)
    p.add_argument("--height",   type=int, default=480)
    p.add_argument("--headless", action="store_true", help="Run without GUI window (for SSH)")
    p.add_argument("--vision",   action="store_true", help="Enable color/shape/centroid analysis")
    p.add_argument("--mock",     action="store_true", help="Simulate camera frames")
    args = p.parse_args()

    app = StandaloneCameraApp(
        camera_index=args.camera,
        width=args.width,
        height=args.height,
        headless=args.headless,
        enable_vision=args.vision,
        mock=args.mock,
    )
    app.run()


if __name__ == "__main__":
    main()
