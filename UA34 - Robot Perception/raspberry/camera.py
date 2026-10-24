#!/usr/bin/env python3
"""
raspberry/camera.py — Object-Oriented Raw USB Webcam Controller (No Filters).

Classes:
  - CameraSnapshot  : Value object holding raw JPEG bytes, width, height, and byte size.
  - USBWebCamera    : Captures 100% raw, unfiltered frames from the USB webcam on
                      Raspberry Pi 5 (via OpenCV or native Linux V4L2 fallback).
  - StandaloneCameraApp : Allows running `python3 camera.py` directly to test the camera.
"""
import os
import sys
import glob
import shutil
import struct
import subprocess
import tempfile
import threading
import time
import argparse
from dataclasses import dataclass
from pathlib import Path

try:
    import cv2
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False


@dataclass(frozen=True)
class CameraSnapshot:
    """Immutable value object representing a raw, unfiltered camera frame."""
    jpeg_bytes: bytes
    width: int
    height: int
    size_bytes: int

    @property
    def resolution_str(self) -> str:
        return f"{self.width}x{self.height}"


class USBWebCamera:
    """
    Raw USB webcam controller for Raspberry Pi 5.
    Does NOT apply any image filter or synthetic placeholder.
    Automatically detects the active USB camera node (/dev/video*) and supports:
      1. OpenCV (cv2.VideoCapture) with zero-lag background frame grabbing.
      2. Native Linux V4L2 fallback (v4l2-ctl / ffmpeg / fswebcam) if cv2 is not
         installed in the current Python environment.
    """
    def __init__(self, camera_index: int = 0, width: int = 640, height: int = 480,
                 jpeg_quality: int = 90):
        self.camera_index = camera_index
        self.req_width    = width
        self.req_height   = height
        self.jpeg_quality = max(10, min(100, int(jpeg_quality)))

        self._cap          = None
        self._device_path  = f"/dev/video{camera_index}"
        self._backend_mode = "NONE"  # "OPENCV" or "V4L2_NATIVE"
        self._lock         = threading.Lock()
        self._stop         = threading.Event()
        self._thread       = None
        self._cached_frame = None

    @staticmethod
    def parse_jpeg_dimensions(jpeg_bytes: bytes) -> tuple[int, int]:
        """Extracts (width, height) directly from raw JPEG SOF headers without filters."""
        if len(jpeg_bytes) < 4 or jpeg_bytes[0:2] != b"\xff\xd8":
            return (0, 0)
        idx = 2
        n = len(jpeg_bytes)
        while idx < n - 8:
            if jpeg_bytes[idx] != 0xFF:
                idx += 1
                continue
            while idx < n and jpeg_bytes[idx] == 0xFF:
                idx += 1
            if idx >= n:
                break
            marker = jpeg_bytes[idx]
            idx += 1
            if marker in (0xD8, 0xD9) or (0xD0 <= marker <= 0xD7):
                continue
            if idx + 2 > n:
                break
            seg_len = struct.unpack(">H", jpeg_bytes[idx : idx + 2])[0]
            if marker in (0xC0, 0xC1, 0xC2):  # SOF0, SOF1, SOF2
                if idx + 7 <= n:
                    height, width = struct.unpack(">HH", jpeg_bytes[idx + 3 : idx + 7])
                    return (width, height)
                break
            idx += seg_len
        return (0, 0)

    def _candidate_indices(self) -> list[int]:
        """Returns candidate video device indices starting with preferred camera_index."""
        candidates = [self.camera_index]
        for idx in range(0, 8):
            if idx not in candidates:
                candidates.append(idx)
        return candidates

    def _try_open_opencv(self) -> bool:
        if not HAS_OPENCV:
            return False

        backend = getattr(cv2, "CAP_V4L2", cv2.CAP_ANY)
        for idx in self._candidate_indices():
            cap = cv2.VideoCapture(idx, backend)
            if not cap.isOpened():
                cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                cap.release()
                continue

            # Request MJPG hardware stream & target resolution (raw, unfiltered)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.req_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.req_height)

            # Warm up and verify this node actually delivers video frames (not a metadata node)
            ok, frame = False, None
            for _ in range(5):
                ok, frame = cap.read()
                if ok and frame is not None and frame.size > 0:
                    break
                time.sleep(0.05)

            if ok and frame is not None and frame.size > 0:
                self.camera_index  = idx
                self._device_path  = f"/dev/video{idx}"
                self._cap          = cap
                self._cached_frame = frame
                self._backend_mode = "OPENCV"
                h, w = frame.shape[:2]
                print(f"[CAMERA] OpenCV connected to USB camera #{idx} ({self._device_path}) @ {w}x{h} (RAW, no filters)")
                return True

            cap.release()
        return False

    def _try_open_v4l2_native(self) -> bool:
        """Fallback for Linux/Raspberry Pi when cv2 is not installed in the active venv."""
        if os.name == "nt":
            return False

        video_devs = sorted(glob.glob("/dev/video*"))
        pref = f"/dev/video{self.camera_index}"
        if pref in video_devs:
            video_devs.remove(pref)
            video_devs.insert(0, pref)

        for dev in video_devs:
            raw_jpeg = self._capture_v4l2_once(dev)
            if raw_jpeg and len(raw_jpeg) > 512:
                w, h = self.parse_jpeg_dimensions(raw_jpeg)
                self._device_path  = dev
                self._backend_mode = "V4L2_NATIVE"
                print(f"[CAMERA] Native V4L2 connected to USB camera {dev} @ {w}x{h} (RAW, no filters)")
                return True
        return False

    def _capture_v4l2_once(self, dev: str) -> bytes | None:
        """Captures a single raw JPEG frame from `dev` using v4l2-ctl, fswebcam, or ffmpeg."""
        # 1. Try v4l2-ctl (pre-installed on Raspberry Pi OS)
        if shutil.which("v4l2-ctl"):
            try:
                cmd = [
                    "v4l2-ctl",
                    f"--device={dev}",
                    f"--set-fmt-video=width={self.req_width},height={self.req_height},pixelformat=MJPG",
                    "--stream-mmap=3",
                    "--stream-skip=2",
                    "--stream-count=1",
                    "--stream-to=-",
                ]
                res = subprocess.run(cmd, capture_output=True, timeout=3.0)
                if res.returncode == 0 and res.stdout.startswith(b"\xff\xd8"):
                    end = res.stdout.rfind(b"\xff\xd9")
                    return res.stdout[: end + 2] if end != -1 else res.stdout
            except Exception:
                pass

        # 2. Try ffmpeg
        if shutil.which("ffmpeg"):
            try:
                cmd = [
                    "ffmpeg", "-nostdin", "-loglevel", "error",
                    "-f", "v4l2", "-video_size", f"{self.req_width}x{self.req_height}",
                    "-i", dev, "-frames:v", "1", "-f", "image2pipe", "-vcodec", "mjpeg", "-"
                ]
                res = subprocess.run(cmd, capture_output=True, timeout=4.0)
                if res.returncode == 0 and res.stdout.startswith(b"\xff\xd8"):
                    return res.stdout
            except Exception:
                pass

        # 3. Try fswebcam
        if shutil.which("fswebcam"):
            try:
                cmd = [
                    "fswebcam", "-q", "--no-banner",
                    "-d", dev, "-r", f"{self.req_width}x{self.req_height}",
                    "--jpeg", str(self.jpeg_quality), "-"
                ]
                res = subprocess.run(cmd, capture_output=True, timeout=4.0)
                if res.returncode == 0 and res.stdout.startswith(b"\xff\xd8"):
                    return res.stdout
            except Exception:
                pass

        return None

    def start(self):
        """Initializes the USB webcam and starts the continuous capture thread if using OpenCV."""
        if self._try_open_opencv():
            self._stop.clear()
            self._thread = threading.Thread(target=self._opencv_reader_worker, daemon=True, name="CameraReader")
            self._thread.start()
            return

        if self._try_open_v4l2_native():
            return

        if not HAS_OPENCV:
            print(
                "[CAMERA ERROR] OpenCV (cv2) no esta instalado y no se pudo capturar por v4l2-ctl.\n"
                "               Ejecuta en la Raspberry Pi: sudo apt install -y python3-opencv v4l-utils"
            )
        else:
            print("[CAMERA ERROR] No se detecto ninguna camara web USB activa en /dev/video*.")

    def _opencv_reader_worker(self):
        """Continuously reads the latest raw frame from the USB webcam so there is zero buffer lag."""
        while not self._stop.is_set():
            with self._lock:
                if self._cap is not None and self._cap.isOpened():
                    ok, frame = self._cap.read()
                    if ok and frame is not None and frame.size > 0:
                        self._cached_frame = frame
            time.sleep(0.02)  # ~50 Hz refresh

    def get_raw_frame(self):
        """Returns the latest raw, unmodified BGR frame (when using OpenCV)."""
        with self._lock:
            if self._cached_frame is not None:
                return self._cached_frame.copy()
            if self._cap is not None and self._cap.isOpened():
                ok, frame = self._cap.read()
                if ok and frame is not None and frame.size > 0:
                    self._cached_frame = frame
                    return frame.copy()
        return None

    def get_snapshot(self) -> CameraSnapshot:
        """
        Returns a CameraSnapshot containing the 100% raw, unfiltered JPEG image
        directly from the USB webcam. Raises RuntimeError if no camera is available.
        """
        if self._backend_mode == "OPENCV" and HAS_OPENCV:
            frame = self.get_raw_frame()
            if frame is not None:
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

        if self._backend_mode == "V4L2_NATIVE":
            raw_jpeg = self._capture_v4l2_once(self._device_path)
            if raw_jpeg:
                w, h = self.parse_jpeg_dimensions(raw_jpeg)
                if w == 0 or h == 0:
                    w, h = self.req_width, self.req_height
                return CameraSnapshot(
                    jpeg_bytes=raw_jpeg,
                    width=w,
                    height=h,
                    size_bytes=len(raw_jpeg),
                )

        raise RuntimeError(
            "No se pudo capturar imagen de la camara USB. Verifica que este conectada y "
            "que python3-opencv o v4l-utils esten instalados en la Raspberry Pi."
        )

    def save_snapshot_to_file(self, output_path: str | Path) -> CameraSnapshot:
        """Captures a raw snapshot and writes it directly to `output_path`."""
        snap = self.get_snapshot()
        Path(output_path).write_bytes(snap.jpeg_bytes)
        return snap

    def stop(self):
        """Stops the background thread and releases the USB webcam."""
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


class StandaloneCameraApp:
    """Runner class when `camera.py` is executed directly or invoked via CLI."""
    def __init__(self, camera_index: int = 0, width: int = 640, height: int = 480,
                 headless: bool = False, output_file: str | None = None):
        self.camera      = USBWebCamera(camera_index, width, height)
        self.headless    = headless or (not HAS_OPENCV)
        self.output_file = output_file

    def run(self) -> int:
        self.camera.start()
        try:
            # Single-shot capture mode (when invoked with --capture-to <filepath>)
            if self.output_file:
                snap = self.camera.save_snapshot_to_file(self.output_file)
                print(f"SNAPSHOT_OK|{snap.width}x{snap.height}|{snap.size_bytes}|{self.output_file}")
                return 0

            print("[CAMERA STANDALONE] Active (RAW mode, no filters). Press Ctrl+C (or 'q' in window) to exit.")
            while True:
                if self.headless:
                    snap = self.camera.get_snapshot()
                    print(f"[RAW SNAPSHOT] Resolution: {snap.resolution_str} | Size: {snap.size_bytes} bytes")
                    time.sleep(1.0)
                else:
                    frame = self.camera.get_raw_frame()
                    if frame is not None:
                        cv2.imshow("Rover USB Camera (RAW)", frame)
                        if (cv2.waitKey(1) & 0xFF) == ord("q"):
                            break
            return 0
        except KeyboardInterrupt:
            return 0
        except Exception as e:
            print(f"[CAMERA ERROR] {e}", file=sys.stderr)
            return 1
        finally:
            if HAS_OPENCV and not self.headless and not self.output_file:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass
            self.camera.stop()


def main():
    p = argparse.ArgumentParser(description="Standalone Raw USB Camera Controller")
    p.add_argument("--camera",     type=int, default=0, help="USB camera index (default: 0)")
    p.add_argument("--width",      type=int, default=640)
    p.add_argument("--height",     type=int, default=480)
    p.add_argument("--headless",   action="store_true", help="Run without GUI window (for SSH)")
    p.add_argument("--capture-to", default=None, help="Capture a single raw JPEG to this file path and exit")
    args = p.parse_args()

    app = StandaloneCameraApp(
        camera_index=args.camera,
        width=args.width,
        height=args.height,
        headless=args.headless,
        output_file=args.capture_to,
    )
    sys.exit(app.run())


if __name__ == "__main__":
    main()
