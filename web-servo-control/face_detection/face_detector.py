"""Face detection module for Falcon Robot video stream.

Provides high-accuracy face detection using OpenCV YuNet DNN model (preferred)
with automatic fallback to skin-color segmentation when model is unavailable.

To enable YuNet DNN detection, download the model to the models/ directory:
  face_detection_yunet_2026may.onnx  (OpenCV 5.x compatible, recommended)
  face_detection_yunet_2023mar.onnx  (OpenCV 4.x only, fallback)

Source: https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet
"""
import cv2
import numpy as np
import time
import os
import threading

_MODEL_DIR = os.path.join(os.path.dirname(__file__), 'models')

# Ordered list of model files to try (first match wins)
_MODEL_CANDIDATES = [
    'face_detection_yunet_2026may.onnx',   # OpenCV 5.x dynamic shape
    'face_detection_yunet_2023mar.onnx',   # OpenCV 4.x fixed shape
]


class FaceDetector:
    """Detects faces in frames and annotates them with red boxes + coordinates.

    Prefers YuNet DNN (deep learning) for high accuracy (AP_easy=0.884).
    Falls back to skin-color segmentation when the model file is not available.
    """

    def __init__(self, detection_interval=0.3, score_threshold=0.6):
        self.detection_interval = detection_interval
        self.score_threshold = score_threshold
        self._detector = None
        self._use_yunet = False
        self._model_path = None
        self._init_detector()

        self._enabled = False
        self._last_detect_time = 0.0
        self._last_faces = []
        self._lock = threading.Lock()

    def _init_detector(self):
        """Try to initialize YuNet DNN detector, fall back to skin detection."""
        for candidate in _MODEL_CANDIDATES:
            model_path = os.path.join(_MODEL_DIR, candidate)
            if not os.path.exists(model_path):
                continue
            if os.path.getsize(model_path) < 50000:
                continue
            try:
                self._detector = cv2.FaceDetectorYN_create(
                    model_path,
                    "",
                    (320, 320),
                    score_threshold=self.score_threshold,
                    nms_threshold=0.3,
                    top_k=5000,
                )
                self._use_yunet = True
                self._model_path = model_path
                print(f"[face] YuNet DNN initialized: {candidate} ({os.path.getsize(model_path)} bytes)")
                return
            except Exception as e:
                print(f"[face] YuNet init failed for {candidate}: {e}")

        self._use_yunet = False
        self._detector = None
        model_desc = os.path.basename(_MODEL_DIR) + '/' + ' or '.join(_MODEL_CANDIDATES)
        print(f"[face] YuNet model not available ({model_desc}), using skin-color fallback")

    @property
    def enabled(self):
        return self._enabled

    @property
    def method(self):
        return "YuNet DNN" if self._use_yunet else "Skin-color segmentation"

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False
        with self._lock:
            self._last_faces = []

    def _detect_yunet(self, frame):
        """Detect faces using YuNet DNN model. Returns list of (x, y, w, h)."""
        h, w = frame.shape[:2]
        try:
            self._detector.setInputSize((w, h))
            _, faces = self._detector.detect(frame)
        except Exception:
            return []

        result = []
        if faces is not None and len(faces) > 0:
            for face in faces:
                x = int(face[0])
                y = int(face[1])
                fw = int(face[2])
                fh = int(face[3])
                score = float(face[4]) if len(face) > 4 else 1.0
                if score >= self.score_threshold and fw > 20 and fh > 20:
                    result.append((x, y, fw, fh))
        return result

    def _detect_skin(self, frame):
        """Detect face-like regions using YCbCr + HSV skin-color segmentation.

        Uses combined color space analysis with morphological cleanup
        to find face-sized, face-shaped skin regions.
        """
        h, w = frame.shape[:2]
        if h < 100 or w < 100:
            return []

        # YCbCr skin detection (Cr: 133-173, Cb: 77-127)
        ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        lower_skin = np.array([0, 133, 77], dtype=np.uint8)
        upper_skin = np.array([255, 173, 127], dtype=np.uint8)
        skin_mask = cv2.inRange(ycrcb, lower_skin, upper_skin)

        # HSV skin detection complementary
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([0, 30, 60], dtype=np.uint8)
        upper_hsv = np.array([50, 255, 255], dtype=np.uint8)
        hsv_mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Combine and clean up
        combined_mask = cv2.bitwise_or(skin_mask, hsv_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        combined_mask = cv2.GaussianBlur(combined_mask, (5, 5), 0)

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        faces = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 1200:
                continue
            x, y, cw, ch = cv2.boundingRect(contour)
            aspect_ratio = cw / max(ch, 1)
            if 0.4 <= aspect_ratio <= 2.2 and cw >= 35 and ch >= 35:
                faces.append((x, y, cw, ch))

        faces.sort(key=lambda f: f[2] * f[3], reverse=True)
        return faces[:5]

    def detect_and_draw(self, frame):
        """Run face detection on the frame and draw red boxes with coordinates.

        Returns the annotated frame. If detection is disabled or not due,
        returns the original frame unchanged.
        """
        if not self._enabled or frame is None:
            return frame

        now = time.time()
        if now - self._last_detect_time >= self.detection_interval:
            self._last_detect_time = now
            if self._use_yunet and self._detector is not None:
                faces = self._detect_yunet(frame)
            else:
                faces = self._detect_skin(frame)
            with self._lock:
                self._last_faces = [(int(x), int(y), int(w), int(h)) for (x, y, w, h) in faces]

        with self._lock:
            faces = list(self._last_faces)

        for i, (x, y, w, h) in enumerate(faces):
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            label = f"Face #{i+1}: ({x},{y}) {w}x{h}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x, y - th - 8), (x + tw + 4, y), (0, 0, 255), -1)
            cv2.putText(frame, label, (x + 2, y - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return frame

    def get_faces(self):
        """Return the latest detected face rectangles."""
        with self._lock:
            return list(self._last_faces)
