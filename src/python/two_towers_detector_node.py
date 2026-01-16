#!/usr/bin/env python3
"""
Two Towers Detection Node
=========================

A ROS2 node that performs real-time person detection using YOLOv8 and publishes
detection coordinates for the tracking system. Includes an optional Flask-based
video streaming server for remote monitoring.

Architecture:
    Camera → YOLOv8 Detection → RelaxedTracker (smoothing) → ROS2 Publisher
                                                          ↘ Flask Stream

Key Features:
    - YOLOv8n-based person detection optimized for Raspberry Pi
    - Temporal smoothing to reduce detection jitter
    - Real-time video streaming via Flask (configurable)
    - Publishes both detection events and "no target" signals

ROS2 Interface:
    Publishers:
        - /tower_a/detections (DetectionArray): Person detection coordinates

    Parameters:
        - enable_streaming (bool, default=True): Enable Flask video server
        - stream_port (int, default=5000): Flask server port
        - tower_id (string, default='tower_a'): Identifier for multi-tower setup

Author: Tate Lloyd <tate.lloyd@yale.edu>
License: MIT
"""

import rclpy
from rclpy.node import Node
from two_towers.msg import Detection, DetectionArray
from std_msgs.msg import Header
import cv2
import time
import threading
from collections import deque
from ultralytics import YOLO
from flask import Flask, Response


# =============================================================================
# FLASK VIDEO STREAMING SERVER
# =============================================================================

app = Flask(__name__)

# Thread-safe frame buffer for Flask streaming
latest_frame = None
frame_lock = threading.Lock()


# =============================================================================
# DETECTION TRACKER
# =============================================================================

class RelaxedTracker:
    """
    Lightweight temporal tracker that smooths detection coordinates over time.

    Uses a sliding window average to reduce jitter from frame-to-frame detection
    noise while maintaining responsiveness. Rejects teleportation (sudden large
    jumps) which typically indicate false positives or new targets entering frame.

    Design Philosophy:
        - Publish quickly with minimal filtering latency
        - Only reject obvious outliers (teleportation > 60% frame distance)
        - Trust the YOLO detector for most decisions

    Attributes:
        detections (deque): Rolling buffer of recent detection coordinates
        last_position (dict): Most recent smoothed position for continuity checks
        frames_since_detection (int): Counter for detection dropout tracking

    Example:
        tracker = RelaxedTracker(buffer_size=3)
        smoothed = tracker.add_detection({'x': 0.5, 'y': 0.5, 'confidence': 0.9})
        if smoothed:
            publish(smoothed)
    """

    # Threshold for rejecting sudden position jumps (normalized frame distance)
    TELEPORTATION_THRESHOLD = 0.6

    def __init__(self, buffer_size: int = 3):
        """
        Initialize the tracker with specified smoothing window.

        Args:
            buffer_size: Number of frames to average for smoothing (default: 3)
        """
        self.detections = deque(maxlen=buffer_size)
        self.last_position = None
        self.frames_since_detection = 0

    def add_detection(self, detection: dict) -> dict | None:
        """
        Add a new detection and compute smoothed position.

        Performs temporal averaging over the buffer window and rejects
        detections that represent implausible movement (teleportation).

        Args:
            detection: Dict with keys 'x', 'y', 'confidence' (x,y normalized 0-1)

        Returns:
            Smoothed detection dict with 'label', 'confidence', 'x', 'y', 'stable'
            keys, or None if detection was rejected as invalid
        """
        self.detections.append(detection)
        self.frames_since_detection = 0

        # Require at least one detection in buffer
        if len(self.detections) < 1:
            return None

        # Compute rolling average over buffer window
        recent = list(self.detections)
        avg_x = sum(d['x'] for d in recent) / len(recent)
        avg_y = sum(d['y'] for d in recent) / len(recent)

        # Reject teleportation: sudden large jumps indicate false positives
        if self.last_position:
            prev_x = self.last_position['x']
            prev_y = self.last_position['y']
            jump_distance = ((avg_x - prev_x)**2 + (avg_y - prev_y)**2)**0.5

            if jump_distance > self.TELEPORTATION_THRESHOLD:
                return None

        # Build smoothed result
        result = {
            'label': 'person',
            'confidence': sum(d['confidence'] for d in recent) / len(recent),
            'x': avg_x,
            'y': avg_y,
            'stable': len(recent) >= 2  # Mark as stable after 2+ consistent frames
        }

        self.last_position = result
        return result

    def clear(self):
        """
        Reset tracker state when target is lost.

        Clears the detection buffer and last known position to prevent
        stale data from affecting future detections.
        """
        self.detections.clear()
        self.last_position = None
        self.frames_since_detection += 1


# =============================================================================
# DETECTION VALIDATION
# =============================================================================

def is_valid_detection(center_x: float, center_y: float, confidence: float) -> bool:
    """
    Validate detection coordinates and confidence against acceptance criteria.

    Uses relaxed thresholds to maximize detection continuity while rejecting
    obvious edge cases and low-confidence detections.

    Validation Rules:
        - Reject detections in outer 5-10% frame edges (often partial/clipped)
        - Require minimum 30% confidence from YOLO

    Args:
        center_x: Normalized X coordinate (0.0 = left, 1.0 = right)
        center_y: Normalized Y coordinate (0.0 = top, 1.0 = bottom)
        confidence: YOLO detection confidence (0.0 - 1.0)

    Returns:
        True if detection passes all validation checks
    """
    # Vertical edge rejection (10% margins)
    if center_y < 0.10 or center_y > 0.90:
        return False

    # Horizontal edge rejection (5% margins - tighter for pan tracking)
    if center_x < 0.05 or center_x > 0.95:
        return False

    # Minimum confidence threshold
    if confidence < 0.30:
        return False

    return True


# =============================================================================
# FRAME ANNOTATION
# =============================================================================

def annotate_frame(frame, results, model, width: int, height: int,
                   person_candidates: list, current_detection: dict) -> any:
    """
    Draw detection overlays and status information on video frame.

    Renders bounding boxes, center crosshair, deadband zone, valid detection
    area, and tracking status for the Flask video stream.

    Visual Elements:
        - Green boxes: Valid person detections
        - Red boxes: Rejected detections (edge/low confidence)
        - Yellow boxes: Non-person objects
        - Blue crosshair: Frame center (tracking target)
        - Gray rectangle: Deadband zone (no movement needed)
        - Orange rectangle: Valid detection zone boundaries

    Args:
        frame: OpenCV BGR image (camera capture)
        results: YOLO inference results
        model: YOLO model instance (for class names)
        width: Frame width in pixels
        height: Frame height in pixels
        person_candidates: List of valid person detections this frame
        current_detection: Currently tracked detection (or None)

    Returns:
        Annotated OpenCV BGR image
    """
    # Mirror frame horizontally (camera is mounted inverted)
    annotated = cv2.flip(frame.copy(), 1)

    # Draw all detection bounding boxes
    for box in results[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

        # Mirror coordinates to match flipped frame
        x1_mirrored = width - x1
        x2_mirrored = width - x2
        x1, x2 = min(x1_mirrored, x2_mirrored), max(x1_mirrored, x2_mirrored)

        cls = int(box.cls[0])
        conf = float(box.conf[0])
        label = model.names[cls]

        # Compute normalized center position
        center_x = ((x1 + x2) / 2) / width
        center_y = ((y1 + y2) / 2) / height

        # Color code by detection validity
        if label == 'person':
            # Invert X for validation (matching detector logic)
            is_valid = is_valid_detection(1.0 - center_x, center_y, conf)
            color = (0, 255, 0) if is_valid else (0, 0, 255)  # Green valid, red invalid
            thickness = 3 if is_valid else 2
        else:
            color = (0, 255, 255)  # Yellow for non-person objects
            thickness = 1

        cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)

        label_text = f"{label} {conf:.2f}"
        cv2.putText(annotated, label_text, (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # Draw center crosshair (tracking reference point)
    cx, cy = width // 2, height // 2
    cv2.line(annotated, (cx - 30, cy), (cx + 30, cy), (255, 0, 0), 2)
    cv2.line(annotated, (cx, cy - 30), (cx, cy + 30), (255, 0, 0), 2)
    cv2.circle(annotated, (cx, cy), 5, (255, 0, 0), -1)

    # Draw deadband zone (8% of frame width)
    deadband = int(0.08 * width)
    cv2.rectangle(annotated,
                  (cx - deadband, cy - deadband),
                  (cx + deadband, cy + deadband),
                  (128, 128, 128), 2)

    # Draw valid detection zone boundaries
    valid_zone = {
        'top': int(0.15 * height),
        'bottom': int(0.85 * height),
        'left': int(0.10 * width),
        'right': int(0.90 * width)
    }
    cv2.rectangle(annotated,
                  (valid_zone['left'], valid_zone['top']),
                  (valid_zone['right'], valid_zone['bottom']),
                  (255, 128, 0), 1)

    # Draw tracking status overlay
    if current_detection:
        stable_text = " (STABLE)" if current_detection.get('stable', False) else " (TRACKING)"
        status = f"TRACKING{stable_text}: ({current_detection['x']:.2f}, {current_detection['y']:.2f})"
        color = (0, 255, 0)
    elif person_candidates:
        status = f"SEARCHING: {len(person_candidates)} candidate(s)"
        color = (0, 255, 255)
    else:
        status = "SCANNING: No person"
        color = (0, 0, 255)

    cv2.putText(annotated, status, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(annotated, f"Objects: {len(results[0].boxes)}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return annotated


# =============================================================================
# FLASK VIDEO STREAMING
# =============================================================================

def generate_frames():
    """
    Generator function for Flask MJPEG video streaming.

    Yields JPEG-encoded frames in multipart format for browser consumption.
    Thread-safe access to the global frame buffer ensures clean handoff
    from the ROS2 detection loop.

    Yields:
        Multipart JPEG frame data with appropriate HTTP headers
    """
    global latest_frame

    while True:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.1)
                continue
            frame = latest_frame.copy()

        # Encode frame as JPEG with quality optimization
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/')
def index():
    """Flask route: Render HTML page with embedded video stream."""
    return """
    <html>
        <head>
            <title>Two Towers Detector</title>
            <style>
                body { margin: 0; padding: 20px; background: #1a1a1a; text-align: center; }
                h1 { color: #fff; font-family: Arial; }
                img { max-width: 95vw; max-height: 85vh; border: 2px solid #444; }
                .info { color: #aaa; font-family: monospace; margin-top: 10px; }
            </style>
        </head>
        <body>
            <h1>Two Towers Detector</h1>
            <img src="/video_feed" />
            <div class="info">
                <p>Green = Valid | Red = Rejected | Blue Crosshair = Aim</p>
                <p>Gray = Deadband | Orange = Valid zone</p>
            </div>
        </body>
    </html>
    """


@app.route('/video_feed')
def video_feed():
    """Flask route: MJPEG video stream endpoint."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# =============================================================================
# ROS2 DETECTOR NODE
# =============================================================================

class TwoTowersDetectorNode(Node):
    """
    ROS2 node for YOLOv8-based person detection and coordinate publishing.

    Captures camera frames, runs YOLO inference, applies temporal smoothing,
    and publishes detection coordinates for the tracking system. Optionally
    runs a Flask server for live video monitoring.

    Detection Pipeline:
        1. Capture frame from USB camera
        2. Flip frame (camera mounted upside-down)
        3. Run YOLOv8 inference at reduced resolution for speed
        4. Extract person detections and validate coordinates
        5. Apply temporal smoothing via RelaxedTracker
        6. Publish smoothed coordinates (or empty array if no target)

    ROS2 Interface:
        Publishers:
            - /{tower_id}/detections (DetectionArray)

        Parameters:
            - enable_streaming (bool): Enable Flask video server
            - stream_port (int): Flask server port number
            - tower_id (string): Tower identifier for multi-agent setup

    Attributes:
        model: YOLOv8 model instance
        cap: OpenCV VideoCapture for camera
        tracker: RelaxedTracker for temporal smoothing
        publisher_: ROS2 publisher for detections
    """

    # Detection loop frequency (Hz)
    DETECTION_RATE_HZ = 15.0

    # Camera resolution (lower = faster inference)
    CAMERA_WIDTH = 320
    CAMERA_HEIGHT = 240

    # YOLO inference settings
    YOLO_CONFIDENCE_THRESHOLD = 0.25
    YOLO_INPUT_SIZE = 160

    def __init__(self):
        """Initialize detector node with camera, YOLO model, and ROS2 interfaces."""
        super().__init__('two_towers_detector')

        # Declare ROS2 parameters
        self.declare_parameter('enable_streaming', True)
        self.declare_parameter('stream_port', 5000)
        self.declare_parameter('tower_id', 'tower_a')

        self.enable_streaming = self.get_parameter('enable_streaming').value
        self.stream_port = self.get_parameter('stream_port').value
        self.tower_id = self.get_parameter('tower_id').value

        # Create detection publisher with tower-specific topic
        topic_name = f'{self.tower_id}/detections'
        self.publisher_ = self.create_publisher(
            DetectionArray,
            topic_name,
            10  # QoS queue depth
        )
        self.get_logger().info(f'Publishing detections to: {topic_name}')

        # Initialize YOLO model
        self.get_logger().info('Loading YOLOv8n model...')
        try:
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise

        # Initialize camera capture
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera device")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.CAMERA_HEIGHT)

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Camera initialized: {self.width}x{self.height}')

        # Initialize temporal smoothing tracker
        self.tracker = RelaxedTracker(buffer_size=3)

        # Frame counter for periodic logging
        self.start_time = time.time()
        self.frame_count = 0

        # Create detection timer at specified rate
        timer_period = 1.0 / self.DETECTION_RATE_HZ
        self.timer = self.create_timer(timer_period, self.process_frame)

        # Start Flask streaming server if enabled
        if self.enable_streaming:
            self._start_flask_server()

        self.get_logger().info(f'Detector node ready (tower_id={self.tower_id})')

    def _start_flask_server(self):
        """Start Flask video streaming server in background thread."""
        import socket
        hostname = socket.gethostname()
        try:
            local_ip = socket.gethostbyname(hostname)
        except socket.gaierror:
            local_ip = '127.0.0.1'

        self.get_logger().info(f'Starting video stream: http://{local_ip}:{self.stream_port}')

        self.flask_thread = threading.Thread(
            target=lambda: app.run(
                host='0.0.0.0',
                port=self.stream_port,
                threaded=True,
                debug=False,
                use_reloader=False
            ),
            daemon=True
        )
        self.flask_thread.start()

    def process_frame(self):
        """
        Main detection loop callback - runs at DETECTION_RATE_HZ.

        Captures frame, runs inference, extracts person detections,
        applies smoothing, and publishes results.
        """
        global latest_frame

        # Capture frame from camera
        ret, frame = self.cap.read()
        if not ret:
            return

        # Flip frame (camera is mounted upside-down)
        frame = cv2.flip(frame, -1)

        # Run YOLO inference with optimized settings
        results = self.model(
            frame,
            verbose=False,
            conf=self.YOLO_CONFIDENCE_THRESHOLD,
            imgsz=self.YOLO_INPUT_SIZE
        )

        # Extract valid person detections
        person_candidates = []

        for box in results[0].boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            label = self.model.names[cls]

            if label != 'person':
                continue

            # Compute normalized center coordinates
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            center_x = ((x1 + x2) / 2) / self.width
            center_y = ((y1 + y2) / 2) / self.height

            # Validate detection position and confidence
            if not is_valid_detection(center_x, center_y, conf):
                continue

            person_candidates.append({
                'label': label,
                'confidence': float(conf),
                'x': float(center_x),
                'y': float(center_y)
            })

        # Select best detection and apply smoothing
        current_detection = None

        if person_candidates:
            # Score candidates by confidence and proximity to center
            # Preference: high confidence, close to center
            best = max(
                person_candidates,
                key=lambda d: d['confidence'] / (1.0 + ((d['x'] - 0.5)**2 + (d['y'] - 0.5)**2))
            )
            current_detection = self.tracker.add_detection(best)
        else:
            self.tracker.clear()

        # Update Flask stream frame
        if self.enable_streaming:
            annotated = annotate_frame(
                frame, results, self.model, self.width,
                self.height, person_candidates, current_detection
            )
            with frame_lock:
                latest_frame = annotated

        # Publish detection or empty array
        if current_detection:
            self._publish_detection(current_detection)
        else:
            self._publish_empty_detection()

        # Periodic logging
        self.frame_count += 1
        if self.frame_count % 20 == 0:
            if current_detection:
                stable = " (stable)" if current_detection.get('stable', False) else ""
                self.get_logger().info(
                    f"Person{stable} @ ({current_detection['x']:.3f}, {current_detection['y']:.3f})"
                )
            else:
                self.get_logger().info("No person detected")

    def _publish_detection(self, det: dict):
        """
        Publish single person detection to ROS2 topic.

        Args:
            det: Detection dict with 'label', 'confidence', 'x', 'y' keys
        """
        msg = DetectionArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.tower_id}_camera'

        detection = Detection()
        detection.label = det['label']
        detection.confidence = det['confidence']
        detection.x = det['x']
        detection.y = det['y']
        detection.timestamp_ms = int(time.time() * 1000)
        msg.detections.append(detection)

        self.publisher_.publish(msg)

    def _publish_empty_detection(self):
        """
        Publish empty detection array to signal 'no target' to tracker.

        The tracker node uses empty arrays to trigger scanning behavior.
        """
        msg = DetectionArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.tower_id}_camera'
        # Empty detections array signals "no person found"
        self.publisher_.publish(msg)

    def __del__(self):
        """Cleanup: release camera on node destruction."""
        if hasattr(self, 'cap'):
            self.cap.release()


# =============================================================================
# ENTRY POINT
# =============================================================================

def main(args=None):
    """ROS2 node entry point."""
    rclpy.init(args=args)

    try:
        node = TwoTowersDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
