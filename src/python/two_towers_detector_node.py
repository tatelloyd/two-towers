#!/usr/bin/env python3
"""
Orthanc Detector Node - YOLO with RELAXED stability for real-time tracking
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

app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()

class RelaxedTracker:
    """Lightweight tracker - publishes quickly with minimal filtering"""
    def __init__(self, buffer_size=3):
        self.detections = deque(maxlen=buffer_size)
        self.last_position = None
        self.frames_since_detection = 0
        
    def add_detection(self, detection):
        """Add detection and return smoothed position"""
        self.detections.append(detection)
        self.frames_since_detection = 0
        
        # Publish immediately after just 1 detection
        if len(self.detections) < 1:
            return None
        
        # Simple averaging over last N frames
        recent = list(self.detections)
        avg_x = sum(d['x'] for d in recent) / len(recent)
        avg_y = sum(d['y'] for d in recent) / len(recent)
        
        # Only reject MASSIVE jumps (likely false positives or new person)
        if self.last_position:
            prev_x = self.last_position['x']
            prev_y = self.last_position['y']
            jump = ((avg_x - prev_x)**2 + (avg_y - prev_y)**2)**0.5
            
            # Very permissive threshold - only reject teleportation
            if jump > 0.6:
                return None
        
        result = {
            'label': 'person',
            'confidence': sum(d['confidence'] for d in recent) / len(recent),
            'x': avg_x,
            'y': avg_y,
            'stable': len(recent) >= 2
        }
        
        self.last_position = result
        return result
    
    def clear(self):
        self.detections.clear()
        self.last_position = None  # Clear ghost detections
        self.frames_since_detection += 1  # CRITICAL: Clear ghost detections!


def is_valid_detection(center_x, center_y, conf):
    """VERY RELAXED validation - maximize detection continuity"""
    # Minimal edge rejection - only reject extreme edges
    if center_y < 0.10 or center_y > 0.90:
        return False
    if center_x < 0.05 or center_x > 0.95:
        return False
    # Lower confidence - trust YOLO more
    if conf < 0.30:
        return False
    return True


def annotate_frame(frame, results, model, width, height, person_candidates, current_detection):
    """Draw camera information"""
    annotated = cv2.flip(frame.copy(), 1)
    
    # Draw all detections
    for box in results[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        
        x1_mirrored = width - x1
        x2_mirrored = width - x2
        x1, x2 = min(x1_mirrored, x2_mirrored), max(x1_mirrored, x2_mirrored)
        
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        label = model.names[cls]
        
        center_x = ((x1 + x2) / 2) / width
        center_y = ((y1 + y2) / 2) / height
        
        if label == 'person':
            is_valid = is_valid_detection(1.0 - center_x, center_y, conf)
            color = (0, 255, 0) if is_valid else (0, 0, 255)
            thickness = 3 if is_valid else 2
        else:
            color = (0, 255, 255)
            thickness = 1
        
        cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
        
        label_text = f"{label} {conf:.2f}"
        cv2.putText(annotated, label_text, (int(x1), int(y1)-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    # Center crosshair
    cx, cy = width // 2, height // 2
    cv2.line(annotated, (cx - 30, cy), (cx + 30, cy), (255, 0, 0), 2)
    cv2.line(annotated, (cx, cy - 30), (cx, cy + 30), (255, 0, 0), 2)
    cv2.circle(annotated, (cx, cy), 5, (255, 0, 0), -1)
    
    # Deadband (wider now)
    deadband = int(0.08 * width)
    cv2.rectangle(annotated, 
                  (cx - deadband, cy - deadband),
                  (cx + deadband, cy + deadband),
                  (128, 128, 128), 2)
    
    # Valid zone
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
    
    # Status
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


def generate_frames():
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.1)
                continue
            frame = latest_frame.copy()
        
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            continue
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/')
def index():
    return """
    <html>
        <head>
            <title>Orthanc Detector</title>
            <style>
                body { margin: 0; padding: 20px; background: #1a1a1a; text-align: center; }
                h1 { color: #fff; font-family: Arial; }
                img { max-width: 95vw; max-height: 85vh; border: 2px solid #444; }
                .info { color: #aaa; font-family: monospace; margin-top: 10px; }
            </style>
        </head>
        <body>
            <h1>🗼 Orthanc Detector</h1>
            <img src="/video_feed" />
            <div class="info">
                <p>🟢 Green = Valid | 🔴 Red = Rejected | 🔵 Crosshair = Aim</p>
                <p>⬜ Gray = Deadband | 🟧 Orange = Valid zone</p>
            </div>
        </body>
    </html>
    """


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


class OrthancDetectorNode(Node):
    def __init__(self):
        super().__init__('orthanc_detector')
        
        self.declare_parameter('enable_streaming', True)
        self.declare_parameter('stream_port', 5000)
        
        self.enable_streaming = self.get_parameter('enable_streaming').value
        self.stream_port = self.get_parameter('stream_port').value
        
        # Publisher
        self.publisher_ = self.create_publisher(
            DetectionArray,
            'orthanc/detections',
            10
        )
        
        # Load YOLO
        self.get_logger().info('Loading YOLOv8n...')
        try:
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info('✅ YOLO loaded')
        except Exception as e:
            self.get_logger().error(f'❌ YOLO failed: {e}')
            raise
        
        # Camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Camera failed")
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.get_logger().info(f'✅ Camera: {self.width}x{self.height}')
        
        # RELAXED tracker - now averages over 3 frames for stability
        self.tracker = RelaxedTracker(buffer_size=3)
        
        # NO warmup - publish immediately
        self.start_time = time.time()
        
        # Timer at 15Hz
        self.timer = self.create_timer(1.0/15.0, self.process_frame)
        self.frame_count = 0
        
        # Flask streaming
        if self.enable_streaming:
            import socket
            hostname = socket.gethostname()
            local_ip = socket.gethostbyname(hostname)
            self.get_logger().info(f'🌐 Stream: http://{local_ip}:{self.stream_port}')
            
            self.flask_thread = threading.Thread(
                target=lambda: app.run(host='0.0.0.0', port=self.stream_port, 
                                      threaded=True, debug=False, use_reloader=False),
                daemon=True
            )
            self.flask_thread.start()
        
        self.get_logger().info('🗼 Detector running (relaxed mode)')
    
    def process_frame(self):
        global latest_frame
        
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # Flip (camera upside down)
        frame = cv2.flip(frame, -1)
        
        # YOLO detection with lower confidence threshold
        results = self.model(frame, verbose=False, conf=0.25, imgsz=160)
        
        # Process persons
        person_candidates = []
        
        for box in results[0].boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            label = self.model.names[cls]
            
            if label != 'person':
                continue
            
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            center_x = ((x1 + x2) / 2) / self.width
            center_y = ((y1 + y2) / 2) / self.height
            
            if not is_valid_detection(center_x, center_y, conf):
                continue
            
            person_candidates.append({
                'label': label,
                'confidence': float(conf),
                'x': float(center_x),
                'y': float(center_y)
            })
        
        # Select best and track
        current_detection = None
        
        if person_candidates:
            best = max(person_candidates, key=lambda d: 
                d['confidence'] / (1.0 + ((d['x']-0.5)**2 + (d['y']-0.5)**2))
            )
            
            current_detection = self.tracker.add_detection(best)
        else:
            self.tracker.clear()
        
        # Stream
        if self.enable_streaming:
            annotated = annotate_frame(frame, results, self.model, self.width, 
                                      self.height, person_candidates, current_detection)
            with frame_lock:
                latest_frame = annotated
        
        # ALWAYS publish if we have ANY detection
        # But ADD TIMEOUT: don't publish stale detections
        if current_detection:
            self.publish_detection(current_detection)
        else:
            # Publish empty array when no person detected
            # This tells C++ tracker to start scanning immediately
            self.publish_empty_detection()
        
        # Log
        self.frame_count += 1
        if self.frame_count % 20 == 0:
            if current_detection:
                stable = " (stable)" if current_detection.get('stable', False) else ""
                self.get_logger().info(
                    f"👤 Person{stable} @ ({current_detection['x']:.3f}, {current_detection['y']:.3f})"
                )
            else:
                self.get_logger().info("🔍 No person")
    
    def publish_detection(self, det):
        """Publish single detection"""
        msg = DetectionArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'orthanc_camera'
        
        detection = Detection()
        detection.label = det['label']
        detection.confidence = det['confidence']
        detection.x = det['x']
        detection.y = det['y']
        detection.timestamp_ms = int(time.time() * 1000)
        msg.detections.append(detection)
        
        self.publisher_.publish(msg)
    
    def publish_empty_detection(self):
        """Publish empty array to signal 'no target' to C++ tracker"""
        msg = DetectionArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'orthanc_camera'
        # Empty detections array signals "no person found"
        self.publisher_.publish(msg)
    
    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OrthancDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()