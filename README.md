# Two Towers - Autonomous Tracking System

> **Status**: Single-tower tracking operational. Multi-tower coordination architecture planned.

A high-performance autonomous person tracking system featuring YOLOv8 computer vision, behavior tree control architecture, and real-time servo actuation. Built for Raspberry Pi 4 with hardware PWM control.

[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%204-red)]()
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue)]()
[![Python](https://img.shields.io/badge/Python-3.9%2B-blue)]()
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green)]()

---

## Project Overview

Two Towers demonstrates core principles of autonomous systems engineering:

- **Behavior Tree Architecture**: Modular decision-making framework enabling reactive autonomy
- **Real-Time Vision Pipeline**: YOLOv8 object detection at 15 Hz with ROS2 message passing
- **Hardware PWM Control**: Jitter-free servo actuation via pigpio daemon (50 Hz control loop)
- **Embedded Linux Deployment**: Headless operation on resource-constrained hardware
- **Scalable Design**: Architecture designed for multi-agent extension

### Current Capabilities

- Autonomous person tracking with proportional control
- Real-time YOLOv8n detection optimized for Raspberry Pi
- Smooth servo control with adaptive gains and deadband
- Scanning behavior when target is lost
- Flask-based video streaming for remote monitoring
- ROS2-based communication between detector and tracker

### Planned Features

- Multi-turret coordination with seamless handoff
- Cooperative tracking from multiple viewpoints
- Predictive tracking with Kalman filtering
- AI-powered decision making via Claude API

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    VISION PIPELINE (Python)                 │
│  ┌──────────┐      ┌──────────┐      ┌────────────────┐    │
│  │  USB Cam │─────▶│ YOLOv8n  │─────▶│  ROS2 Topic    │    │
│  │  320x240 │      │  15 Hz   │      │  /detections   │    │
│  └──────────┘      └──────────┘      └────────┬───────┘    │
│                                               │            │
│  Optional: Flask video stream on port 5000    │            │
└───────────────────────────────────────────────┼────────────┘
                                                │
                                                ▼
┌───────────────────────────────────────────────┼────────────┐
│              CONTROL SYSTEM (C++)             │            │
│  ┌────────────────────────────────────────────▼───────┐   │
│  │           Tracker Node (20 Hz)                     │   │
│  │  ┌─────────────────────────────────────────────┐   │   │
│  │  │  State Machine: TRACKING <──> SCANNING      │   │   │
│  │  │                                             │   │   │
│  │  │  Proportional Control:                      │   │   │
│  │  │    error = target_pos - center              │   │   │
│  │  │    adjustment = clamp(error * gain, max)    │   │   │
│  │  └─────────────────────────────────────────────┘   │   │
│  └────────────────────┬───────────────────────────────┘   │
│                       ▼                                    │
│  ┌─────────────────────────────────────────────────────┐  │
│  │              Turret Hardware Layer                   │  │
│  │  ┌──────────────────┐    ┌──────────────────┐       │  │
│  │  │ ServoController  │    │ ServoController  │       │  │
│  │  │   (Pan/GPIO 17)  │    │  (Tilt/GPIO 27)  │       │  │
│  │  └────────┬─────────┘    └─────────┬────────┘       │  │
│  └───────────┼────────────────────────┼────────────────┘  │
│              ▼                        ▼                    │
│       ┌──────────┐              ┌──────────┐              │
│       │ Pan Servo│              │Tilt Servo│              │
│       │  SG90    │              │  SG90    │              │
│       └──────────┘              └──────────┘              │
└─────────────────────────────────────────────────────────────┘
```

---

## Hardware Requirements

| Component | Specifications | Notes |
|-----------|---------------|-------|
| **Raspberry Pi 4** | 4GB+ RAM | Main controller |
| **Pan/Tilt Mount** | [SparkFun ROB-14045](https://www.sparkfun.com/products/14045) | Includes 2x SG90 servos |
| **USB Webcam** | 320x240 @ 15fps | Any UVC-compatible camera |
| **Power Supply** | 5V 3A USB-C | Official RPi PSU recommended |
| **MicroSD Card** | 32GB+ Class 10 | Raspberry Pi OS (Debian 12) |

---

## Quick Start

### Prerequisites

```bash
# System dependencies
sudo apt-get update && sudo apt-get install -y \
    cmake libpigpio-dev pigpio python3-pip \
    nlohmann-json3-dev

# Enable pigpiod daemon (required for hardware PWM)
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# ROS2 Humble (if not installed)
# Follow: https://docs.ros.org/en/humble/Installation.html
```

### Installation

```bash
git clone https://github.com/tatelloyd/two-towers.git
cd two-towers

# Python environment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Build ROS2 package
colcon build
source install/setup.bash
```

### Running the System

**Terminal 1 - Vision Pipeline:**
```bash
source venv/bin/activate
source install/setup.bash
ros2 run two_towers two_towers_detector_node.py
```

**Terminal 2 - Tracker Node:**
```bash
source install/setup.bash
ros2 run two_towers orthanc_tracker_node
```

The turret will:
1. Scan back and forth searching for a person
2. Lock onto detected person and track
3. Apply proportional control to keep target centered
4. Resume scanning if target is lost

**Video Stream**: Open `http://<raspberry-pi-ip>:5000` in a browser

**Stop**: Press `Ctrl+C` in either terminal (servos auto-center on shutdown)

---

## Control Algorithm

### Proportional Tracking

The tracker uses adaptive proportional control with error-dependent gains:

```cpp
// Error from frame center (normalized 0-1 coordinates)
double x_error = -(target_x - 0.5);  // Inverted for camera orientation
double y_error = -(target_y - 0.5);

// Adaptive gain selection
double gain = |error| > 0.15 ? 8.0 :    // Large error: fast response
              |error| > 0.08 ? 4.0 :    // Medium error: moderate
                               2.0;      // Small error: fine tuning

// Apply with deadband and saturation
if (|error| < 0.05) error = 0;          // 5% deadband
double adjustment = clamp(error * gain, -2.0, 2.0);  // Max 2° per tick
```

### State Machine

```
┌─────────────┐         Target Found         ┌─────────────┐
│  SCANNING   │ ───────────────────────────▶ │  TRACKING   │
│             │                              │             │
│  Sweep pan  │ ◀─────────────────────────── │  Center on  │
│  left/right │      Target Lost (60 frames) │  target     │
└─────────────┘                              └─────────────┘
```

---

## Project Structure

```
two-towers/
├── src/
│   ├── cpp/
│   │   ├── orthanc_tracker_node.cpp  # ROS2 tracker (simple version)
│   │   ├── bt_nodes.hpp              # Behavior tree nodes
│   │   ├── Turret.{cpp,hpp}          # Pan/tilt turret controller
│   │   ├── ServoController.{cpp,hpp} # PWM servo abstraction
│   │   └── main.cpp                  # Standalone tracker (file IPC)
│   └── python/
│       └── two_towers_detector_node.py  # YOLO detection + Flask
├── msg/
│   ├── Detection.msg                 # Single detection
│   ├── DetectionArray.msg            # Array of detections
│   └── TurretState.msg               # Tower status (for multi-agent)
├── config/
│   └── turret_config.json            # Hardware configuration
├── tests/
│   ├── cpp/menu.cpp                  # Interactive servo testing
│   └── python/test_*.py              # Vision pipeline tests
├── CMakeLists.txt                    # ROS2 build configuration
├── package.xml                       # ROS2 package manifest
└── requirements.txt                  # Python dependencies
```

---

## Configuration

Key parameters can be tuned in the source files:

### Tracking Control (`orthanc_tracker_node.cpp`)
```cpp
static constexpr double DEADBAND = 0.05;           // 5% center tolerance
static constexpr double GAIN_LARGE = 8.0;          // Fast acquisition
static constexpr double GAIN_FINE = 2.0;           // Fine centering
static constexpr double MAX_ADJUSTMENT = 2.0;      // Max degrees per tick
static constexpr int SCAN_THRESHOLD_FRAMES = 60;   // Frames before scanning
```

### Detection (`two_towers_detector_node.py`)
```python
DETECTION_RATE_HZ = 15.0          # Detection frequency
YOLO_CONFIDENCE_THRESHOLD = 0.25  # Minimum detection confidence
YOLO_INPUT_SIZE = 160             # Model input (smaller = faster)
```

### Hardware (GPIO pins)
- **Pan Servo**: GPIO 17
- **Tilt Servo**: GPIO 27
- **PWM Range**: 500-2500 μs (0°-180°)

---

## Troubleshooting

### Servo not moving
```bash
# Check pigpiod is running
sudo systemctl status pigpiod

# Restart if needed
sudo systemctl restart pigpiod
```

### Camera not detected
```bash
# List video devices
ls /dev/video*

# Test camera
python3 -c "import cv2; print(cv2.VideoCapture(0).isOpened())"
```

### ROS2 topics not connecting
```bash
# List active topics
ros2 topic list

# Echo detection messages
ros2 topic echo /tower_a/detections
```

---

## Development Roadmap

### Phase 1: Single-Tower (Complete)
- [x] Behavior tree framework
- [x] Real-time YOLO detection
- [x] Proportional tracking control
- [x] ROS2 integration
- [x] Video streaming

### Phase 2: Multi-Tower (Planned)
- [ ] Deploy second turret hardware
- [ ] Tower status publishing
- [ ] Coordinator node for handoff
- [ ] Cooperative tracking mode

### Phase 3: Advanced Features (Future)
- [ ] Kalman filter motion prediction
- [ ] AI-powered decision making
- [ ] Multi-person tracking

---

## License

MIT License - See LICENSE file for details

---

## Author

**Tate Lloyd**
Robotics & Embedded Systems Engineer

- GitHub: [@tatelloyd](https://github.com/tatelloyd)
- LinkedIn: [/in/tatelloyd](https://www.linkedin.com/in/tatelloyd/)
- Email: tate.lloyd@yale.edu

---

*Built to demonstrate real-time autonomy, embedded control systems, and robotics software engineering.*
