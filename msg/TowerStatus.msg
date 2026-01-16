# TowerStatus.msg - Tower state message for multi-agent coordination
#
# Published by each tower's tracker node to enable:
#   - Cooperative tracking (both towers track same target)
#   - Seamless handoff (transfer tracking responsibility)
#   - Coverage optimization (coordinate scanning patterns)
#   - Fault tolerance (detect tower failures)
#
# Topic: /tower_a/status, /tower_b/status
# Publisher: Tracker node for each tower
# Subscribers: Coordinator node, other tower tracker nodes

# ROS2 header with timestamp for synchronization
std_msgs/Header header

# Tower identification
string tower_id                          # Unique tower ID ("tower_a", "tower_b", etc.)

# Servo positions (degrees)
float64 pan_angle                        # Current pan servo angle (0-180)
float64 tilt_angle                       # Current tilt servo angle (0-180)

# Operational state
# Values: "scanning", "tracking", "locked", "handoff_requesting", "handoff_receiving"
string state

# Target tracking information
bool has_target                          # True if currently tracking a target
geometry_msgs/Point target_position      # Target position in normalized camera coords (0-1)
float64 target_confidence                # Detection confidence (0-1)

# Velocity estimation for predictive handoff
float64 target_velocity_x                # Estimated X velocity (normalized units/sec)
float64 target_velocity_y                # Estimated Y velocity (normalized units/sec)

# Handoff coordination
bool handoff_requested                   # True if requesting handoff to another tower
string handoff_target_tower              # ID of tower to hand off to (if requesting)
float64 time_to_edge                     # Estimated seconds until target leaves FOV

# Health/diagnostics
float64 tracking_confidence              # Overall tracking quality (0-1)
int32 frames_since_detection             # Frames since last valid detection
