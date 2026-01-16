/**
 * @file orthanc_tracker_node.cpp
 * @brief ROS2 Tracker Node - Simple Proportional Control Implementation
 *
 * A lightweight ROS2 node that subscribes to person detection messages and
 * controls a pan/tilt turret to keep the detected person centered in frame.
 * Uses proportional control with adaptive gains for smooth, responsive tracking.
 *
 * @section Architecture
 *
 *     Detection Topic           Control Output
 *           |                        |
 *           v                        v
 *    +--------------+         +-------------+
 *    |   Tracker    |-------->|   Turret    |
 *    |    Node      |         |  Hardware   |
 *    +--------------+         +-------------+
 *           |
 *           v
 *    State Machine:
 *    [TRACKING] <--> [SCANNING]
 *
 * @section Control Algorithm
 *
 * The tracker uses proportional control with adaptive gains:
 *
 *    error = target_position - center (0.5, 0.5)
 *    gain = f(|error|)  // Higher gain for larger errors
 *    adjustment = clamp(error * gain, max_adjustment)
 *    new_angle = current_angle + adjustment
 *
 * Adaptive gain schedule:
 *    |error| > 0.15: gain = 8.0  (fast acquisition)
 *    |error| > 0.08: gain = 4.0  (medium tracking)
 *    |error| <= 0.08: gain = 2.0 (fine centering)
 *
 * @section ROS2 Interface
 *
 * Subscriptions:
 *    - /tower_a/detections (two_towers/msg/DetectionArray)
 *
 * @author Tate Lloyd <tate.lloyd@yale.edu>
 * @license MIT
 */

#include <rclcpp/rclcpp.hpp>
#include <two_towers/msg/detection_array.hpp>
#include "Turret.hpp"
#include <memory>
#include <cmath>

/**
 * @class TwoTowersTrackerNode
 * @brief ROS2 node for proportional person tracking with pan/tilt turret
 *
 * Implements a two-state controller:
 *   1. TRACKING: Person detected - apply proportional control to center target
 *   2. SCANNING: No person detected - sweep pan axis to search for targets
 *
 * The node automatically transitions between states based on detection
 * continuity, with a configurable timeout before entering scan mode.
 */
class TwoTowersTrackerNode : public rclcpp::Node {
public:
    //=========================================================================
    // CONTROL PARAMETERS
    //=========================================================================

    // Proportional gains for different error magnitudes (degrees per normalized unit)
    static constexpr double GAIN_LARGE = 8.0;   // For |error| > 0.15
    static constexpr double GAIN_MEDIUM = 4.0;  // For |error| > 0.08
    static constexpr double GAIN_FINE = 2.0;    // For |error| <= 0.08

    // Error thresholds for gain selection
    static constexpr double ERROR_THRESHOLD_LARGE = 0.15;
    static constexpr double ERROR_THRESHOLD_MEDIUM = 0.08;

    // Deadband: don't move if target is within this distance from center
    static constexpr double DEADBAND = 0.05;

    // Maximum adjustment per control cycle (degrees)
    static constexpr double MAX_ADJUSTMENT = 2.0;

    // Servo angle limits (degrees)
    static constexpr double SERVO_MIN_ANGLE = 10.0;
    static constexpr double SERVO_MAX_ANGLE = 170.0;

    // Frames without detection before entering scan mode
    static constexpr int SCAN_THRESHOLD_FRAMES = 60;

    // Scanning parameters
    static constexpr double SCAN_LEFT_LIMIT = 30.0;
    static constexpr double SCAN_RIGHT_LIMIT = 150.0;
    static constexpr double SCAN_STEP = 15.0;

    //=========================================================================
    // CONSTRUCTOR / DESTRUCTOR
    //=========================================================================

    /**
     * @brief Construct tracker node with hardware initialization
     *
     * Initializes:
     *   - Pan/tilt turret on GPIO pins 17 (pan) and 27 (tilt)
     *   - ROS2 subscription to detection topic
     *   - Turret centered at Pan=90, Tilt=30 (slightly downward)
     */
    TwoTowersTrackerNode()
        : Node("two_towers_tracker"),
          turret_(std::make_unique<Turret>(17, 27)),
          frames_without_detection_(0),
          tracking_active_(false),
          last_scan_direction_(1)
    {
        // Subscribe to detection messages
        subscription_ = this->create_subscription<two_towers::msg::DetectionArray>(
            "tower_a/detections",
            10,
            std::bind(&TwoTowersTrackerNode::detection_callback, this, std::placeholders::_1)
        );

        // Initialize turret to starting position
        turret_->setPanAngle(90.0);   // Center horizontally
        turret_->setTiltAngle(30.0);  // Slight downward angle

        RCLCPP_INFO(this->get_logger(), "Two Towers tracker started");
        RCLCPP_INFO(this->get_logger(), "  Turret initialized: Pan=90, Tilt=30");
        RCLCPP_INFO(this->get_logger(), "  Listening for detections on: tower_a/detections");
    }

    /**
     * @brief Destructor - safely center turret before shutdown
     */
    ~TwoTowersTrackerNode() {
        RCLCPP_INFO(this->get_logger(), "Centering turret before shutdown...");
        turret_->centerAll();
    }

private:
    //=========================================================================
    // ROS2 CALLBACK
    //=========================================================================

    /**
     * @brief Process incoming detection messages
     *
     * Called for each DetectionArray message. Selects the best person
     * detection (if any) and either tracks it or enters scanning mode.
     *
     * Target Selection:
     *   score = confidence / (1 + distance_from_center)
     *   Prioritizes high-confidence, centered detections
     *
     * @param msg Detection array from vision node
     */
    void detection_callback(const two_towers::msg::DetectionArray::SharedPtr msg) {
        // Find best person detection using score-based selection
        const two_towers::msg::Detection* best_person = nullptr;
        double best_score = -1.0;

        for (const auto& det : msg->detections) {
            if (det.label == "person") {
                // Score formula: confidence / (1 + distance_from_center)
                // Higher score = better candidate
                double dist_from_center = std::sqrt(
                    std::pow(det.x - 0.5, 2) + std::pow(det.y - 0.5, 2)
                );
                double score = det.confidence / (1.0 + dist_from_center);

                if (score > best_score) {
                    best_score = score;
                    best_person = &det;
                }
            }
        }

        if (best_person != nullptr) {
            // Person detected - engage tracking
            track_target(*best_person);
            frames_without_detection_ = 0;
            tracking_active_ = true;
        } else {
            // No person detected - increment dropout counter
            frames_without_detection_++;

            if (frames_without_detection_ > SCAN_THRESHOLD_FRAMES) {
                // Lost target - enter scan mode
                if (tracking_active_) {
                    RCLCPP_INFO(this->get_logger(), "Target lost - entering scan mode");
                    tracking_active_ = false;
                }
                scan_for_target();
            }
            // Brief dropout: hold position (target may reappear)
        }
    }

    //=========================================================================
    // TRACKING CONTROL
    //=========================================================================

    /**
     * @brief Apply proportional control to center target in frame
     *
     * Computes error from frame center, applies adaptive gain based on
     * error magnitude, and commands servo adjustments with deadband
     * and saturation limits.
     *
     * Coordinate System:
     *   - X axis: 0.0 (left) to 1.0 (right), center = 0.5
     *   - Y axis: 0.0 (top) to 1.0 (bottom), center = 0.5
     *   - Positive X error = target is RIGHT of center = increase pan
     *   - Positive Y error = target is BELOW center = adjust tilt
     *
     * @param person Detection to track
     */
    void track_target(const two_towers::msg::Detection& person) {
        double current_pan = turret_->getPanAngle();
        double current_tilt = turret_->getTiltAngle();

        // Compute error from center (0.5, 0.5)
        // Negative sign: camera coordinate system inversion
        double x_error = -(person.x - 0.5);
        double y_error = -(person.y - 0.5);

        // Select adaptive gain based on error magnitude
        double pan_gain = select_gain(std::abs(x_error));
        double tilt_gain = select_gain(std::abs(y_error));

        // Apply deadband: suppress micro-movements near center
        if (std::abs(x_error) < DEADBAND) x_error = 0.0;
        if (std::abs(y_error) < DEADBAND) y_error = 0.0;

        // Compute adjustments with saturation
        double pan_adj = std::clamp(x_error * pan_gain, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);
        double tilt_adj = std::clamp(y_error * tilt_gain, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);

        // Calculate new angles with servo limits
        double new_pan = std::clamp(current_pan + pan_adj, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        double new_tilt = std::clamp(current_tilt + tilt_adj, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        // Command turret
        turret_->setPanAngle(new_pan);
        turret_->setTiltAngle(new_tilt);

        // Warn if hitting limits
        if (new_pan <= SERVO_MIN_ANGLE || new_pan >= SERVO_MAX_ANGLE) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Pan servo at limit: %.1f", new_pan);
        }

        // Periodic status logging (every 10th call)
        static int log_counter = 0;
        if (++log_counter % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "Tracking: (%.3f, %.3f) | Pan: %.1f->%.1f | Tilt: %.1f->%.1f",
                person.x, person.y, current_pan, new_pan, current_tilt, new_tilt);
        }
    }

    /**
     * @brief Select proportional gain based on error magnitude
     *
     * Implements adaptive gain schedule:
     *   - Large errors: high gain for fast acquisition
     *   - Small errors: low gain for smooth centering
     *
     * @param error_magnitude Absolute value of tracking error
     * @return Gain value to use for proportional control
     */
    double select_gain(double error_magnitude) const {
        if (error_magnitude > ERROR_THRESHOLD_LARGE) {
            return GAIN_LARGE;
        } else if (error_magnitude > ERROR_THRESHOLD_MEDIUM) {
            return GAIN_MEDIUM;
        }
        return GAIN_FINE;
    }

    //=========================================================================
    // SCANNING BEHAVIOR
    //=========================================================================

    /**
     * @brief Execute scanning pattern to search for targets
     *
     * Sweeps pan axis back and forth between scan limits when no
     * target is detected. Maintains direction state to create
     * continuous sweeping motion.
     */
    void scan_for_target() {
        double current_pan = turret_->getPanAngle();
        double new_pan;

        // Determine scan direction and compute new position
        if (current_pan >= SCAN_RIGHT_LIMIT) {
            new_pan = current_pan - SCAN_STEP;
            last_scan_direction_ = -1;
        } else if (current_pan <= SCAN_LEFT_LIMIT) {
            new_pan = current_pan + SCAN_STEP;
            last_scan_direction_ = 1;
        } else {
            // Continue in current direction
            new_pan = current_pan + (last_scan_direction_ * SCAN_STEP);
        }

        // Clamp to scan limits
        new_pan = std::clamp(new_pan, SCAN_LEFT_LIMIT, SCAN_RIGHT_LIMIT);

        // Reverse direction at limits
        if (new_pan >= SCAN_RIGHT_LIMIT || new_pan <= SCAN_LEFT_LIMIT) {
            last_scan_direction_ *= -1;
        }

        turret_->setPanAngle(new_pan);

        // Periodic logging
        static int scan_log_counter = 0;
        if (++scan_log_counter % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), "Scanning: Pan=%.1f", new_pan);
        }
    }

    //=========================================================================
    // MEMBER VARIABLES
    //=========================================================================

    /// ROS2 subscription to detection topic
    rclcpp::Subscription<two_towers::msg::DetectionArray>::SharedPtr subscription_;

    /// Hardware turret controller
    std::unique_ptr<Turret> turret_;

    /// Counter for detection dropout tracking
    int frames_without_detection_;

    /// State flag: true if actively tracking a target
    bool tracking_active_;

    /// Scan direction: 1 = right, -1 = left
    int last_scan_direction_;
};


//=============================================================================
// MAIN ENTRY POINT
//=============================================================================

/**
 * @brief Node entry point
 *
 * Initializes ROS2, creates tracker node, and spins until shutdown.
 * Handles exceptions gracefully and ensures clean shutdown.
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<TwoTowersTrackerNode>();
        RCLCPP_INFO(node->get_logger(), "Tracker running. Press Ctrl+C to stop.");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("two_towers_tracker"),
                     "Fatal error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
