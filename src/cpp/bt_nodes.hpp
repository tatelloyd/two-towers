/**
 * @file bt_nodes.hpp
 * @brief Behavior Tree Node Implementations for Two Towers Tracking System
 *
 * Defines custom BehaviorTree.CPP nodes for autonomous person tracking:
 *   - HasPersonDetection: Condition node that reads and validates detections
 *   - ProportionalTrackAction: Action node for proportional tracking control
 *   - SimpleScanAction: Action node for systematic search patterns
 *
 * @section BehaviorTreeStructure Behavior Tree Structure
 *
 *     Fallback
 *     ├── Sequence (Active Tracking)
 *     │   ├── HasPersonDetection?   <- Reads detections.json
 *     │   └── ProportionalTrack     <- Commands servos
 *     └── SimpleScan                <- Fallback search pattern
 *
 * @section IPC Inter-Process Communication
 *
 * Uses file-based IPC with JSON format and POSIX file locking:
 *   - Python detector writes detections.json with exclusive lock
 *   - C++ tracker reads with shared lock
 *   - Timestamp validation rejects stale data (>500ms)
 *
 * @section ControlAlgorithm Control Algorithm
 *
 * Proportional control with adaptive gains and exponential smoothing:
 *
 *     smoothed_pos = alpha * new_pos + (1 - alpha) * smoothed_pos
 *     error = -(smoothed_pos - 0.5)  // Inverted for camera orientation
 *     gain = f(|error|)               // Adaptive gain selection
 *     adjustment = clamp(error * gain, max_adj)
 *
 * @author Tate Lloyd <tate.lloyd@yale.edu>
 * @license MIT
 */

#ifndef BT_NODES_HPP
#define BT_NODES_HPP

#include <behaviortree_cpp/behavior_tree.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <nlohmann/json.hpp>
#include <sys/file.h>

#include "Turret.hpp"

using json = nlohmann::json;


//=============================================================================
// TRACKER STATE (SINGLETON)
//=============================================================================

/**
 * @struct TrackerState
 * @brief Global state container for behavior tree nodes
 *
 * Provides shared state across all BT nodes including:
 *   - Detection data and validation state
 *   - Smoothed tracking coordinates
 *   - Recovery mode for temporary target loss
 *   - Hardware turret reference
 *
 * Uses Meyer's singleton pattern for thread-safe initialization.
 */
struct TrackerState {
    //-------------------------------------------------------------------------
    // Detection Source
    //-------------------------------------------------------------------------

    /// Path to JSON detection file written by Python detector
    std::string detection_file = "detections.json";

    /// Raw detections from current frame
    std::vector<json> current_detections;

    /// Selected target person (best candidate from detections)
    json target_person;

    /// True if a valid person detection exists this tick
    bool has_person = false;

    //-------------------------------------------------------------------------
    // Smoothed Position Tracking
    //-------------------------------------------------------------------------

    /// Exponentially smoothed X coordinate (0.0-1.0, center=0.5)
    double smoothed_x = 0.5;

    /// Exponentially smoothed Y coordinate (0.0-1.0, center=0.5)
    double smoothed_y = 0.5;

    //-------------------------------------------------------------------------
    // Detection Continuity Tracking
    //-------------------------------------------------------------------------

    /// Frames since last valid detection (for dropout handling)
    int frames_since_detection = 0;

    /// Timestamp of last successful detection
    std::chrono::steady_clock::time_point last_detection_time;

    /// Maximum time without detection before declaring target lost (seconds)
    const double loss_timeout_seconds = 2.0;

    //-------------------------------------------------------------------------
    // Target Lock State
    //-------------------------------------------------------------------------

    /// Counter for consecutive frames with target near center
    int consecutive_centered_frames = 0;

    /// Frames needed at center before declaring "locked"
    const int frames_needed_to_lock = 5;

    //-------------------------------------------------------------------------
    // Recovery Mode (Temporary Target Loss)
    //-------------------------------------------------------------------------

    /// True if in recovery mode (recently lost target, holding position)
    bool in_recovery_mode = false;

    /// Last known X position before target was lost
    double last_known_x = 0.5;

    /// Last known Y position before target was lost
    double last_known_y = 0.5;

    //-------------------------------------------------------------------------
    // Velocity Filtering (Jump Rejection)
    //-------------------------------------------------------------------------

    /// Previous smoothed X for velocity calculation
    double prev_x = 0.5;

    /// Previous smoothed Y for velocity calculation
    double prev_y = 0.5;

    /// Maximum allowed position change per tick (20% of frame)
    const double max_velocity = 0.2;

    //-------------------------------------------------------------------------
    // Hardware Reference
    //-------------------------------------------------------------------------

    /// Pointer to turret hardware controller (set externally)
    Turret* turret = nullptr;

    //-------------------------------------------------------------------------
    // Singleton Access
    //-------------------------------------------------------------------------

    /**
     * @brief Get singleton instance
     * @return Reference to global TrackerState
     */
    static TrackerState& get() {
        static TrackerState instance;
        return instance;
    }
};


//=============================================================================
// CONDITION NODE: HasPersonDetection
//=============================================================================

/**
 * @class HasPersonDetection
 * @brief BT Condition node that reads and validates person detections
 *
 * Reads detections from JSON file with file locking, validates freshness
 * and position plausibility, then updates TrackerState if valid.
 *
 * Detection Validation Pipeline:
 *   1. Open file with shared lock (prevents write conflicts)
 *   2. Parse JSON and check timestamp (<500ms old)
 *   3. Find "person" detections within valid frame region
 *   4. Reject detections too far from current position (false positives)
 *   5. Check velocity constraints (reject teleportation)
 *   6. Prefer "stable" detections marked by Python tracker
 *
 * Return Values:
 *   - SUCCESS: Valid person detected, TrackerState updated
 *   - SUCCESS: No detection but in recovery mode (hold position)
 *   - FAILURE: No valid detection and recovery timeout expired
 *
 * @note This node modifies TrackerState as a side effect
 */
class HasPersonDetection : public BT::ConditionNode {
public:
    HasPersonDetection(const std::string& name)
        : BT::ConditionNode(name, {}) {}

    BT::NodeStatus tick() override {
        auto& state = TrackerState::get();
        state.has_person = false;
        state.current_detections.clear();

        std::cout << "[Detection] Reading: " << state.detection_file << std::endl;

        //---------------------------------------------------------------------
        // File Access with Locking
        //---------------------------------------------------------------------

        // Open file descriptor for locking
        int fd = open(state.detection_file.c_str(), O_RDONLY);
        if (fd < 0) {
            std::cout << "[Detection] WARNING: Cannot open " << state.detection_file << std::endl;
            state.frames_since_detection++;
            return BT::NodeStatus::FAILURE;
        }

        // Acquire shared lock (blocks if writer has exclusive lock)
        if (flock(fd, LOCK_SH) != 0) {
            close(fd);
            std::cout << "[Detection] WARNING: Cannot acquire lock" << std::endl;
            state.frames_since_detection++;
            return BT::NodeStatus::FAILURE;
        }

        // Open ifstream for JSON parsing
        std::ifstream file(state.detection_file);

        try {
            json data;
            file >> data;

            // Release lock after reading
            flock(fd, LOCK_UN);
            close(fd);

            //---------------------------------------------------------------------
            // Timestamp Validation
            //---------------------------------------------------------------------

            if (data.contains("timestamp")) {
                double timestamp = data["timestamp"];
                auto now = std::chrono::system_clock::now();
                double current_time = std::chrono::duration<double>(
                    now.time_since_epoch()).count();

                double age = current_time - timestamp;

                // Reject stale detections (>500ms old)
                if (age > 0.5) {
                    std::cout << "[Detection] WARNING: Stale detection (" << age << "s old)" << std::endl;
                    state.frames_since_detection++;
                    return BT::NodeStatus::FAILURE;
                }
                std::cout << "[Detection] Age: " << (age * 1000) << "ms" << std::endl;
            }

            //---------------------------------------------------------------------
            // Parse Detection Array
            //---------------------------------------------------------------------

            json detections_array = data.contains("detections") ?
                                    data["detections"] : data;

            std::cout << "[Detection] Parsed " << detections_array.size() << " detections" << std::endl;

            if (!detections_array.is_array()) {
                std::cout << "[Detection] ERROR: Not an array!" << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            //---------------------------------------------------------------------
            // Find Valid Person Detection
            //---------------------------------------------------------------------

            for (const auto& det : detections_array) {
                state.current_detections.push_back(det);

                if (det.value("label", "") == "person") {
                    double x = det.value("x", 0.5);
                    double y = det.value("y", 0.5);
                    double conf = det.value("confidence", 0.0);

                    // Reject edge detections (often partial/clipped)
                    // Only accept detections in middle 60% of frame
                    if (x < 0.2 || x > 0.8 || y < 0.2 || y > 0.8) {
                        std::cout << "[Detection] Rejected: edge position (" << x << ", " << y << ")" << std::endl;
                        continue;
                    }

                    // Reject detections too far from current smoothed position
                    // Prevents jumping to false positives
                    double distance = std::sqrt(std::pow(x - state.smoothed_x, 2) +
                                                std::pow(y - state.smoothed_y, 2));

                    // Allow larger jumps when first acquiring target
                    double max_allowed_distance = (state.frames_since_detection < 3) ? 0.35 : 0.20;

                    if (distance > max_allowed_distance && state.has_person) {
                        std::cout << "[Detection] Rejected: too far from current (dist=" << distance << ")" << std::endl;
                        continue;
                    }

                    // Prefer stable detections (marked by Python tracker's temporal filter)
                    bool is_stable = det.value("stable", false);
                    double score = conf;
                    if (is_stable) {
                        score *= 1.5;  // Heavily prefer stable detections
                    }

                    // Velocity check: reject sudden teleportation
                    double velocity = std::sqrt(std::pow(x - state.prev_x, 2) +
                                                std::pow(y - state.prev_y, 2));

                    // Allow larger velocity if target was recently lost
                    if (velocity < state.max_velocity || state.frames_since_detection > 10) {
                        state.target_person = det;
                        state.has_person = true;
                        std::cout << "[Detection] Accepted person" << (is_stable ? " (STABLE)" : "")
                                  << " conf=" << conf << " dist=" << distance << std::endl;
                        break;  // Take first valid detection
                    } else {
                        std::cout << "[Detection] Rejected: velocity too high (" << velocity << ")" << std::endl;
                    }
                }
            }

        } catch (const std::exception& e) {
            // JSON parse error - release lock and fail gracefully
            flock(fd, LOCK_UN);
            close(fd);
            std::cout << "[Detection] ERROR: JSON parse failed: " << e.what() << std::endl;
            state.frames_since_detection++;
            return BT::NodeStatus::FAILURE;
        }

        //---------------------------------------------------------------------
        // Update State and Return
        //---------------------------------------------------------------------

        if (state.has_person) {
            double x = state.target_person.value("x", 0.0);
            double y = state.target_person.value("y", 0.0);

            // Update recovery position
            state.last_known_x = x;
            state.last_known_y = y;
            state.frames_since_detection = 0;
            state.in_recovery_mode = false;

            // Update velocity tracking (for next frame's jump rejection)
            state.prev_x = state.smoothed_x;
            state.prev_y = state.smoothed_y;

            // Update smoothed position
            state.smoothed_x = x;
            state.smoothed_y = y;

            std::cout << "[Detection] Person locked at (" << x << ", " << y << ")" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        // No valid detection - handle recovery mode
        std::cout << "[Detection] No valid person (frames lost: " << state.frames_since_detection << ")" << std::endl;
        state.frames_since_detection++;

        // Hold position during brief dropouts (prevents chasing ghosts)
        if (state.frames_since_detection <= 20) {
            std::cout << "[Detection] Recovery mode - holding position" << std::endl;
            state.in_recovery_mode = true;
            return BT::NodeStatus::SUCCESS;  // Continue tracking last known position
        }

        return BT::NodeStatus::FAILURE;  // Target truly lost
    }
};


//=============================================================================
// ACTION NODE: ProportionalTrackAction
//=============================================================================

/**
 * @class ProportionalTrackAction
 * @brief BT Action node that applies proportional control to track target
 *
 * Implements smooth proportional tracking with:
 *   - Exponential smoothing on position coordinates
 *   - Adaptive gains based on error magnitude
 *   - Deadband to prevent micro-adjustments
 *   - Saturation limits on servo adjustments
 *   - Recovery mode support (track last known position)
 *
 * Control Flow:
 *   1. Get target position (current detection or recovery position)
 *   2. Apply exponential smoothing (alpha = 0.4)
 *   3. Compute error from frame center
 *   4. Apply deadband (5% threshold)
 *   5. Select adaptive gain based on error magnitude
 *   6. Compute and clamp servo adjustments
 *   7. Command turret hardware
 *
 * Return Values:
 *   - SUCCESS: Target centered (within deadband)
 *   - SUCCESS: Adjustment applied successfully
 *   - FAILURE: No turret hardware available
 *   - FAILURE: No target to track and not in recovery mode
 */
class ProportionalTrackAction : public BT::SyncActionNode {
public:
    //-------------------------------------------------------------------------
    // Control Parameters
    //-------------------------------------------------------------------------

    /// Exponential smoothing factor (0-1, higher = more responsive)
    static constexpr double SMOOTHING_ALPHA = 0.4;

    /// Deadband threshold (don't move if error < this)
    static constexpr double DEADBAND = 0.05;

    /// Maximum servo adjustment per tick (degrees)
    static constexpr double MAX_ADJUSTMENT = 2.0;

    /// Servo angle limits (degrees)
    static constexpr double SERVO_MIN = 10.0;
    static constexpr double SERVO_MAX = 170.0;

    /// Gain schedule thresholds
    static constexpr double LARGE_ERROR_THRESHOLD = 0.15;
    static constexpr double MEDIUM_ERROR_THRESHOLD = 0.08;

    /// Gain values for different error magnitudes
    static constexpr double GAIN_LARGE = 8.0;
    static constexpr double GAIN_MEDIUM = 4.0;
    static constexpr double GAIN_FINE = 2.0;

    ProportionalTrackAction(const std::string& name)
        : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        auto& state = TrackerState::get();

        // Verify hardware is available
        if (!state.turret) {
            std::cout << "[Track] WARNING: No turret available" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        //---------------------------------------------------------------------
        // Determine Target Position
        //---------------------------------------------------------------------

        double target_x, target_y;

        if (state.in_recovery_mode) {
            // Use last known position during recovery
            target_x = state.last_known_x;
            target_y = state.last_known_y;
            std::cout << "[Track] Recovery mode: using last position (" << target_x << ", " << target_y << ")" << std::endl;
        } else if (!state.has_person) {
            std::cout << "[Track] WARNING: No target to track" << std::endl;
            return BT::NodeStatus::FAILURE;
        } else {
            target_x = state.smoothed_x;
            target_y = state.smoothed_y;
        }

        //---------------------------------------------------------------------
        // Apply Exponential Smoothing
        //---------------------------------------------------------------------

        state.smoothed_x = SMOOTHING_ALPHA * target_x + (1.0 - SMOOTHING_ALPHA) * state.smoothed_x;
        state.smoothed_y = SMOOTHING_ALPHA * target_y + (1.0 - SMOOTHING_ALPHA) * state.smoothed_y;

        //---------------------------------------------------------------------
        // Compute Error (Inverted for Upside-Down Camera)
        //---------------------------------------------------------------------

        double x_error = -(state.smoothed_x - 0.5);
        double y_error = -(state.smoothed_y - 0.5);

        //---------------------------------------------------------------------
        // Check for Centered Target
        //---------------------------------------------------------------------

        bool is_centered = (std::abs(x_error) < DEADBAND && std::abs(y_error) < DEADBAND);

        if (is_centered && !state.in_recovery_mode) {
            state.consecutive_centered_frames++;
            if (state.consecutive_centered_frames >= state.frames_needed_to_lock) {
                std::cout << "[Track] LOCKED (centered for " << state.consecutive_centered_frames << " frames)" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
        } else {
            state.consecutive_centered_frames = 0;
        }

        //---------------------------------------------------------------------
        // Apply Deadband
        //---------------------------------------------------------------------

        if (std::abs(x_error) < DEADBAND) x_error = 0;
        if (std::abs(y_error) < DEADBAND) y_error = 0;

        if (x_error == 0 && y_error == 0) {
            return BT::NodeStatus::SUCCESS;
        }

        //---------------------------------------------------------------------
        // Compute Adaptive Gains
        //---------------------------------------------------------------------

        double pan_gain = select_gain(std::abs(x_error));
        double tilt_gain = select_gain(std::abs(y_error));

        //---------------------------------------------------------------------
        // Compute and Clamp Adjustments
        //---------------------------------------------------------------------

        double pan_adj = std::clamp(x_error * pan_gain, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);
        double tilt_adj = std::clamp(y_error * tilt_gain, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);

        //---------------------------------------------------------------------
        // Apply to Hardware
        //---------------------------------------------------------------------

        double current_pan = state.turret->getPanAngle();
        double current_tilt = state.turret->getTiltAngle();

        double new_pan = std::clamp(current_pan + pan_adj, SERVO_MIN, SERVO_MAX);
        double new_tilt = std::clamp(current_tilt + tilt_adj, SERVO_MIN, SERVO_MAX);

        // Warn if hitting limits
        if (new_pan <= SERVO_MIN || new_pan >= SERVO_MAX) {
            std::cout << "[Track] WARNING: Pan servo at limit!" << std::endl;
        }

        state.turret->setPanAngle(new_pan);
        state.turret->setTiltAngle(new_tilt);

        //---------------------------------------------------------------------
        // Dynamic Wait Based on Movement Size
        //---------------------------------------------------------------------

        double total_movement = std::abs(pan_adj) + std::abs(tilt_adj);
        int wait_ms = std::min(100, static_cast<int>(40 + total_movement * 10));
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));

        //---------------------------------------------------------------------
        // Status Logging
        //---------------------------------------------------------------------

        std::cout << "[Track] target(" << target_x << ", " << target_y << ") "
                  << "err(" << x_error << ", " << y_error << ") "
                  << "-> pan " << current_pan << "->" << new_pan << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

private:
    /**
     * @brief Select gain based on error magnitude
     * @param error_magnitude Absolute value of tracking error
     * @return Appropriate gain value
     */
    double select_gain(double error_magnitude) const {
        if (error_magnitude > LARGE_ERROR_THRESHOLD) {
            return GAIN_LARGE;
        } else if (error_magnitude > MEDIUM_ERROR_THRESHOLD) {
            return GAIN_MEDIUM;
        }
        return GAIN_FINE;
    }
};


//=============================================================================
// ACTION NODE: SimpleScanAction
//=============================================================================

/**
 * @class SimpleScanAction
 * @brief BT Action node that executes systematic search pattern
 *
 * Sweeps the pan axis back and forth when no target is detected.
 * Uses time-gated movement to prevent continuous motion that would
 * blur camera frames.
 *
 * Scan Pattern:
 *   - Sweep right until hitting 150 degrees
 *   - Reverse and sweep left until hitting 30 degrees
 *   - Repeat
 *
 * Timing:
 *   - 2 second interval between movements
 *   - 15 degree steps per movement
 *
 * Return Values:
 *   - SUCCESS: Scan step executed or waiting between steps
 *   - FAILURE: No turret hardware available
 */
class SimpleScanAction : public BT::SyncActionNode {
public:
    //-------------------------------------------------------------------------
    // Scan Parameters
    //-------------------------------------------------------------------------

    /// Time between scan movements (seconds)
    static constexpr double SCAN_INTERVAL = 2.0;

    /// Pan step size per movement (degrees)
    static constexpr double SCAN_STEP = 15.0;

    /// Scan range limits (degrees)
    static constexpr double SCAN_MIN = 30.0;
    static constexpr double SCAN_MAX = 150.0;

    /// Servo limits (degrees)
    static constexpr double SERVO_MIN = 10.0;
    static constexpr double SERVO_MAX = 170.0;

    SimpleScanAction(const std::string& name)
        : BT::SyncActionNode(name, {}),
          last_move_time_(std::chrono::steady_clock::now()),
          scanning_right_(true) {}

    BT::NodeStatus tick() override {
        auto& state = TrackerState::get();

        if (!state.turret) {
            return BT::NodeStatus::FAILURE;
        }

        //---------------------------------------------------------------------
        // Time Gating: Only Move at Intervals
        //---------------------------------------------------------------------

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_move_time_).count();

        if (elapsed < SCAN_INTERVAL) {
            return BT::NodeStatus::SUCCESS;  // Wait between movements
        }

        last_move_time_ = now;

        //---------------------------------------------------------------------
        // Compute Scan Movement
        //---------------------------------------------------------------------

        double current_pan = state.turret->getPanAngle();
        double pan_adj = 0.0;

        // Reverse direction at limits
        if (current_pan >= SCAN_MAX) {
            scanning_right_ = false;
        } else if (current_pan <= SCAN_MIN) {
            scanning_right_ = true;
        }

        // Apply step in current direction
        if (scanning_right_) {
            pan_adj = SCAN_STEP;
            std::cout << "[Scan] Sweeping right (+15)" << std::endl;
        } else {
            pan_adj = -SCAN_STEP;
            std::cout << "[Scan] Sweeping left (-15)" << std::endl;
        }

        //---------------------------------------------------------------------
        // Apply Movement
        //---------------------------------------------------------------------

        double new_pan = std::clamp(current_pan + pan_adj, SERVO_MIN, SERVO_MAX);
        state.turret->setPanAngle(new_pan);

        std::cout << "[Scan] Pan: " << current_pan << " -> " << new_pan << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

private:
    /// Timestamp of last scan movement
    std::chrono::steady_clock::time_point last_move_time_;

    /// Current scan direction (true = right, false = left)
    bool scanning_right_;
};


#endif // BT_NODES_HPP
