#include <rclcpp/rclcpp.hpp>
#include <two_towers/msg/detection_array.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include "Turret.hpp"
#include <memory>
#include <cmath>
#include <chrono>

// Shared state between ROS2 node and BehaviorTree
struct TrackerState {
    Turret* turret = nullptr;
    two_towers::msg::DetectionArray::SharedPtr latest_detections;
    rclcpp::Node* node = nullptr;

    // Simplified tracking state - NO MEMORY SYSTEM
    bool has_target = false;
    double target_x = 0.5;
    double target_y = 0.5;
    
    // Low-pass filter for smooth tracking
    double filtered_x = 0.5;
    double filtered_y = 0.5;
    const double FILTER_ALPHA = 0.6;  // Higher = more responsive (0.6 vs old 0.4)
    
    // Motion prediction with smoothing
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double raw_velocity_x = 0.0;
    double raw_velocity_y = 0.0;
    std::chrono::steady_clock::time_point last_update_time;
    bool velocity_initialized = false;
    const double VELOCITY_ALPHA = 0.3;  // Low-pass filter for velocity
    
    // Timing
    std::chrono::steady_clock::time_point last_tick_time;
    
    // Corner escape detection
    int frames_at_limit = 0;
    const int MAX_FRAMES_AT_LIMIT = 20;  // 1.3 seconds at 15Hz

    static TrackerState& get() {
        static TrackerState instance;
        return instance;
    }

    void update_target(double new_x, double new_y) {
        // Calculate raw velocity from unfiltered position changes
        auto now = std::chrono::steady_clock::now();
        if (velocity_initialized) {
            double dt = std::chrono::duration<double>(now - last_update_time).count();
            if (dt > 0.01 && dt < 0.5) {  // Sanity check (10ms to 500ms)
                // Use RAW position change for velocity, not filtered
                raw_velocity_x = (new_x - target_x) / dt;
                raw_velocity_y = (new_y - target_y) / dt;
                
                // Clamp raw velocity
                raw_velocity_x = std::clamp(raw_velocity_x, -2.0, 2.0);
                raw_velocity_y = std::clamp(raw_velocity_y, -2.0, 2.0);
                
                // Low-pass filter velocity for smooth prediction
                velocity_x = VELOCITY_ALPHA * raw_velocity_x + (1.0 - VELOCITY_ALPHA) * velocity_x;
                velocity_y = VELOCITY_ALPHA * raw_velocity_y + (1.0 - VELOCITY_ALPHA) * velocity_y;
            }
        } else {
            // Initialize velocities to zero
            velocity_x = 0.0;
            velocity_y = 0.0;
        }
        last_update_time = now;
        velocity_initialized = true;
        
        // Low-pass filter position
        filtered_x = FILTER_ALPHA * new_x + (1.0 - FILTER_ALPHA) * filtered_x;
        filtered_y = FILTER_ALPHA * new_y + (1.0 - FILTER_ALPHA) * filtered_y;
        
        target_x = filtered_x;
        target_y = filtered_y;
        has_target = true;
    }
    
    void get_predicted_target(double lookahead_sec, double& pred_x, double& pred_y) {
        // Predict where target will be in lookahead_sec seconds
        pred_x = target_x + velocity_x * lookahead_sec;
        pred_y = target_y + velocity_y * lookahead_sec;
        
        // Clamp to valid range
        pred_x = std::clamp(pred_x, 0.0, 1.0);
        pred_y = std::clamp(pred_y, 0.0, 1.0);
    }
    
    void clear_target() {
        has_target = false;
        frames_at_limit = 0;
        velocity_x = 0.0;
        velocity_y = 0.0;
        raw_velocity_x = 0.0;
        raw_velocity_y = 0.0;
        velocity_initialized = false;
    }
};

// ============================================================================
// BehaviorTree Condition: Check if person detected (NO MEMORY)
// ============================================================================
class HasPersonDetection : public BT::ConditionNode {
public:
    HasPersonDetection(const std::string& name) : BT::ConditionNode(name, {}) {}
    
    BT::NodeStatus tick() override {
        auto& state = TrackerState::get();
        
        if (!state.latest_detections || state.latest_detections->detections.empty()) {
            state.clear_target();
            return BT::NodeStatus::FAILURE;
        }

        // Find best person (closest to center, highest confidence)
        const two_towers::msg::Detection* best_person = nullptr;
        double best_score = -1.0;
        
        for (const auto& det : state.latest_detections->detections) {
            if (det.label == "person") {
                double dist = std::sqrt(
                    std::pow(det.x - 0.5, 2) + std::pow(det.y - 0.5, 2)
                );
                double score = det.confidence / (1.0 + dist);

                if (score > best_score) {
                    best_score = score;
                    best_person = &det;
                }
            }
        }

        if (!best_person) {
            state.clear_target();
            return BT::NodeStatus::FAILURE;
        }
        
        // Update target with filtering
        state.update_target(best_person->x, best_person->y);
        
        return BT::NodeStatus::SUCCESS;
    }
};

// ============================================================================
// BehaviorTree Action: Smooth proportional tracking
// ============================================================================
class SmoothTrackAction : public BT::SyncActionNode {
public:
    SmoothTrackAction(const std::string& name) : BT::SyncActionNode(name, {}) {}
    
    BT::NodeStatus tick() override {
        auto& state = TrackerState::get();
        
        if (!state.turret || !state.has_target) {
            return BT::NodeStatus::FAILURE;
        }

        double current_pan = state.turret->getPanAngle();
        double current_tilt = state.turret->getTiltAngle();
        
        // Use PREDICTED position (150ms lookahead compensates for system latency)
        double target_x, target_y;
        state.get_predicted_target(0.15, target_x, target_y);
        
        // Calculate error (negative because camera coordinates are inverted)
        double x_error = -(target_x - 0.5);
        double y_error = -(target_y - 0.5);

        // WIDER deadband to prevent oscillation
        const double deadband = 0.08;  // Increased from 0.05
        if (std::abs(x_error) < deadband) x_error = 0.0;
        if (std::abs(y_error) < deadband) y_error = 0.0;

        // If centered, we're done
        if (x_error == 0.0 && y_error == 0.0) {
            static int log_count = 0;
            if (++log_count % 30 == 0 && state.node) {
                RCLCPP_INFO(state.node->get_logger(), "🎯 Centered on target");
            }
            return BT::NodeStatus::SUCCESS;
        }

        // ADAPTIVE gains based on velocity magnitude
        double speed = std::sqrt(state.velocity_x * state.velocity_x + 
                                state.velocity_y * state.velocity_y);
        
        // More aggressive boost for fast motion (up to 2.5x)
        double speed_multiplier = 1.0 + std::min(speed * 0.8, 1.5);
        
        const double pan_gain = (std::abs(x_error) > 0.20 ? 3.5 :
                         (std::abs(x_error) > 0.12 ? 2.0 : 1.0)) * speed_multiplier;

        const double tilt_gain = (std::abs(y_error) > 0.20 ? 3.5 : 
                          (std::abs(y_error) > 0.12 ? 2.0 : 1.0)) * speed_multiplier;
        
        // Calculate adjustments with ADAPTIVE clamps (up to 3° when moving fast)
        double max_adj = 1.5 + speed * 0.8;  // 1.5° base, up to ~2.5° when fast
        max_adj = std::min(max_adj, 3.0);    // Cap at 3°
        double pan_adj = std::clamp(x_error * pan_gain, -max_adj, max_adj);
        double tilt_adj = std::clamp(y_error * tilt_gain, -max_adj, max_adj);
        
        double new_pan = std::clamp(current_pan + pan_adj, 10.0, 170.0);
        double new_tilt = std::clamp(current_tilt + tilt_adj, 10.0, 170.0);
        
        state.turret->setPanAngle(new_pan);
        state.turret->setTiltAngle(new_tilt);

        // Small delay proportional to movement
        double total_movement = std::abs(pan_adj) + std::abs(tilt_adj);
        int wait_ms = std::min(80, static_cast<int>(30 + total_movement * 15));
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
        
        // Log occasionally
        static int log_count = 0;
        if (++log_count % 20 == 0 && state.node) {
            RCLCPP_INFO(state.node->get_logger(),
                "🎯 Track: (%.3f,%.3f) pred:(%.3f,%.3f) vel:(%.2f,%.2f) | Pan: %.1f°→%.1f°",
                state.target_x, state.target_y, target_x, target_y,
                state.velocity_x, state.velocity_y,
                current_pan, new_pan);
        }
        
        return BT::NodeStatus::SUCCESS;
    }
};

// ============================================================================
// BehaviorTree Action: Intelligent scanning with corner escape
// ============================================================================
class IntelligentScanAction : public BT::SyncActionNode {
public:
    IntelligentScanAction(const std::string& name) 
        : BT::SyncActionNode(name, {}), 
        scan_direction_(1),
        last_scan_time_(std::chrono::steady_clock::now()) {}
    
    BT::NodeStatus tick() override {
        auto& state = TrackerState::get();
        
        if (!state.turret) {
            return BT::NodeStatus::FAILURE;
        }
        
        // Scan every 0.8 seconds
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - last_scan_time_).count();
        
        if (elapsed < 0.8) {
            return BT::NodeStatus::SUCCESS;
        }
        
        last_scan_time_ = now;

        double current_pan = state.turret->getPanAngle();
        double pan_adj = 0.0;

        // FIXED: Detect being stuck at hard limits
        bool at_right_limit = (current_pan >= 165.0);
        bool at_left_limit = (current_pan <= 15.0);
        
        if (at_right_limit || at_left_limit) {
            state.frames_at_limit++;
            
            // Force direction reversal if stuck
            if (state.frames_at_limit > state.MAX_FRAMES_AT_LIMIT) {
                if (state.node) {
                    RCLCPP_WARN(state.node->get_logger(), 
                        "⚠️  Stuck at %s limit! Reversing scan direction",
                        at_right_limit ? "RIGHT" : "LEFT");
                }
                scan_direction_ = at_right_limit ? -1 : 1;
                state.frames_at_limit = 0;
            }
        } else {
            state.frames_at_limit = 0;
        }
        
        // Normal direction switching at comfortable margins
        if (current_pan >= 155.0 && scan_direction_ == 1) {
            scan_direction_ = -1;
        } else if (current_pan <= 25.0 && scan_direction_ == -1) {
            scan_direction_ = 1;
        }
        
        // Apply movement
        pan_adj = scan_direction_ == 1 ? 15.0 : -15.0;
        
        double new_pan = std::clamp(current_pan + pan_adj, 10.0, 170.0);
        state.turret->setPanAngle(new_pan);
        
        if (state.node) {
            RCLCPP_INFO(state.node->get_logger(), 
                "🔍 Scan: %.1f° → %.1f° (%s)", 
                current_pan, new_pan, 
                scan_direction_ == 1 ? "→" : "←");
        }
        
        return BT::NodeStatus::SUCCESS;
    }
    
private:
    int scan_direction_;
    std::chrono::steady_clock::time_point last_scan_time_;
};

// ============================================================================
// ROS2 Node with BehaviorTree
// ============================================================================
class OrthancTrackerBTNode : public rclcpp::Node {
public:
    OrthancTrackerBTNode() 
        : Node("orthanc_tracker_bt"),
          turret_(std::make_unique<Turret>(17, 27))
    {
        // Initialize turret
        turret_->setPanAngle(90.0);
        turret_->setTiltAngle(30.0);
        
        // Setup shared state
        auto& state = TrackerState::get();
        state.turret = turret_.get();
        state.node = this;
        state.last_tick_time = std::chrono::steady_clock::now();
        
        // Subscribe to detections
        subscription_ = this->create_subscription<two_towers::msg::DetectionArray>(
            "orthanc/detections",
            10,
            [this](two_towers::msg::DetectionArray::SharedPtr msg) {
                TrackerState::get().latest_detections = msg;
            }
        );
        
        // Create BehaviorTree
        BT::BehaviorTreeFactory factory;
        factory.registerNodeType<HasPersonDetection>("HasPersonDetection");
        factory.registerNodeType<SmoothTrackAction>("SmoothTrack");
        factory.registerNodeType<IntelligentScanAction>("IntelligentScan");
        
        const std::string xml_tree = R"(
            <root BTCPP_format="4">
                <BehaviorTree ID="TrackingTree">
                    <Fallback>
                        <Sequence>
                            <HasPersonDetection/>
                            <SmoothTrack/>
                        </Sequence>
                        <IntelligentScan/>
                    </Fallback>
                </BehaviorTree>
            </root>
        )";
        
        tree_ = factory.createTreeFromText(xml_tree);
        
        // Timer at 15Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(67),
            [this]() { tree_.tickOnce(); }
        );
        
        RCLCPP_INFO(this->get_logger(), "🗼 Orthanc tracker initialized (15Hz control)");
    }
    
    ~OrthancTrackerBTNode() {
        RCLCPP_INFO(this->get_logger(), "Centering turret...");
        turret_->centerAll();
    }

private:
    std::unique_ptr<Turret> turret_;
    BT::Tree tree_;
    rclcpp::Subscription<two_towers::msg::DetectionArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================================
// Main
// ============================================================================
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<OrthancTrackerBTNode>();
        RCLCPP_INFO(node->get_logger(), "🔄 Tracker running. Ctrl+C to stop.");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("orthanc_tracker_bt"), 
                     "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}