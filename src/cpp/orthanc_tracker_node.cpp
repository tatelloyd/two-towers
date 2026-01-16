#include <rclcpp/rclcpp.hpp>
#include <two_towers/msg/detection_array.hpp>
#include "Turret.hpp"
#include <memory>
#include <cmath>

class OrthancTrackerNode : public rclcpp::Node {
public:
    OrthancTrackerNode() 
        : Node("orthanc_tracker"),
          turret_(std::make_unique<Turret>(17, 27)),  // GPIO pins for pan, tilt
          frames_without_detection_(0),
          tracking_active_(false),
          last_scan_direction_(1)
    {
        // Subscribe to orthanc/detections topic
        subscription_ = this->create_subscription<two_towers::msg::DetectionArray>(
            "orthanc/detections",
            10,
            std::bind(&OrthancTrackerNode::detection_callback, this, std::placeholders::_1)
        );

        // Initialize turret to center position
        turret_->setPanAngle(90.0);   // Center
        turret_->setTiltAngle(30.0);  // Look slightly down
        
        RCLCPP_INFO(this->get_logger(), "🗼 Orthanc tracker started - listening for detections");
        RCLCPP_INFO(this->get_logger(), "   Turret centered at Pan=90°, Tilt=45°");
        RCLCPP_INFO(this->get_logger(), "   Listening for detections...");
    }

    ~OrthancTrackerNode() {
        // Return turret to center on shutdown
        RCLCPP_INFO(this->get_logger(), "Centering turret before shutdown...");
        turret_->centerAll();
    }

private:
    void detection_callback(const two_towers::msg::DetectionArray::SharedPtr msg) {
        // Find the best "person" detection (closest to center, highest confidence)
        const two_towers::msg::Detection* best_person = nullptr;
        double best_score = -1.0;
        
        for (const auto& det : msg->detections) {
            if (det.label == "person") {
                // Score = confidence / distance_from_center
                // Prioritizes centered, high-confidence detections
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
            // Person detected - track it!
            track_target(*best_person);
            frames_without_detection_ = 0;
            tracking_active_ = true;
        } else {
            // No person detected
            frames_without_detection_++;
            
            if (frames_without_detection_ > 60) {  // ~3 seconds at 10Hz
                // Lost target - start scanning
                if (tracking_active_) {
                    RCLCPP_INFO(this->get_logger(), "Target lost - entering scan mode");
                    tracking_active_ = false;
                }
                scan_for_target();
            }
            // else: hold position briefly in case target reappears
        }
        
        
    }

    void track_target(const two_towers::msg::Detection& person) {
        // Get current turret position
        double current_pan = turret_->getPanAngle();
        double current_tilt = turret_->getTiltAngle();
        
        // Calculate error from center (0.5, 0.5)
        double x_error = -(person.x - 0.5);  // Positive = target is RIGHT
        double y_error = -(person.y - 0.5);  // Positive = target is DOWN
        
        // Proportional control gains (degrees per normalized unit)  
        double pan_gain = std::abs(x_error) > 0.15 ? 8.0 :
                         (std::abs(x_error) > 0.08 ? 4.0 : 2.0);
        double tilt_gain = std::abs(y_error) > 0.15 ? 8.0 : 
                          (std::abs(y_error) > 0.08 ? 4.0 : 2.0);
        

        // Deadband - don't move if target is very close to center
        const double deadband = 0.05;
        if (std::abs(x_error) < deadband) x_error = 0.0;
        if (std::abs(y_error) < deadband) y_error = 0.0;
        
        // Calculate adjustments
        double pan_adj = std::clamp(x_error * pan_gain, -2.0, 2.0);
        double tilt_adj = std::clamp(y_error * tilt_gain, -2.0, 2.0);
        
        // Clamp adjustments to reasonable limits
        pan_adj = std::clamp(pan_adj, -30.0, 30.0);
        tilt_adj = std::clamp(tilt_adj, -20.0, 20.0);
        
        // Calculate new angles
        double new_pan = std::clamp(current_pan + pan_adj, 10.0, 170.0);
        double new_tilt = std::clamp(current_tilt + tilt_adj, 10.0, 170.0);
        
        // Apply new angles
        turret_->setPanAngle(new_pan);
        turret_->setTiltAngle(new_tilt);

        if (new_pan <= 10.0 || new_pan >= 170.0) {
            std::cout << "⚠️  Pan servo at limit!\n";
        }
        
        // Log tracking info (only occasionally to avoid spam)
        static int log_counter = 0;
        if (++log_counter % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "🎯 Tracking person @ (%.3f, %.3f) | Pan: %.1f°→%.1f° | Tilt: %.1f°→%.1f°",
                person.x, person.y, current_pan, new_pan, current_tilt, new_tilt);
        }
    }
    
    void scan_for_target() {
        // Simple scanning pattern: sweep pan back and forth
        double current_pan = turret_->getPanAngle();
        
        // Scan limits
        const double scan_left = 30.0;
        const double scan_right = 150.0;
        const double scan_step = 15.0;
        
        double new_pan;
        if (current_pan >= scan_right) {
            new_pan = current_pan - scan_step;  // Sweep left
            last_scan_direction_ = -1;
        } else if (current_pan <= scan_left) {
            new_pan = current_pan + scan_step;  // Sweep right
            last_scan_direction_ = 1;
        } else {
            // Continue in current direction
            new_pan = current_pan + (last_scan_direction_ * scan_step);
        }
        
        new_pan = std::clamp(new_pan, scan_left, scan_right);
       if (new_pan >= scan_right || new_pan <= scan_left) {
            last_scan_direction_ *= -1;
        }
        
        turret_->setPanAngle(new_pan);
        
        // Log occasionally
        static int scan_log_counter = 0;
        if (++scan_log_counter % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), "🔍 Scanning: Pan=%.1f°", new_pan);
        }
    }
    
    rclcpp::Subscription<two_towers::msg::DetectionArray>::SharedPtr subscription_;
    std::unique_ptr<Turret> turret_;
    int frames_without_detection_;
    bool tracking_active_;
    int last_scan_direction_ = 1;  // 1 = right, -1 = left
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<OrthancTrackerNode>();
        RCLCPP_INFO(node->get_logger(), "🔄 Orthanc tracker running. Press Ctrl+C to stop.");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("orthanc_tracker"), 
                     "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}