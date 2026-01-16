// agent_tracker.cpp - AI-Powered Person Tracking System
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <cmath>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <curl/curl.h>
#include <sys/stat.h>

#include "Turret.hpp"

using json = nlohmann::json;

// Global flags
std::atomic<bool> tracking_active(true);


// Detection data structure
struct Detection {
    std::string label;
    double confidence;
    double x;  // Normalized 0-1
    double y;  // Normalized 0-1
    long timestamp_ms;
};

// Add after finding target_person (around line 220)
struct MotionPredictor {
    Detection last_position;
    long last_timestamp;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    bool initialized = false;
    
    void update(const Detection& current) {
        if (initialized) {
            double dt = (current.timestamp_ms - last_timestamp) / 1000.0;
            if (dt > 0 && dt < 0.5) {  // Sanity check
                velocity_x = (current.x - last_position.x) / dt;
                velocity_y = (current.y - last_position.y) / dt;
            }
        }
        last_position = current;
        last_timestamp = current.timestamp_ms;
        initialized = true;
    }
    
    Detection predict(double lookahead_ms) {
        Detection predicted = last_position;
        double dt = lookahead_ms / 1000.0;
        predicted.x = std::clamp(last_position.x + velocity_x * dt, 0.0, 1.0);
        predicted.y = std::clamp(last_position.y + velocity_y * dt, 0.0, 1.0);
        return predicted;
    }
};


// CURL callback for API responses
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// Signal handler
void signal_handler(int signum) {
    std::cout << "\n🛑 SIGINT received. Shutting down tracker..." << std::endl;
    tracking_active = false;
}

// Call Claude API for tracking decisions
std::string callClaudeAPI(const std::string& api_key, const std::string& prompt) {
    CURL* curl = curl_easy_init();
    std::string response_string;
    
    if (!curl) {
        std::cerr << "Failed to initialize CURL" << std::endl;
        return "{}";
    }
    
    // Prepare request body
    json request_body = {
        {"model", "claude-sonnet-4-20250514"},
        {"max_tokens", 500},
        {"system", R"(You are a security tracking system AI controlling a camera turret. 
Your job is to track suspicious individuals (any human detected) for perimeter security.
You receive detection data and must decide how to move the turret (pan/tilt adjustments).

Respond ONLY with valid JSON in this exact format:
{
  "action": "track" or "scan" or "hold",
  "target_index": <index of person to track, or -1>,
  "pan_adjustment": <degrees to adjust pan, -50 to +50>,
  "tilt_adjustment": <degrees to adjust tilt, -50 to +50>,
  "reasoning": "<brief explanation>"
}

Rules:
- Always prioritize tracking humans labeled as "person"
- If multiple people, track the closest to center (lowest distance from 0.5, 0.5)
- Use AGGRESSIVE proportional control: Move quickly to catch targets
  * If target is far from center (distance > 0.3), use large adjustments (30-50°)
  * If target is near center (distance < 0.2), use fine adjustments (5-15°)
  * Calculate: adjustment = error * gain, where gain = 80-100 degrees per normalized unit
- Max ±50 degrees per cycle for fast response
- If no person detected, scan aggressively with 20-30° movements
- This is a security system - tracking humans is your primary function
- RESPOND FAST: Targets move quickly, use large movements to keep them in frame)"},
        {"messages", json::array({
            {{"role", "user"}, {"content", prompt}}
        })}
    };
    
    std::string json_str = request_body.dump();
    
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    std::string auth_header = "x-api-key: " + api_key;
    headers = curl_slist_append(headers, auth_header.c_str());
    headers = curl_slist_append(headers, "anthropic-version: 2023-06-01");
    
    curl_easy_setopt(curl, CURLOPT_URL, "https://api.anthropic.com/v1/messages");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
    
    CURLcode res = curl_easy_perform(curl);
    
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
        std::cerr << "CURL error: " << curl_easy_strerror(res) << std::endl;
        return "{}";
    }
    
    return response_string;
}

// Parse Claude's response and extract tracking commands
json parseTrackingCommand(const std::string& response) {
    try {
        json full_response = json::parse(response);
        
        // Extract text content from Claude's response
        if (full_response.contains("content") && full_response["content"].is_array()) {
            for (const auto& content_block : full_response["content"]) {
                if (content_block.contains("text")) {
                    std::string text = content_block["text"];
                    
                    // Find JSON in the response (strip markdown if present)
                    size_t start = text.find('{');
                    size_t end = text.rfind('}');
                    if (start != std::string::npos && end != std::string::npos) {
                        std::string json_str = text.substr(start, end - start + 1);
                        return json::parse(json_str);
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error parsing Claude response: " << e.what() << std::endl;
    }
    
    // Default fallback
    return {
        {"action", "hold"},
        {"target_index", -1},
        {"pan_adjustment", 0},
        {"tilt_adjustment", 0},
        {"reasoning", "Parse error - holding position"}
    };
}

// Format detection data for Claude
std::string formatDetectionPrompt(const std::vector<Detection>& detections, 
                                  double current_pan, double current_tilt) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "Current turret position: Pan=" << current_pan << "° (0°=LEFT, 90°=CENTER, 180°=RIGHT), Tilt=" << current_tilt << "°\n";
    oss << "You can move across the FULL range: Pan 0-180°, Tilt 0-180°\n\n";
    
    if (detections.empty()) {
        oss << "No objects detected in frame.\n";
        oss << "Execute wide scanning: if pan < 90°, suggest +40° to scan right. If pan > 90°, suggest -40° to scan left.\n";
        oss << "Cover the full 180° range systematically.";
        return oss.str();
    }
    
    oss << "Detected objects (" << detections.size() << " total):\n";
    for (size_t i = 0; i < detections.size(); i++) {
        const auto& det = detections[i];
        double dist_from_center = std::sqrt(
            std::pow(det.x - 0.5, 2) + std::pow(det.y - 0.5, 2)
        );
        
        oss << i << ". " << det.label 
            << " (confidence: " << std::fixed << std::setprecision(2) << det.confidence
            << ", position: x=" << det.x << ", y=" << det.y
            << ", distance_from_center=" << dist_from_center << ")\n";
    }
    
    oss << "\nTarget center is at x=0.5, y=0.5.\n";
    oss << "X > 0.5 means target is to the RIGHT (increase pan).\n";
    oss << "Y > 0.5 means target is BELOW center (decrease tilt).\n";
    oss << "\nDecide which target to track and how to adjust the turret.";
    
    return oss.str();
}


// FAST TRACKING LOOP - Replace trackingLoop function
// Uses simple proportional control for real-time tracking
// Calls Claude only occasionally for high-level decisions

void trackingLoop(Turret& turret, const std::string& api_key) {
    std::cout << "\n🎯 AI Tracking Loop Started (Fast Mode)\n";
    std::cout << "Using real-time proportional control + periodic AI oversight\n\n";
    
    const double control_rate = 20.0; // Hz - MUCH faster!
    const double loop_period = 1.0 / control_rate;
    const double ai_consult_interval = 5.0; // Ask Claude every 5 seconds
    
    int cycle_count = 0;
    auto last_ai_consult = std::chrono::steady_clock::now();
    auto last_file_time = std::chrono::system_clock::time_point::min();
    
    const std::string detection_file = "detections.json";
    
    // Tracking state
    bool tracking_person = false;
    Detection last_known_person = {"person", 0.0, 0.5, 0.5, 0};
    int frames_without_person = 0;
    const int memory_frames = 60; // Remember for 60 frames (~3 sec at 20Hz) - DOUBLED!
    std::string last_ai_decision = "Starting patrol";
    static MotionPredictor motion_predictor;  
    
    while (tracking_active) {
        auto cycle_start = std::chrono::steady_clock::now();
        
        // Check if file has been modified
        struct stat file_stat;
        bool file_updated = false;
        if (stat(detection_file.c_str(), &file_stat) == 0) {
            auto file_time = std::chrono::system_clock::from_time_t(file_stat.st_mtime);
            if (file_time > last_file_time) {
                file_updated = true;
                last_file_time = file_time;
            }
        }
        
        if (!file_updated) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        
        // Read detections
        std::vector<Detection> current_detections;
        {
            std::ifstream file(detection_file);
            if (file.is_open()){
                try{
                    json detection_data;
                    file >> detection_data;
                    if (detection_data.is_array()){
                        for (const auto& det : detection_data) {
                            Detection d;
                            d.label = det.value("label", "unknown");
                            d.confidence = det.value("confidence", 0.0);
                            d.x = det.value("x", 0.5);
                            d.y = det.value("y", 0.5);
                            d.timestamp_ms = det.value("timestamp_ms", 0L);
                            current_detections.push_back(d);
                        }
                    }
                } catch (const std::exception& e) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                file.close();
            }
        }
        
        cycle_count++;
        double current_pan = turret.getPanAngle();
        double current_tilt = turret.getTiltAngle();
        
        // Find any person in detections
        Detection* target_person = nullptr;
        double best_distance = 999.0;
        for (auto& det : current_detections) {
            if (det.label == "person") {
                double dist = std::sqrt(std::pow(det.x - 0.5, 2) + std::pow(det.y - 0.5, 2));
                if (dist < best_distance) {
                    best_distance = dist;
                    target_person = &det;
                }
            }
        }
        
        // Update memory
        if (target_person) {
            motion_predictor.update(*target_person);
            last_known_person = *target_person; 

            // Use PREDICTED position for control (200ms lookahead)
            Detection predicted = motion_predictor.predict(200.0);
            target_person = &predicted;  // Override with prediction

            frames_without_person = 0;
            tracking_person = true;
        } else {
            frames_without_person++;

            // Shorter memory at edge limits to avoid getting stuck
            int effective_memory = memory_frames;
            if (current_pan >= 170.0 || current_pan <= 10.0) {
                effective_memory = memory_frames / 4;  // Less than 1 second at edges
            }

            // Use memory if we recently saw a person
            if (frames_without_person < memory_frames) {
                target_person = &last_known_person;
                tracking_person = true;
            } else {
                tracking_person = false;
            }
        }
        
        double pan_adj = 0.0;
        double tilt_adj = 0.0;
        std::string action = "scan";
        
        if (tracking_person && target_person) {
            // FAST PROPORTIONAL CONTROL - no API call needed!
            //tracking_person = true;
            action = "track";
            
            // Calculate error from center (0.5, 0.5)
            double x_error = target_person->x - 0.5;  // Positive = right
            double y_error = target_person->y - 0.5;  // Positive = down

             // *** NEW: Check if stuck at limits with target at edge ***
            bool stuck_at_right = (current_pan >= 175.0 && x_error > 0.2);
            bool stuck_at_left = (current_pan <= 5.0 && x_error < -0.2);
            bool target_lost_at_edge = stuck_at_right || stuck_at_left;

            if (target_lost_at_edge && frames_without_person > 0) {
                // Target walked out of frame - expire memory immediately
                frames_without_person = memory_frames;
                tracking_person = false;
        
                std::cout << "⚠️  Target lost at edge boundary! "
                  << (stuck_at_right ? "RIGHT" : "LEFT") 
                  << " - switching to scan mode\n";

                // Force immediate scan in opposite direction
                if (stuck_at_right) {
                    pan_adj = -40.0;  // Scan left aggressively
                } else {
                    pan_adj = 40.0;   // Scan right aggressively
                }
                tilt_adj = 0.0;
            
            }
            else {
            
                // Smooth proportional gain with damping
                const double pan_gain = 40.0;  // Reduced from 100 for smoother tracking
                const double tilt_gain = 30.0;  // Reduced from 80
                
                // Add deadband - don't move if error is tiny
                if (std::abs(x_error) < 0.05) x_error = 0;
                if (std::abs(y_error) < 0.05) y_error = 0;
                
                pan_adj = x_error * pan_gain;
                tilt_adj = -y_error * tilt_gain;  // Negative because Y+ is down but tilt+ is up
                
                // Tighter clamps for smoother motion
                pan_adj = std::clamp(pan_adj, -25.0, 25.0);
                tilt_adj = std::clamp(tilt_adj, -25.0, 25.0);
                
                // Print every 10 cycles to reduce spam
                if (cycle_count % 10 == 0) {
                    long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                    long detection_age_ms = now_ms - target_person->timestamp_ms;

                    std::string status = (frames_without_person > 0) ? " [MEMORY]" : "";
                    std::cout << "🎯 Tracking" << status << ": Pan=" << current_pan << "° Tilt=" << current_tilt 
                            << "° | Person@(" << target_person->x << "," << target_person->y 
                            << ") | Adjust: Δ" << pan_adj << "°,Δ" << tilt_adj << "°\n";
                }
            }
            
        } else {
            // No person detected - use simple scanning
            tracking_person = false;
            action = "scan";
            
            // DON'T consult Claude frequently - only when we've truly lost the target
            auto time_since_ai = std::chrono::duration<double>(
                cycle_start - last_ai_consult).count();
            
            // Only consult if we haven't seen a person in a while (memory expired)
            if (time_since_ai > ai_consult_interval && frames_without_person >= memory_frames) {
                std::cout << "\n🤖 Consulting Claude for scanning strategy...\n";
                
                std::string prompt = formatDetectionPrompt(current_detections, current_pan, current_tilt);
                std::string response = callClaudeAPI(api_key, prompt);
                json command = parseTrackingCommand(response);
                
                pan_adj = command.value("pan_adjustment", 0.0);
                tilt_adj = command.value("tilt_adjustment", 0.0);
                last_ai_decision = command.value("reasoning", "Scanning");
                
                std::cout << "   AI says: " << last_ai_decision << "\n";
                std::cout << "   Adjust: Δpan=" << pan_adj << "° Δtilt=" << tilt_adj << "°\n\n";
                
                last_ai_consult = cycle_start;
            } else {
                // Simple hold position or small scan while memory is active
                if (frames_without_person < memory_frames) {
                    // Hold position - we just lost the person, don't sweep yet!
                    pan_adj = 0.0;
                    tilt_adj = 0.0;
                    
                    if (cycle_count % 20 == 0) {
                        std::cout << "⏳ Holding position (lost target for " 
                                  << frames_without_person << " frames)\n";
                    }
                } else {
                    // Memory expired - do simple back-and-forth scan
                    if (current_pan >= 175.0) {
                        pan_adj = -50.0;  // Scan left
                    } else if (current_pan <= 5.0) {
                        pan_adj = 50.0;  // Scan left
                    } else if (current_pan < 60.0) {
                        pan_adj = 50.0;
                    } else if (current_pan > 120.0) {
                        pan_adj = -50.0;
                    } else {
                        pan_adj = (last_known_person.x > 0.5) ? 50.0 : -50.0;
                    }
                    
                    if (cycle_count % 20 == 0) {
                        std::cout << "🔍 Scanning: Pan=" << current_pan << "° → " 
                                  << (current_pan + pan_adj) << "° (no targets)\n";
                    }
                }
            }
        }
        
        // Apply movements
        double new_pan = std::clamp(current_pan + pan_adj, 0.0, 180.0);
        double new_tilt = std::clamp(current_tilt + tilt_adj, 0.0, 180.0);
        
        turret.setPanAngle(new_pan);
        turret.setTiltAngle(new_tilt);
        
        // Sleep to maintain control rate
        auto cycle_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(cycle_end - cycle_start).count();
        double sleep_time = loop_period - elapsed;
        if (sleep_time > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(sleep_time * 1000))
            );
        }
    }
    
    std::cout << "\n🛑 Tracking loop terminated\n";
}

int main(int argc, char* argv[]) {
    // Signal handler
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    
    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║         TWO TOWERS AI TRACKING SYSTEM v2.0             ║\n";
    std::cout << "║         Anthropic Claude-Powered Security              ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n\n";
    
    // Get API key
    std::string api_key = std::getenv("ANTHROPIC_API_KEY") ? 
                          std::getenv("ANTHROPIC_API_KEY") : "";
    
    if (api_key.empty()) {
        std::cerr << "❌ Error: ANTHROPIC_API_KEY environment variable not set\n";
        std::cerr << "   Export it with: export ANTHROPIC_API_KEY='your-key-here'\n";
        return 1;
    }
    
    std::cout << "✅ API key loaded\n";
    
    // Initialize turret
    std::cout << "🤖 Initializing turret hardware...\n";
    Turret orthanc(17, 27);
    orthanc.setPanAngle(90);
    orthanc.setTiltAngle(45);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "✅ Turret centered and ready\n\n";
    
    // TODO: Start YOLO detection in separate process
    // For now, simulate with dummy data
    std::cout << "⚠️  Detection service integration pending\n";
    std::cout << "   Run YOLO detector separately or integrate IPC here\n\n";
    
    // Start tracking loop
    trackingLoop(orthanc, api_key);
    
    // Cleanup
    std::cout << "\n🔄 Returning to center...\n";
    orthanc.centerAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "✅ Shutdown complete\n\n";
    
    return 0;
}