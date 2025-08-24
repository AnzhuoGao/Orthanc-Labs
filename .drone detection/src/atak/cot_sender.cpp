#include "atak/cot_sender.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>

namespace orthanc::atak {

CoTSender::CoTSender(const AtakConfig& config) 
    : config_(config) {
}

CoTSender::~CoTSender() {
    stop();
}

bool CoTSender::start() {
    if (running_.load()) {
        return false;
    }
    
    running_ = true;
    
    // Stub implementation - would normally connect to ATAK server
    connected_ = true; // Simulate successful connection
    
    // Start sender thread
    sender_thread_ = std::thread(&CoTSender::sender_thread_func, this);
    heartbeat_thread_ = std::thread(&CoTSender::heartbeat_thread_func, this);
    
    std::cout << "ATAK CoT sender started (stub)" << std::endl;
    return true;
}

void CoTSender::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_ = false;
    connected_ = false;
    
    // Notify sender thread
    queue_cv_.notify_all();
    
    // Join threads
    if (sender_thread_.joinable()) {
        sender_thread_.join();
    }
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
    
    std::cout << "ATAK CoT sender stopped (stub)" << std::endl;
}

void CoTSender::send_detections(const std::vector<orthanc::core::DroneDetection>& detections) {
    if (!connected_.load()) {
        return;
    }
    
    // Generate CoT messages for each detection
    for (const auto& detection : detections) {
        if (detection.confidence >= config_.min_confidence_threshold) {
            std::string cot_xml = generate_detection_cot(detection);
            queue_message(cot_xml);
        }
    }
}

void CoTSender::send_heartbeat() {
    if (!connected_.load()) {
        return;
    }
    
    std::string heartbeat_xml = generate_heartbeat_cot();
    queue_message(heartbeat_xml);
}

void CoTSender::queue_message(const std::string& xml_content) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    message_queue_.emplace(xml_content);
    
    // Update stats
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    stats_.messages_queued++;
    
    queue_cv_.notify_one();
}

void CoTSender::update_config(const AtakConfig& config) {
    config_ = config;
}

CoTSender::SendStats CoTSender::get_stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

// Private implementations
void CoTSender::sender_thread_func() {
    while (running_.load()) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] { return !message_queue_.empty() || !running_.load(); });
        
        if (!running_.load()) {
            break;
        }
        
        if (!message_queue_.empty()) {
            CoTMessage message = message_queue_.front();
            message_queue_.pop();
            lock.unlock();
            
            // Simulate sending message
            bool success = send_message(message);
            
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            if (success) {
                stats_.messages_sent++;
                stats_.last_successful_send = std::chrono::steady_clock::now();
            } else {
                stats_.messages_failed++;
            }
        }
    }
}

void CoTSender::heartbeat_thread_func() {
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(30));
        if (running_.load()) {
            send_heartbeat();
        }
    }
}

bool CoTSender::connect_to_server() {
    // Stub implementation - would create socket connection
    return true;
}

void CoTSender::disconnect_from_server() {
    // Stub implementation - would close socket
}

bool CoTSender::send_message(const CoTMessage& message) {
    // Stub implementation - would send via TCP socket
    std::cout << "Sending CoT message (stub): " << message.xml_content.substr(0, 100) << "..." << std::endl;
    return true;
}

std::string CoTSender::generate_detection_cot(const orthanc::core::DroneDetection& detection) const {
    auto gps_pos = world_to_gps(detection.position);
    
    std::ostringstream oss;
    oss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
    oss << "<event version=\"2.0\" uid=\"" << xml_escape(detection.uuid) << "\"";
    oss << " type=\"a-n-A-M-F-Q\" time=\"" << get_timestamp_iso() << "\"";
    oss << " start=\"" << get_timestamp_iso() << "\"";
    oss << " stale=\"" << get_timestamp_stale() << "\" how=\"h-g-i-g-o\">";
    oss << "<point lat=\"" << std::fixed << std::setprecision(8) << gps_pos.x << "\"";
    oss << " lon=\"" << std::fixed << std::setprecision(8) << gps_pos.y << "\"";
    oss << " hae=\"" << std::fixed << std::setprecision(2) << gps_pos.z << "\"";
    oss << " ce=\"" << std::fixed << std::setprecision(1) << detection.uncertainty << "\"";
    oss << " le=\"9999999.0\"/>";
    oss << "<detail>";
    oss << "<contact callsign=\"DRONE-" << detection.id << "\"/>";
    oss << "<__group name=\"Cyan\" role=\"Team Member\"/>";
    oss << "<track speed=\"" << std::fixed << std::setprecision(1);
    if (detection.velocity_valid) {
        float speed = std::sqrt(detection.velocity.x * detection.velocity.x + 
                               detection.velocity.y * detection.velocity.y);
        oss << speed;
    } else {
        oss << "0.0";
    }
    oss << "\" course=\"0.0\"/>";
    oss << "<sensor confidence=\"" << std::fixed << std::setprecision(2) << detection.confidence << "\"/>";
    oss << "</detail>";
    oss << "</event>";
    
    return oss.str();
}

std::string CoTSender::generate_heartbeat_cot() const {
    std::ostringstream oss;
    oss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
    oss << "<event version=\"2.0\" uid=\"" << config_.sensor_callsign << "-heartbeat\"";
    oss << " type=\"t-x-d-d\" time=\"" << get_timestamp_iso() << "\"";
    oss << " start=\"" << get_timestamp_iso() << "\"";
    oss << " stale=\"" << get_timestamp_stale() << "\" how=\"h-g-i-g-o\">";
    oss << "<point lat=\"" << std::fixed << std::setprecision(8) << config_.sensor_location.x << "\"";
    oss << " lon=\"" << std::fixed << std::setprecision(8) << config_.sensor_location.y << "\"";
    oss << " hae=\"" << std::fixed << std::setprecision(2) << config_.sensor_location.z << "\"";
    oss << " ce=\"1.0\" le=\"1.0\"/>";
    oss << "<detail>";
    oss << "<contact callsign=\"" << xml_escape(config_.sensor_callsign) << "\"/>";
    oss << "</detail>";
    oss << "</event>";
    
    return oss.str();
}

std::string CoTSender::get_timestamp_iso() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%S");
    oss << "." << std::setfill('0') << std::setw(3) << ms.count() << "Z";
    return oss.str();
}

std::string CoTSender::get_timestamp_stale() const {
    auto stale_time = std::chrono::system_clock::now() + std::chrono::seconds(300);
    auto time_t = std::chrono::system_clock::to_time_t(stale_time);
    
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%S.000Z");
    return oss.str();
}

orthanc::core::Point3D CoTSender::world_to_gps(const orthanc::core::Point3D& world_pos) const {
    // Stub implementation - would convert world coordinates to GPS
    // For now, just add offsets to sensor location
    return orthanc::core::Point3D(
        config_.sensor_location.x + world_pos.x * 0.00001f,
        config_.sensor_location.y + world_pos.y * 0.00001f,
        config_.sensor_location.z + world_pos.z
    );
}

std::string CoTSender::xml_escape(const std::string& text) const {
    std::string result;
    result.reserve(text.size() * 1.1);
    
    for (char c : text) {
        switch (c) {
            case '<': result += "&lt;"; break;
            case '>': result += "&gt;"; break;
            case '&': result += "&amp;"; break;
            case '"': result += "&quot;"; break;
            case '\'': result += "&apos;"; break;
            default: result += c; break;
        }
    }
    
    return result;
}

} // namespace orthanc::atak 