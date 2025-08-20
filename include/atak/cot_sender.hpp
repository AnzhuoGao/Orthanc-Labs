#pragma once

#include "core/fast_voxel_detector.hpp"
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>

namespace orthanc::atak {

struct AtakConfig {
    std::string server_ip{"127.0.0.1"};
    int server_port{8087};
    std::string sensor_callsign{"DRONE-DETECTOR-01"};
    orthanc::core::Point3D sensor_location{38.8895, -77.0352, 0.0}; // lat, lon, alt
    
    // Transmission settings
    std::chrono::milliseconds update_interval{1000};
    float min_confidence_threshold{0.3f};
    bool send_velocity_data{true};
    bool compress_messages{false};
    
    // Network settings
    int connection_timeout_ms{5000};
    int send_timeout_ms{1000};
    int max_retry_attempts{3};
};

struct CoTMessage {
    std::string xml_content;
    std::chrono::steady_clock::time_point created_time;
    int retry_count{0};
    
    CoTMessage() = default;
    explicit CoTMessage(std::string content) 
        : xml_content(std::move(content)), created_time(std::chrono::steady_clock::now()) {}
};

class CoTSender {
public:
    explicit CoTSender(const AtakConfig& config = {});
    ~CoTSender();
    
    // Non-copyable, movable
    CoTSender(const CoTSender&) = delete;
    CoTSender& operator=(const CoTSender&) = delete;
    CoTSender(CoTSender&&) = default;
    CoTSender& operator=(CoTSender&&) = default;
    
    // Connection management
    bool start();
    void stop();
    bool is_connected() const { return connected_.load(); }
    
    // Message sending
    void send_detections(const std::vector<orthanc::core::DroneDetection>& detections);
    void send_heartbeat();
    void queue_message(const std::string& xml_content);
    
    // Configuration
    void update_config(const AtakConfig& config);
    const AtakConfig& get_config() const { return config_; }
    
    // Statistics
    struct SendStats {
        size_t messages_sent{0};
        size_t messages_failed{0};
        size_t messages_queued{0};
        std::chrono::microseconds average_send_time{0};
        std::chrono::steady_clock::time_point last_successful_send;
    };
    SendStats get_stats() const;

private:
    AtakConfig config_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    
    // Network components
    int socket_fd_{-1};
    std::thread sender_thread_;
    std::thread heartbeat_thread_;
    
    // Message queue
    std::queue<CoTMessage> message_queue_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    SendStats stats_;
    
    // Thread functions
    void sender_thread_func();
    void heartbeat_thread_func();
    
    // Network functions
    bool connect_to_server();
    void disconnect_from_server();
    bool send_message(const CoTMessage& message);
    
    // CoT message generation
    std::string generate_detection_cot(const orthanc::core::DroneDetection& detection) const;
    std::string generate_heartbeat_cot() const;
    
    // Utility functions
    std::string get_timestamp_iso() const;
    std::string get_timestamp_stale() const;
    orthanc::core::Point3D world_to_gps(const orthanc::core::Point3D& world_pos) const;
    
    // XML utilities
    std::string xml_escape(const std::string& text) const;
};

} // namespace orthanc::atak 