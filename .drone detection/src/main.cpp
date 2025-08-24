#include "core/fast_voxel_detector.hpp"
#include "rtsp/stream_processor.hpp"
#include "atak/cot_sender.hpp"
#include "config/config_manager.hpp"

#include <iostream>
#include <memory>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

// Global shutdown flag
std::atomic<bool> g_running{true};

void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ". Shutting down gracefully..." << std::endl;
    g_running = false;
}

void print_banner() {
    std::cout << R"(
    ╔═══════════════════════════════════════════════════════════════╗
    ║                 Orthanc Technologies                          ║
    ║              Fast Drone Detection System                      ║
    ║                     Version 1.0.0                            ║
    ╚═══════════════════════════════════════════════════════════════╝
    
    Optimized for high-performance real-time drone detection
    Features:
    • SIMD-accelerated voxel processing
    • Multi-camera RTSP stream support
    • Real-time ATAK integration
    • Advanced clustering algorithms
    • Hardware-optimized implementations
    
)" << std::endl;
}

void print_performance_stats(
    const std::shared_ptr<orthanc::core::FastVoxelDetector>& detector,
    const std::shared_ptr<orthanc::rtsp::StreamProcessor>& processor,
    const std::shared_ptr<orthanc::atak::CoTSender>& cot_sender) {
    
    auto detector_stats = detector->get_performance_stats();
    auto atak_stats = cot_sender->get_stats();
    
    std::cout << "\n=== Performance Statistics ===" << std::endl;
    std::cout << "Detector:" << std::endl;
    std::cout << "  Last processing time: " << detector_stats.last_processing_time.count() << " μs" << std::endl;
    std::cout << "  Total voxels processed: " << detector_stats.total_voxels_processed << std::endl;
    std::cout << "  Active tracks: " << detector_stats.active_tracks << std::endl;
    std::cout << "  Clusters found: " << detector_stats.clusters_found << std::endl;
    
    std::cout << "ATAK Sender:" << std::endl;
    std::cout << "  Messages sent: " << atak_stats.messages_sent << std::endl;
    std::cout << "  Messages failed: " << atak_stats.messages_failed << std::endl;
    std::cout << "  Messages queued: " << atak_stats.messages_queued << std::endl;
    std::cout << "  Connected: " << (cot_sender->is_connected() ? "Yes" : "No") << std::endl;
    
    auto detections = detector->get_detections();
    std::cout << "Current detections: " << detections.size() << std::endl;
    for (const auto& detection : detections) {
        std::cout << "  Drone " << detection.id << ": "
                  << "pos=(" << detection.position.x << ", " << detection.position.y << ", " << detection.position.z << ") "
                  << "confidence=" << detection.confidence << std::endl;
    }
    std::cout << "================================" << std::endl;
}

void simulate_test_drone(const std::shared_ptr<orthanc::core::FastVoxelDetector>& detector) {
    static float t = 0.0f;
    t += 0.1f;
    
    detector->clear_voxels();
    
    // Simulate a drone moving in a circular pattern
    const auto& config = detector->get_config();
    float center_x = config.grid_size[0] / 2.0f;
    float center_y = config.grid_size[1] / 2.0f;
    float center_z = config.grid_size[2] / 2.0f;
    
    float radius = std::min(config.grid_size[0], config.grid_size[1]) / 6.0f;
    float drone_x = center_x + radius * std::cos(t);
    float drone_y = center_y + radius * std::sin(t);
    float drone_z = center_z + 5.0f * std::sin(t / 2.0f);
    
    // Create a drone-shaped cluster
    for (int dx = -2; dx <= 2; dx++) {
        for (int dy = -2; dy <= 2; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
                float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (distance > 2.5f) continue;
                
                int vx = static_cast<int>(drone_x + dx);
                int vy = static_cast<int>(drone_y + dy);
                int vz = static_cast<int>(drone_z + dz);
                
                if (vx >= 0 && vx < config.grid_size[0] &&
                    vy >= 0 && vy < config.grid_size[1] &&
                    vz >= 0 && vz < config.grid_size[2]) {
                    
                    float intensity = 1.0f - (distance / 3.0f);
                    if (intensity < 0.2f) intensity = 0.2f;
                    
                    detector->update_voxel(vx, vy, vz, intensity);
                }
            }
        }
    }
    
    // Trigger processing
    std::vector<float> dummy_grid(config.grid_size[0] * config.grid_size[1] * config.grid_size[2], 0.0f);
    detector->process_voxel_data(dummy_grid.data(), dummy_grid.size());
}

int main(int argc, char* argv[]) {
    // Install signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    print_banner();
    
    // Parse command line arguments
    std::string config_file = "config/fast_drone_detector.json";
    if (argc > 1) {
        config_file = argv[1];
    }
    
    // Load configuration
    orthanc::config::ConfigManager config_manager;
    if (!config_manager.load_from_file(config_file)) {
        std::cout << "Failed to load config file: " << config_file << std::endl;
        std::cout << "Using default configuration..." << std::endl;
        config_manager.get_config() = orthanc::config::ConfigManager::create_default_config();
        
        // Save default config for future use
        if (!config_manager.save_to_file(config_file)) {
            std::cout << "Warning: Could not save default config to " << config_file << std::endl;
        } else {
            std::cout << "Default configuration saved to " << config_file << std::endl;
        }
    }
    
    const auto& system_config = config_manager.get_config();
    
    // Validate configuration
    if (!config_manager.validate_config()) {
        std::cerr << "Configuration validation failed:" << std::endl;
        for (const auto& error : config_manager.get_validation_errors()) {
            std::cerr << "  - " << error << std::endl;
        }
        return 1;
    }
    
    std::cout << "Configuration loaded successfully." << std::endl;
    std::cout << "Test mode: " << (system_config.enable_test_mode ? "Enabled" : "Disabled") << std::endl;
    std::cout << "Log level: " << system_config.log_level << std::endl;
    std::cout << "Cameras configured: " << system_config.cameras.size() << std::endl;
    
    try {
        // Initialize core components
        auto detector = std::make_shared<orthanc::core::FastVoxelDetector>(system_config.detector);
        auto stream_processor = std::make_shared<orthanc::rtsp::StreamProcessor>(system_config.processing);
        auto cot_sender = std::make_shared<orthanc::atak::CoTSender>(system_config.atak);
        
        std::cout << "\nStarting ATAK integration..." << std::endl;
        if (!cot_sender->start()) {
            std::cerr << "Warning: Failed to connect to ATAK server at " 
                      << system_config.atak.server_ip << ":" << system_config.atak.server_port << std::endl;
            std::cerr << "Continuing without ATAK integration..." << std::endl;
        } else {
            std::cout << "ATAK integration started successfully." << std::endl;
        }
        
        // Set up frame callback for debugging (optional)
        if (system_config.log_level == "DEBUG") {
            stream_processor->set_frame_callback([](int camera_id, const cv::Mat& frame, const cv::Mat& motion_mask) {
                std::cout << "Camera " << camera_id << ": frame " << frame.cols << "x" << frame.rows 
                          << ", motion pixels: " << cv::countNonZero(motion_mask) << std::endl;
            });
        }
        
        // Configure cameras
        for (const auto& camera_config : system_config.cameras) {
            if (!stream_processor->add_camera(camera_config)) {
                std::cerr << "Warning: Failed to configure camera " << camera_config.camera_id 
                          << " (" << camera_config.rtsp_url << ")" << std::endl;
            } else {
                std::cout << "Camera " << camera_config.camera_id << " configured." << std::endl;
            }
        }
        
        // Start processing
        if (!system_config.enable_test_mode && !system_config.cameras.empty()) {
            std::cout << "\nStarting real-time stream processing..." << std::endl;
            if (!stream_processor->start_processing(detector)) {
                std::cerr << "Failed to start stream processing!" << std::endl;
                return 1;
            }
            std::cout << "Stream processing started successfully." << std::endl;
        }
        
        std::cout << "\nDrone detection system is running." << std::endl;
        std::cout << "Press Ctrl+C to stop.\n" << std::endl;
        
        // Main processing loop
        auto last_stats_time = std::chrono::steady_clock::now();
        auto last_cot_send_time = std::chrono::steady_clock::now();
        
        while (g_running) {
            auto now = std::chrono::steady_clock::now();
            
            // Test mode simulation
            if (system_config.enable_test_mode) {
                simulate_test_drone(detector);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // Send detections to ATAK
            auto elapsed_cot = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cot_send_time);
            if (elapsed_cot >= system_config.atak.update_interval) {
                auto detections = detector->get_detections();
                if (!detections.empty() && cot_sender->is_connected()) {
                    cot_sender->send_detections(detections);
                }
                last_cot_send_time = now;
            }
            
            // Performance statistics
            if (system_config.enable_performance_logging) {
                auto elapsed_stats = std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_time);
                if (elapsed_stats.count() >= system_config.performance_report_interval_sec) {
                    print_performance_stats(detector, stream_processor, cot_sender);
                    last_stats_time = now;
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        // Shutdown sequence
        std::cout << "\nShutting down system components..." << std::endl;
        
        if (stream_processor->is_processing()) {
            stream_processor->stop_processing();
            std::cout << "Stream processing stopped." << std::endl;
        }
        
        if (cot_sender->is_connected()) {
            cot_sender->stop();
            std::cout << "ATAK integration stopped." << std::endl;
        }
        
        // Final statistics
        print_performance_stats(detector, stream_processor, cot_sender);
        
        std::cout << "\nSystem shutdown complete." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 