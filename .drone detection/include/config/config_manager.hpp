#pragma once

#include "core/fast_voxel_detector.hpp"
#include "rtsp/stream_processor.hpp"
#include "atak/cot_sender.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace orthanc::config {

struct SystemConfig {
    orthanc::core::DetectorConfig detector;
    orthanc::rtsp::ProcessingConfig processing;
    orthanc::atak::AtakConfig atak;
    std::vector<orthanc::rtsp::CameraConfig> cameras;
    
    // System-wide settings
    bool enable_test_mode{false};
    bool enable_performance_logging{true};
    std::string log_level{"INFO"};
    int performance_report_interval_sec{10};
};

class ConfigManager {
public:
    ConfigManager() = default;
    
    // Configuration loading/saving
    bool load_from_file(const std::string& filename);
    bool save_to_file(const std::string& filename) const;
    bool load_from_json(const nlohmann::json& json);
    nlohmann::json to_json() const;
    
    // Configuration access
    const SystemConfig& get_config() const { return config_; }
    SystemConfig& get_config() { return config_; }
    
    // Individual component config getters
    const orthanc::core::DetectorConfig& get_detector_config() const { return config_.detector; }
    const orthanc::rtsp::ProcessingConfig& get_processing_config() const { return config_.processing; }
    const orthanc::atak::AtakConfig& get_atak_config() const { return config_.atak; }
    const std::vector<orthanc::rtsp::CameraConfig>& get_camera_configs() const { return config_.cameras; }
    
    // Validation
    bool validate_config() const;
    std::vector<std::string> get_validation_errors() const;
    
    // Default configuration generation
    static SystemConfig create_default_config();
    static SystemConfig create_test_config();

private:
    SystemConfig config_;
    
    // JSON conversion helpers
    static void to_json(nlohmann::json& j, const orthanc::core::DetectorConfig& config);
    static void from_json(const nlohmann::json& j, orthanc::core::DetectorConfig& config);
    
    static void to_json(nlohmann::json& j, const orthanc::rtsp::ProcessingConfig& config);
    static void from_json(const nlohmann::json& j, orthanc::rtsp::ProcessingConfig& config);
    
    static void to_json(nlohmann::json& j, const orthanc::atak::AtakConfig& config);
    static void from_json(const nlohmann::json& j, orthanc::atak::AtakConfig& config);
    
    static void to_json(nlohmann::json& j, const orthanc::rtsp::CameraConfig& config);
    static void from_json(const nlohmann::json& j, orthanc::rtsp::CameraConfig& config);
    
    static void to_json(nlohmann::json& j, const orthanc::core::Point3D& point);
    static void from_json(const nlohmann::json& j, orthanc::core::Point3D& point);
};

// Utility functions for configuration management
std::string get_default_config_path();
std::string get_user_config_path();
bool create_config_directories();

} // namespace orthanc::config 