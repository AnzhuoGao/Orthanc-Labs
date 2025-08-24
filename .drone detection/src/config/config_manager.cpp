#include "config/config_manager.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>

namespace orthanc::config {

bool ConfigManager::load_from_file(const std::string& filename) {
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        nlohmann::json json;
        file >> json;
        return load_from_json(json);
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading config from " << filename << ": " << e.what() << std::endl;
        return false;
    }
}

bool ConfigManager::save_to_file(const std::string& filename) const {
    try {
        // Create directory if it doesn't exist
        std::filesystem::path file_path(filename);
        std::filesystem::create_directories(file_path.parent_path());
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        nlohmann::json json = to_json();
        file << json.dump(2);
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error saving config to " << filename << ": " << e.what() << std::endl;
        return false;
    }
}

bool ConfigManager::load_from_json(const nlohmann::json& json) {
    try {
        // Load detector config
        if (json.contains("detector")) {
            from_json(json["detector"], config_.detector);
        }
        
        // Load processing config
        if (json.contains("processing")) {
            from_json(json["processing"], config_.processing);
        }
        
        // Load ATAK config
        if (json.contains("atak")) {
            from_json(json["atak"], config_.atak);
        }
        
        // Load camera configs
        if (json.contains("cameras")) {
            config_.cameras.clear();
            for (const auto& camera_json : json["cameras"]) {
                orthanc::rtsp::CameraConfig camera;
                from_json(camera_json, camera);
                config_.cameras.push_back(camera);
            }
        }
        
        // Load system config
        if (json.contains("system")) {
            const auto& sys = json["system"];
            if (sys.contains("enable_test_mode")) {
                config_.enable_test_mode = sys["enable_test_mode"];
            }
            if (sys.contains("enable_performance_logging")) {
                config_.enable_performance_logging = sys["enable_performance_logging"];
            }
            if (sys.contains("log_level")) {
                config_.log_level = sys["log_level"];
            }
            if (sys.contains("performance_report_interval_sec")) {
                config_.performance_report_interval_sec = sys["performance_report_interval_sec"];
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing JSON config: " << e.what() << std::endl;
        return false;
    }
}

nlohmann::json ConfigManager::to_json() const {
    nlohmann::json json;
    
    // Detector config
    to_json(json["detector"], config_.detector);
    
    // Processing config
    to_json(json["processing"], config_.processing);
    
    // ATAK config
    to_json(json["atak"], config_.atak);
    
    // Camera configs
    json["cameras"] = nlohmann::json::array();
    for (const auto& camera : config_.cameras) {
        nlohmann::json camera_json;
        to_json(camera_json, camera);
        json["cameras"].push_back(camera_json);
    }
    
    // System config
    json["system"]["enable_test_mode"] = config_.enable_test_mode;
    json["system"]["enable_performance_logging"] = config_.enable_performance_logging;
    json["system"]["log_level"] = config_.log_level;
    json["system"]["performance_report_interval_sec"] = config_.performance_report_interval_sec;
    
    return json;
}

bool ConfigManager::validate_config() const {
    auto errors = get_validation_errors();
    return errors.empty();
}

std::vector<std::string> ConfigManager::get_validation_errors() const {
    std::vector<std::string> errors;
    
    // Validate detector config
    if (config_.detector.voxel_size <= 0) {
        errors.push_back("Detector voxel size must be positive");
    }
    
    if (config_.detector.grid_size[0] <= 0 || config_.detector.grid_size[1] <= 0 || config_.detector.grid_size[2] <= 0) {
        errors.push_back("Detector grid size must be positive in all dimensions");
    }
    
    // Validate ATAK config
    if (config_.atak.server_port <= 0 || config_.atak.server_port > 65535) {
        errors.push_back("ATAK server port must be between 1 and 65535");
    }
    
    // Validate cameras
    for (size_t i = 0; i < config_.cameras.size(); ++i) {
        const auto& camera = config_.cameras[i];
        if (camera.rtsp_url.empty()) {
            errors.push_back("Camera " + std::to_string(i) + " has empty RTSP URL");
        }
    }
    
    return errors;
}

SystemConfig ConfigManager::create_default_config() {
    SystemConfig config;
    
    // Default detector settings
    config.detector.grid_size = {200, 200, 100};
    config.detector.voxel_size = 0.2f;
    config.detector.grid_origin = orthanc::core::Point3D(-20.0f, -20.0f, 0.0f);
    
    // Default processing settings  
    config.processing.motion_threshold = 15.0f;
    config.processing.frame_skip = 1;
    
    // Default ATAK settings
    config.atak.server_ip = "127.0.0.1";
    config.atak.server_port = 8087;
    
    // Default system settings
    config.enable_test_mode = true;
    config.enable_performance_logging = true;
    config.log_level = "INFO";
    
    return config;
}

SystemConfig ConfigManager::create_test_config() {
    auto config = create_default_config();
    config.enable_test_mode = true;
    config.log_level = "DEBUG";
    config.performance_report_interval_sec = 5;
    return config;
}

// JSON conversion implementations
void ConfigManager::to_json(nlohmann::json& j, const orthanc::core::DetectorConfig& config) {
    j["grid_size"] = config.grid_size;
    j["voxel_size"] = config.voxel_size;
    to_json(j["grid_origin"], config.grid_origin);
    j["intensity_threshold"] = config.intensity_threshold;
    j["cluster_distance"] = config.cluster_distance;
    j["min_voxels_per_cluster"] = config.min_voxels_per_cluster;
    j["max_voxels_per_cluster"] = config.max_voxels_per_cluster;
    j["max_tracking_distance"] = config.max_tracking_distance;
    j["max_tracking_age_ms"] = config.max_tracking_age.count();
    j["use_simd"] = config.use_simd;
    j["worker_threads"] = config.worker_threads;
}

void ConfigManager::from_json(const nlohmann::json& j, orthanc::core::DetectorConfig& config) {
    if (j.contains("grid_size")) config.grid_size = j["grid_size"];
    if (j.contains("voxel_size")) config.voxel_size = j["voxel_size"];
    if (j.contains("grid_origin")) from_json(j["grid_origin"], config.grid_origin);
    if (j.contains("intensity_threshold")) config.intensity_threshold = j["intensity_threshold"];
    if (j.contains("cluster_distance")) config.cluster_distance = j["cluster_distance"];
    if (j.contains("min_voxels_per_cluster")) config.min_voxels_per_cluster = j["min_voxels_per_cluster"];
    if (j.contains("max_voxels_per_cluster")) config.max_voxels_per_cluster = j["max_voxels_per_cluster"];
    if (j.contains("max_tracking_distance")) config.max_tracking_distance = j["max_tracking_distance"];
    if (j.contains("max_tracking_age_ms")) config.max_tracking_age = std::chrono::milliseconds(j["max_tracking_age_ms"]);
    if (j.contains("use_simd")) config.use_simd = j["use_simd"];
    if (j.contains("worker_threads")) config.worker_threads = j["worker_threads"];
}

void ConfigManager::to_json(nlohmann::json& j, const orthanc::rtsp::ProcessingConfig& config) {
    j["motion_threshold"] = config.motion_threshold;
    j["voxel_accumulation_rate"] = config.voxel_accumulation_rate;
    j["voxel_decay_rate"] = config.voxel_decay_rate;
    j["frame_skip"] = config.frame_skip;
    j["enable_gpu_acceleration"] = config.enable_gpu_acceleration;
    j["max_ray_distance_voxels"] = config.max_ray_distance_voxels;
    j["background_learning_rate"] = config.background_learning_rate;
    j["background_history"] = config.background_history;
    j["background_threshold"] = config.background_threshold;
}

void ConfigManager::from_json(const nlohmann::json& j, orthanc::rtsp::ProcessingConfig& config) {
    if (j.contains("motion_threshold")) config.motion_threshold = j["motion_threshold"];
    if (j.contains("voxel_accumulation_rate")) config.voxel_accumulation_rate = j["voxel_accumulation_rate"];
    if (j.contains("voxel_decay_rate")) config.voxel_decay_rate = j["voxel_decay_rate"];
    if (j.contains("frame_skip")) config.frame_skip = j["frame_skip"];
    if (j.contains("enable_gpu_acceleration")) config.enable_gpu_acceleration = j["enable_gpu_acceleration"];
    if (j.contains("max_ray_distance_voxels")) config.max_ray_distance_voxels = j["max_ray_distance_voxels"];
    if (j.contains("background_learning_rate")) config.background_learning_rate = j["background_learning_rate"];
    if (j.contains("background_history")) config.background_history = j["background_history"];
    if (j.contains("background_threshold")) config.background_threshold = j["background_threshold"];
}

void ConfigManager::to_json(nlohmann::json& j, const orthanc::atak::AtakConfig& config) {
    j["server_ip"] = config.server_ip;
    j["server_port"] = config.server_port;
    j["sensor_callsign"] = config.sensor_callsign;
    to_json(j["sensor_location"], config.sensor_location);
    j["update_interval_ms"] = config.update_interval.count();
    j["min_confidence_threshold"] = config.min_confidence_threshold;
    j["send_velocity_data"] = config.send_velocity_data;
    j["compress_messages"] = config.compress_messages;
    j["connection_timeout_ms"] = config.connection_timeout_ms;
    j["send_timeout_ms"] = config.send_timeout_ms;
    j["max_retry_attempts"] = config.max_retry_attempts;
}

void ConfigManager::from_json(const nlohmann::json& j, orthanc::atak::AtakConfig& config) {
    if (j.contains("server_ip")) config.server_ip = j["server_ip"];
    if (j.contains("server_port")) config.server_port = j["server_port"];
    if (j.contains("sensor_callsign")) config.sensor_callsign = j["sensor_callsign"];
    if (j.contains("sensor_location")) from_json(j["sensor_location"], config.sensor_location);
    if (j.contains("update_interval_ms")) config.update_interval = std::chrono::milliseconds(j["update_interval_ms"]);
    if (j.contains("min_confidence_threshold")) config.min_confidence_threshold = j["min_confidence_threshold"];
    if (j.contains("send_velocity_data")) config.send_velocity_data = j["send_velocity_data"];
    if (j.contains("compress_messages")) config.compress_messages = j["compress_messages"];
    if (j.contains("connection_timeout_ms")) config.connection_timeout_ms = j["connection_timeout_ms"];
    if (j.contains("send_timeout_ms")) config.send_timeout_ms = j["send_timeout_ms"];
    if (j.contains("max_retry_attempts")) config.max_retry_attempts = j["max_retry_attempts"];
}

void ConfigManager::to_json(nlohmann::json& j, const orthanc::rtsp::CameraConfig& config) {
    j["camera_id"] = config.camera_id;
    j["rtsp_url"] = config.rtsp_url;
    to_json(j["position"], config.position);
    j["yaw"] = config.yaw;
    j["pitch"] = config.pitch;
    j["roll"] = config.roll;
    j["fov_horizontal"] = config.fov_horizontal;
    j["fov_vertical"] = config.fov_vertical;
    j["focal_length_x"] = config.focal_length_x;
    j["focal_length_y"] = config.focal_length_y;
    j["principal_x"] = config.principal_x;
    j["principal_y"] = config.principal_y;
}

void ConfigManager::from_json(const nlohmann::json& j, orthanc::rtsp::CameraConfig& config) {
    if (j.contains("camera_id")) config.camera_id = j["camera_id"];
    if (j.contains("rtsp_url")) config.rtsp_url = j["rtsp_url"];
    if (j.contains("position")) from_json(j["position"], config.position);
    if (j.contains("yaw")) config.yaw = j["yaw"];
    if (j.contains("pitch")) config.pitch = j["pitch"];
    if (j.contains("roll")) config.roll = j["roll"];
    if (j.contains("fov_horizontal")) config.fov_horizontal = j["fov_horizontal"];
    if (j.contains("fov_vertical")) config.fov_vertical = j["fov_vertical"];
    if (j.contains("focal_length_x")) config.focal_length_x = j["focal_length_x"];
    if (j.contains("focal_length_y")) config.focal_length_y = j["focal_length_y"];
    if (j.contains("principal_x")) config.principal_x = j["principal_x"];
    if (j.contains("principal_y")) config.principal_y = j["principal_y"];
}

void ConfigManager::to_json(nlohmann::json& j, const orthanc::core::Point3D& point) {
    j = {point.x, point.y, point.z};
}

void ConfigManager::from_json(const nlohmann::json& j, orthanc::core::Point3D& point) {
    if (j.is_array() && j.size() >= 3) {
        point.x = j[0];
        point.y = j[1];
        point.z = j[2];
    }
}

} // namespace orthanc::config 