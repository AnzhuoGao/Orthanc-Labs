#pragma once

#include "core/fast_voxel_detector.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <functional>

namespace orthanc::rtsp {

struct CameraConfig {
    int camera_id;
    std::string rtsp_url;
    orthanc::core::Point3D position;
    float yaw, pitch, roll; // Rotation angles in radians
    float fov_horizontal; // Field of view in radians
    float fov_vertical;
    
    // Intrinsic parameters
    float focal_length_x, focal_length_y;
    float principal_x, principal_y;
    
    CameraConfig() : camera_id(0), yaw(0), pitch(0), roll(0), 
                    fov_horizontal(1.047f), fov_vertical(0.785f),
                    focal_length_x(800), focal_length_y(800),
                    principal_x(320), principal_y(240) {}
};

struct ProcessingConfig {
    float motion_threshold{15.0f};
    float voxel_accumulation_rate{0.1f};
    float voxel_decay_rate{0.95f};
    int frame_skip{1}; // Process every Nth frame
    bool enable_gpu_acceleration{true};
    int max_ray_distance_voxels{150};
    
    // Background subtraction parameters
    float background_learning_rate{0.001f};
    int background_history{500};
    float background_threshold{25.0f};
};

class StreamProcessor {
public:
    explicit StreamProcessor(const ProcessingConfig& config = {});
    ~StreamProcessor();
    
    // Non-copyable, movable
    StreamProcessor(const StreamProcessor&) = delete;
    StreamProcessor& operator=(const StreamProcessor&) = delete;
    StreamProcessor(StreamProcessor&&) = default;
    StreamProcessor& operator=(StreamProcessor&&) = default;
    
    // Camera management
    bool add_camera(const CameraConfig& camera);
    void remove_camera(int camera_id);
    void update_camera_config(const CameraConfig& camera);
    
    // Processing control
    bool start_processing(std::shared_ptr<orthanc::core::FastVoxelDetector> detector);
    void stop_processing();
    bool is_processing() const { return processing_.load(); }
    
    // Configuration
    void update_config(const ProcessingConfig& config);
    const ProcessingConfig& get_config() const { return config_; }
    
    // Callbacks for events
    using FrameCallback = std::function<void(int camera_id, const cv::Mat& frame, 
                                           const cv::Mat& motion_mask)>;
    void set_frame_callback(FrameCallback callback) { frame_callback_ = std::move(callback); }
    
    // Performance monitoring
    struct StreamStats {
        size_t frames_processed{0};
        size_t motion_pixels_detected{0};
        size_t rays_cast{0};
        float average_fps{0.0f};
        std::chrono::microseconds average_processing_time{0};
    };
    StreamStats get_stats(int camera_id) const;

private:
    ProcessingConfig config_;
    std::vector<CameraConfig> cameras_;
    std::atomic<bool> processing_{false};
    
    // Processing threads
    std::vector<std::thread> capture_threads_;
    std::vector<std::thread> processing_threads_;
    
    // Shared detector
    std::shared_ptr<orthanc::core::FastVoxelDetector> detector_;
    
    // OpenCV components
    struct CameraState {
        cv::VideoCapture capture;
        cv::Ptr<cv::BackgroundSubtractor> bg_subtractor;
        cv::Mat previous_frame;
        cv::Mat motion_mask;
        
        // Performance tracking
        mutable std::mutex stats_mutex;
        StreamStats stats;
        std::chrono::steady_clock::time_point last_fps_update;
        size_t frames_since_fps_update{0};
    };
    std::vector<std::unique_ptr<CameraState>> camera_states_;
    
    // Callbacks
    FrameCallback frame_callback_;
    
    // Thread functions
    void capture_thread(int camera_index);
    void processing_thread(int camera_index);
    
    // Processing functions
    void detect_motion(int camera_index, const cv::Mat& frame, cv::Mat& motion_mask);
    void cast_rays_to_voxels(int camera_index, const cv::Mat& motion_mask);
    
    // Ray casting utilities
    struct Ray {
        orthanc::core::Point3D origin;
        orthanc::core::Point3D direction;
    };
    
    Ray pixel_to_ray(int camera_index, int pixel_x, int pixel_y) const;
    void cast_ray_dda(const Ray& ray, float intensity);
    
    // Camera transformations
    cv::Mat get_rotation_matrix(const CameraConfig& camera) const;
    orthanc::core::Point3D transform_point(const cv::Mat& rotation, 
                                          const orthanc::core::Point3D& point) const;
};

} // namespace orthanc::rtsp 