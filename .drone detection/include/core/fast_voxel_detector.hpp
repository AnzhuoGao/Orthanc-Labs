#pragma once

#include <vector>
#include <array>
#include <memory>
#include <atomic>
#include <chrono>
#include <string>
#include <unordered_map>
#include <mutex>

#if defined(HAVE_AVX2) && (defined(__x86_64__) || defined(__i386__))
#include <immintrin.h>
#endif

#ifdef HAVE_NEON
#include <arm_neon.h>
#endif

namespace orthanc::core {

// Optimized 3D point structure with SIMD alignment
struct alignas(16) Point3D {
    float x, y, z;
    float _padding; // For SIMD alignment
    
    Point3D() : x(0), y(0), z(0), _padding(0) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_), _padding(0) {}
    
    float distance_squared(const Point3D& other) const noexcept {
        const float dx = x - other.x;
        const float dy = y - other.y;
        const float dz = z - other.z;
        return dx*dx + dy*dy + dz*dz;
    }
};

// Drone detection result with timestamp
struct DroneDetection {
    uint32_t id;
    std::string uuid;
    Point3D position;
    Point3D velocity;
    float confidence;
    float uncertainty;
    std::chrono::steady_clock::time_point timestamp;
    bool velocity_valid;
    
    DroneDetection() : id(0), confidence(0.0f), uncertainty(10.0f), 
                      velocity_valid(false) {}
};

// Configuration for the detector
struct DetectorConfig {
    // Grid parameters
    std::array<int, 3> grid_size{200, 200, 100};
    float voxel_size{0.2f}; // meters
    Point3D grid_origin{-20.0f, -20.0f, 0.0f};
    
    // Detection thresholds
    float intensity_threshold{0.3f};
    float cluster_distance{1.5f}; // meters
    int min_voxels_per_cluster{8};
    int max_voxels_per_cluster{2000};
    
    // Tracking parameters
    float max_tracking_distance{5.0f}; // meters
    std::chrono::milliseconds max_tracking_age{5000};
    
    // Performance tuning
    bool use_simd{true};
    int worker_threads{0}; // 0 = auto-detect
};

class FastVoxelDetector {
public:
    explicit FastVoxelDetector(const DetectorConfig& config = {});
    ~FastVoxelDetector();
    
    // Non-copyable, movable
    FastVoxelDetector(const FastVoxelDetector&) = delete;
    FastVoxelDetector& operator=(const FastVoxelDetector&) = delete;
    FastVoxelDetector(FastVoxelDetector&&) = default;
    FastVoxelDetector& operator=(FastVoxelDetector&&) = default;
    
    // Main processing interface
    void process_voxel_data(const float* voxel_data, size_t data_size);
    void update_voxel(int x, int y, int z, float intensity);
    void clear_voxels();
    
    // Results access (thread-safe)
    std::vector<DroneDetection> get_detections() const;
    size_t get_detection_count() const;
    
    // Configuration
    void update_config(const DetectorConfig& config);
    const DetectorConfig& get_config() const { return config_; }
    
    // Performance monitoring
    struct PerformanceStats {
        std::chrono::microseconds last_processing_time{0};
        size_t total_voxels_processed{0};
        size_t clusters_found{0};
        size_t active_tracks{0};
    };
    PerformanceStats get_performance_stats() const;

private:
    DetectorConfig config_;
    
    // Voxel grid storage (optimized for cache efficiency)
    std::vector<float> voxel_grid_;
    mutable std::mutex voxel_mutex_;
    
    // Detection state
    std::vector<DroneDetection> current_detections_;
    mutable std::mutex detection_mutex_;
    std::atomic<uint32_t> next_track_id_{1};
    
    // Performance tracking
    mutable PerformanceStats perf_stats_;
    mutable std::mutex perf_mutex_;
    
    // Internal processing methods
    void find_clusters(std::vector<std::vector<Point3D>>& clusters);
    void track_detections(const std::vector<std::vector<Point3D>>& clusters);
    void remove_old_tracks();
    
    // Optimized clustering algorithms
    void cluster_dbscan_simd(const std::vector<Point3D>& points, 
                            std::vector<std::vector<Point3D>>& clusters);
    void cluster_dbscan_standard(const std::vector<Point3D>& points, 
                                std::vector<std::vector<Point3D>>& clusters);
    
    // SIMD-optimized functions
#if defined(HAVE_AVX2) && (defined(__x86_64__) || defined(__i386__))
    void process_voxels_avx2(const float* data, size_t count, 
                            std::vector<Point3D>& candidate_points);
#endif
#ifdef HAVE_NEON
    void process_voxels_neon(const float* data, size_t count,
                            std::vector<Point3D>& candidate_points);
#endif
    void process_voxels_standard(const float* data, size_t count,
                                std::vector<Point3D>& candidate_points);
    
    // Utility functions
    inline size_t voxel_index(int x, int y, int z) const noexcept {
        return static_cast<size_t>(z) * config_.grid_size[0] * config_.grid_size[1] +
               static_cast<size_t>(y) * config_.grid_size[0] + 
               static_cast<size_t>(x);
    }
    
    inline Point3D voxel_to_world(int x, int y, int z) const noexcept {
        return Point3D(
            config_.grid_origin.x + (x + 0.5f) * config_.voxel_size,
            config_.grid_origin.y + (y + 0.5f) * config_.voxel_size,
            config_.grid_origin.z + (z + 0.5f) * config_.voxel_size
        );
    }
    
    std::string generate_uuid(uint32_t id) const;
};

} // namespace orthanc::core 