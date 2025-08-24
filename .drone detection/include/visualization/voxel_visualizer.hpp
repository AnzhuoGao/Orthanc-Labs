#pragma once

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <chrono>

#ifdef HAVE_ZMQ
#include <zmq.hpp>
#endif

namespace orthanc::visualization {

/**
 * Structure representing the serializable voxel grid data
 */
struct VoxelGridData {
    std::array<int, 3> grid_size;       // Grid dimensions [X, Y, Z]
    float voxel_size;                   // Size of each voxel in meters
    std::array<float, 3> grid_origin;   // World coordinates of grid corner
    std::vector<float> voxel_data;      // Flattened voxel intensities
    
    // Serialization methods
    std::vector<uint8_t> serialize() const;
    bool deserialize(const std::vector<uint8_t>& data);
    
    // File I/O
    bool save_to_file(const std::string& filename) const;
    bool load_from_file(const std::string& filename);
    
    // Utility methods
    size_t get_voxel_count() const;
    size_t get_data_size_bytes() const;
    void clear();
    void resize(int x, int y, int z);
};

/**
 * Structure representing a drone detection for visualization
 */
struct VisualizationDetection {
    uint32_t id;                        // Unique detection ID
    std::string uuid;                   // UUID string
    std::array<float, 3> position;      // Position in world coordinates
    std::array<float, 3> velocity;      // Velocity vector
    float confidence;                   // Detection confidence [0-1]
    float uncertainty;                  // Position uncertainty in meters
    std::chrono::system_clock::time_point timestamp;
    
    // Serialization
    std::vector<uint8_t> serialize() const;
    bool deserialize(const std::vector<uint8_t>& data);
};

/**
 * Collection of detections for batch serialization
 */
struct DetectionBatch {
    std::vector<VisualizationDetection> detections;
    std::chrono::system_clock::time_point timestamp;
    
    std::vector<uint8_t> serialize() const;
    bool deserialize(const std::vector<uint8_t>& data);
    void clear();
};

/**
 * Configuration for visualization system
 */
struct VisualizationConfig {
    // ZMQ settings
    bool enable_zmq_streaming = false;
    std::string zmq_address = "tcp://127.0.0.1:5556";
    int zmq_port = 5556;
    
    // File output settings
    bool enable_file_output = false;
    std::string output_directory = "./voxels";
    int save_interval_ms = 1000;
    
    // Data compression
    bool enable_compression = true;
    int compression_level = 6;  // zlib compression level
    
    // Performance settings
    int max_queue_size = 100;
    bool drop_on_overflow = true;
};

/**
 * Main voxel visualizer class for streaming and saving voxel data
 */
class VoxelVisualizer {
public:
    explicit VoxelVisualizer(const VisualizationConfig& config = {});
    ~VoxelVisualizer();
    
    // Configuration
    void update_config(const VisualizationConfig& config);
    VisualizationConfig get_config() const;
    
    // Main interface methods
    void update_voxel_grid(const VoxelGridData& grid_data);
    void update_detections(const DetectionBatch& detections);
    void update_both(const VoxelGridData& grid_data, const DetectionBatch& detections);
    
    // Control methods
    bool start();
    void stop();
    bool is_running() const;
    
    // Status and statistics
    struct Statistics {
        uint64_t frames_sent = 0;
        uint64_t frames_saved = 0;
        uint64_t bytes_transmitted = 0;
        uint64_t compression_ratio_percent = 100;
        std::chrono::milliseconds avg_processing_time{0};
        int queue_size = 0;
        bool zmq_connected = false;
    };
    
    Statistics get_statistics() const;
    void reset_statistics();
    
    // Utility methods
    static std::string generate_filename(const std::string& base_dir, 
                                       const std::string& prefix = "voxel_grid");

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

/**
 * Utility class for loading and analyzing saved voxel data
 */
class VoxelDataAnalyzer {
public:
    struct ClusterInfo {
        std::array<float, 3> centroid;
        std::vector<std::array<float, 3>> points;
        std::vector<float> intensities;
        float total_intensity;
        float max_intensity;
        size_t voxel_count;
        std::array<float, 3> bbox_min;
        std::array<float, 3> bbox_max;
    };
    
    struct AnalysisResult {
        std::vector<ClusterInfo> clusters;
        size_t total_significant_voxels;
        float threshold_used;
        std::array<float, 3> grid_bounds_min;
        std::array<float, 3> grid_bounds_max;
    };
    
    static AnalysisResult analyze_voxel_file(const std::string& filename, 
                                           float threshold = 0.0f, 
                                           float percentile = 99.0f);
    
    static std::vector<std::array<float, 3>> extract_significant_points(
        const VoxelGridData& grid_data, 
        float threshold,
        std::vector<float>* intensities = nullptr);
    
    static AnalysisResult cluster_points(const std::vector<std::array<float, 3>>& points,
                                       const std::vector<float>& intensities,
                                       float cluster_distance = 2.0f,
                                       int min_cluster_size = 5);
};

/**
 * Helper functions for file format compatibility
 */
namespace format_utils {
    // Convert to/from legacy format used by pixel-to-voxel projector
    bool convert_to_legacy_format(const VoxelGridData& grid_data, 
                                const std::string& output_file);
    
    bool load_legacy_format(const std::string& input_file, 
                          VoxelGridData& grid_data);
    
    // Export to common formats for external visualization tools
    bool export_to_ply(const VoxelGridData& grid_data, 
                      const std::string& output_file,
                      float threshold = 0.0f);
    
    bool export_to_numpy(const VoxelGridData& grid_data,
                        const std::string& output_file);
}

} // namespace orthanc::visualization 