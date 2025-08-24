#include "core/fast_voxel_detector.hpp"
#include <algorithm>
#include <execution>
#include <thread>
#include <random>
#include <unordered_set>
#include <queue>
#include <cmath>

#if defined(HAVE_AVX2) && (defined(__x86_64__) || defined(__i386__))
#include <immintrin.h>
#endif

#ifdef HAVE_NEON
#include <arm_neon.h>
#endif

namespace orthanc::core {

FastVoxelDetector::FastVoxelDetector(const DetectorConfig& config) 
    : config_(config) {
    
    // Initialize voxel grid
    const size_t grid_volume = config_.grid_size[0] * config_.grid_size[1] * config_.grid_size[2];
    voxel_grid_.resize(grid_volume, 0.0f);
    
    // Reserve space for detections to avoid reallocations
    current_detections_.reserve(100);
}

FastVoxelDetector::~FastVoxelDetector() = default;

void FastVoxelDetector::process_voxel_data(const float* voxel_data, size_t data_size) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Validate input
    const size_t expected_size = config_.grid_size[0] * config_.grid_size[1] * config_.grid_size[2];
    if (data_size != expected_size) {
        return;
    }
    
    // Update internal grid
    {
        std::lock_guard<std::mutex> lock(voxel_mutex_);
        std::copy(voxel_data, voxel_data + data_size, voxel_grid_.data());
    }
    
    // Find candidate points above threshold
    std::vector<Point3D> candidate_points;
    candidate_points.reserve(data_size / 100); // Estimate 1% activation
    
#if defined(HAVE_AVX2) && (defined(__x86_64__) || defined(__i386__))
    if (config_.use_simd) {
        process_voxels_avx2(voxel_data, data_size, candidate_points);
    } else {
        process_voxels_standard(voxel_data, data_size, candidate_points);
    }
#elif defined(HAVE_NEON)
    if (config_.use_simd) {
        process_voxels_neon(voxel_data, data_size, candidate_points);
    } else {
        process_voxels_standard(voxel_data, data_size, candidate_points);
    }
#else
    process_voxels_standard(voxel_data, data_size, candidate_points);
#endif
    
    // Cluster the candidate points
    std::vector<std::vector<Point3D>> clusters;
    if (!candidate_points.empty()) {
#if defined(HAVE_AVX2) && (defined(__x86_64__) || defined(__i386__))
        if (config_.use_simd) {
            cluster_dbscan_simd(candidate_points, clusters);
        } else {
            cluster_dbscan_standard(candidate_points, clusters);
        }
#else
        cluster_dbscan_standard(candidate_points, clusters);
#endif
    }
    
    // Update tracking
    track_detections(clusters);
    remove_old_tracks();
    
    // Update performance stats
    auto end_time = std::chrono::high_resolution_clock::now();
    std::lock_guard<std::mutex> perf_lock(perf_mutex_);
    perf_stats_.last_processing_time = 
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    perf_stats_.total_voxels_processed += data_size;
    perf_stats_.clusters_found = clusters.size();
    perf_stats_.active_tracks = current_detections_.size();
}

#if defined(HAVE_AVX2) && (defined(__x86_64__) || defined(__i386__))
void FastVoxelDetector::process_voxels_avx2(const float* data, size_t count, 
                                           std::vector<Point3D>& candidate_points) {
    const __m256 threshold_vec = _mm256_set1_ps(config_.intensity_threshold);
    const size_t simd_count = count - (count % 8);
    
    for (size_t i = 0; i < simd_count; i += 8) {
        __m256 values = _mm256_loadu_ps(&data[i]);
        __m256 mask = _mm256_cmp_ps(values, threshold_vec, _CMP_GE_OQ);
        
        int mask_bits = _mm256_movemask_ps(mask);
        if (mask_bits != 0) {
            // Process individual elements that passed threshold
            for (int bit = 0; bit < 8; ++bit) {
                if (mask_bits & (1 << bit)) {
                    size_t idx = i + bit;
                    int z = idx / (config_.grid_size[0] * config_.grid_size[1]);
                    int y = (idx % (config_.grid_size[0] * config_.grid_size[1])) / config_.grid_size[0];
                    int x = idx % config_.grid_size[0];
                    candidate_points.emplace_back(voxel_to_world(x, y, z));
                }
            }
        }
    }
    
    // Handle remaining elements
    process_voxels_standard(&data[simd_count], count - simd_count, candidate_points);
}
#endif

#ifdef HAVE_NEON
void FastVoxelDetector::process_voxels_neon(const float* data, size_t count, 
                                           std::vector<Point3D>& candidate_points) {
    const float32x4_t threshold_vec = vdupq_n_f32(config_.intensity_threshold);
    const size_t simd_count = count - (count % 4);
    
    for (size_t i = 0; i < simd_count; i += 4) {
        float32x4_t values = vld1q_f32(&data[i]);
        uint32x4_t mask = vcgeq_f32(values, threshold_vec);
        
        // Check each element in the vector
        uint32_t mask_array[4];
        vst1q_u32(mask_array, mask);
        
        for (int bit = 0; bit < 4; ++bit) {
            if (mask_array[bit] != 0) {
                size_t idx = i + bit;
                int z = idx / (config_.grid_size[0] * config_.grid_size[1]);
                int y = (idx % (config_.grid_size[0] * config_.grid_size[1])) / config_.grid_size[0];
                int x = idx % config_.grid_size[0];
                candidate_points.emplace_back(voxel_to_world(x, y, z));
            }
        }
    }
    
    // Handle remaining elements
    for (size_t i = simd_count; i < count; ++i) {
        if (data[i] >= config_.intensity_threshold) {
            int z = i / (config_.grid_size[0] * config_.grid_size[1]);
            int y = (i % (config_.grid_size[0] * config_.grid_size[1])) / config_.grid_size[0];
            int x = i % config_.grid_size[0];
            candidate_points.emplace_back(voxel_to_world(x, y, z));
        }
    }
}
#endif

void FastVoxelDetector::process_voxels_standard(const float* data, size_t count,
                                               std::vector<Point3D>& candidate_points) {
    for (size_t i = 0; i < count; ++i) {
        if (data[i] >= config_.intensity_threshold) {
            int z = i / (config_.grid_size[0] * config_.grid_size[1]);
            int y = (i % (config_.grid_size[0] * config_.grid_size[1])) / config_.grid_size[0];
            int x = i % config_.grid_size[0];
            candidate_points.emplace_back(voxel_to_world(x, y, z));
        }
    }
}

void FastVoxelDetector::cluster_dbscan_standard(const std::vector<Point3D>& points, 
                                               std::vector<std::vector<Point3D>>& clusters) {
    if (points.empty()) return;
    
    const float eps_sq = config_.cluster_distance * config_.cluster_distance;
    std::vector<bool> visited(points.size(), false);
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) continue;
        
        std::vector<Point3D> cluster;
        std::queue<size_t> neighbors;
        
        visited[i] = true;
        neighbors.push(i);
        
        while (!neighbors.empty()) {
            size_t current = neighbors.front();
            neighbors.pop();
            cluster.push_back(points[current]);
            
            // Find all neighbors within eps distance
            for (size_t j = 0; j < points.size(); ++j) {
                if (!visited[j] && points[current].distance_squared(points[j]) <= eps_sq) {
                    visited[j] = true;
                    neighbors.push(j);
                }
            }
        }
        
        // Only keep clusters within size limits
        if (cluster.size() >= static_cast<size_t>(config_.min_voxels_per_cluster) &&
            cluster.size() <= static_cast<size_t>(config_.max_voxels_per_cluster)) {
            clusters.push_back(std::move(cluster));
        }
    }
}

void FastVoxelDetector::cluster_dbscan_simd(const std::vector<Point3D>& points, 
                                           std::vector<std::vector<Point3D>>& clusters) {
    // For SIMD clustering, we can optimize distance calculations
    // For now, fallback to standard implementation
    cluster_dbscan_standard(points, clusters);
}

void FastVoxelDetector::track_detections(const std::vector<std::vector<Point3D>>& clusters) {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    std::vector<bool> cluster_assigned(clusters.size(), false);
    
    // Try to match clusters to existing detections
    for (auto& detection : current_detections_) {
        float min_distance_sq = std::numeric_limits<float>::max();
        int best_cluster = -1;
        
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (cluster_assigned[i]) continue;
            
            // Calculate cluster centroid
            Point3D centroid(0, 0, 0);
            for (const auto& point : clusters[i]) {
                centroid.x += point.x;
                centroid.y += point.y;
                centroid.z += point.z;
            }
            centroid.x /= clusters[i].size();
            centroid.y /= clusters[i].size();
            centroid.z /= clusters[i].size();
            
            float dist_sq = detection.position.distance_squared(centroid);
            float max_dist_sq = config_.max_tracking_distance * config_.max_tracking_distance;
            
            if (dist_sq < min_distance_sq && dist_sq <= max_dist_sq) {
                min_distance_sq = dist_sq;
                best_cluster = static_cast<int>(i);
            }
        }
        
        if (best_cluster >= 0) {
            // Update existing detection
            cluster_assigned[best_cluster] = true;
            
            // Calculate new position (cluster centroid)
            Point3D old_pos = detection.position;
            Point3D new_pos(0, 0, 0);
            for (const auto& point : clusters[best_cluster]) {
                new_pos.x += point.x;
                new_pos.y += point.y;
                new_pos.z += point.z;
            }
            new_pos.x /= clusters[best_cluster].size();
            new_pos.y /= clusters[best_cluster].size();
            new_pos.z /= clusters[best_cluster].size();
            
            // Update velocity
            auto dt = std::chrono::duration<float>(now - detection.timestamp).count();
            if (dt > 0.01f) { // Minimum time interval
                detection.velocity.x = (new_pos.x - old_pos.x) / dt;
                detection.velocity.y = (new_pos.y - old_pos.y) / dt;
                detection.velocity.z = (new_pos.z - old_pos.z) / dt;
                detection.velocity_valid = true;
            }
            
            detection.position = new_pos;
            detection.timestamp = now;
            detection.confidence = std::min(1.0f, static_cast<float>(clusters[best_cluster].size()) / 50.0f);
            detection.uncertainty = std::sqrt(min_distance_sq);
        }
    }
    
    // Create new detections for unassigned clusters
    for (size_t i = 0; i < clusters.size(); ++i) {
        if (!cluster_assigned[i]) {
            DroneDetection new_detection;
            new_detection.id = next_track_id_++;
            new_detection.uuid = generate_uuid(new_detection.id);
            
            // Calculate position (cluster centroid)
            for (const auto& point : clusters[i]) {
                new_detection.position.x += point.x;
                new_detection.position.y += point.y;
                new_detection.position.z += point.z;
            }
            new_detection.position.x /= clusters[i].size();
            new_detection.position.y /= clusters[i].size();
            new_detection.position.z /= clusters[i].size();
            
            new_detection.timestamp = now;
            new_detection.confidence = std::min(1.0f, static_cast<float>(clusters[i].size()) / 50.0f);
            new_detection.velocity_valid = false;
            
            current_detections_.push_back(std::move(new_detection));
        }
    }
}

void FastVoxelDetector::remove_old_tracks() {
    auto now = std::chrono::steady_clock::now();
    current_detections_.erase(
        std::remove_if(current_detections_.begin(), current_detections_.end(),
            [&](const DroneDetection& detection) {
                return (now - detection.timestamp) > config_.max_tracking_age;
            }),
        current_detections_.end()
    );
}

void FastVoxelDetector::update_voxel(int x, int y, int z, float intensity) {
    if (x < 0 || x >= config_.grid_size[0] ||
        y < 0 || y >= config_.grid_size[1] ||
        z < 0 || z >= config_.grid_size[2]) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(voxel_mutex_);
    voxel_grid_[voxel_index(x, y, z)] = intensity;
}

void FastVoxelDetector::clear_voxels() {
    std::lock_guard<std::mutex> lock(voxel_mutex_);
    std::fill(voxel_grid_.begin(), voxel_grid_.end(), 0.0f);
}

std::vector<DroneDetection> FastVoxelDetector::get_detections() const {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    return current_detections_;
}

size_t FastVoxelDetector::get_detection_count() const {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    return current_detections_.size();
}

void FastVoxelDetector::update_config(const DetectorConfig& config) {
    std::lock_guard<std::mutex> voxel_lock(voxel_mutex_);
    std::lock_guard<std::mutex> detection_lock(detection_mutex_);
    
    config_ = config;
    
    // Resize voxel grid if necessary
    const size_t new_size = config_.grid_size[0] * config_.grid_size[1] * config_.grid_size[2];
    if (voxel_grid_.size() != new_size) {
        voxel_grid_.clear();
        voxel_grid_.resize(new_size, 0.0f);
    }
}

FastVoxelDetector::PerformanceStats FastVoxelDetector::get_performance_stats() const {
    std::lock_guard<std::mutex> lock(perf_mutex_);
    return perf_stats_;
}

std::string FastVoxelDetector::generate_uuid(uint32_t id) const {
    auto now = std::chrono::system_clock::now();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 15);
    
    const char* hex_chars = "0123456789abcdef";
    std::string random_hex;
    for (int i = 0; i < 8; ++i) {
        random_hex += hex_chars[dis(gen)];
    }
    
    return "drone-" + std::to_string(id) + "-" + std::to_string(millis) + "-" + random_hex;
}

} // namespace orthanc::core 