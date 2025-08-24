#include "visualization/voxel_visualizer.hpp"
#include <fstream>
#include <iostream>
#include <thread>
#include <queue>
#include <condition_variable>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <algorithm>

#ifdef HAVE_ZMQ
#include <zlib.h>
#endif

namespace orthanc::visualization {

// VoxelGridData implementation
std::vector<uint8_t> VoxelGridData::serialize() const {
    std::vector<uint8_t> data;
    
    // Calculate total size needed
    size_t header_size = sizeof(int) * 3 + sizeof(float) * 4; // grid_size + voxel_size + grid_origin
    size_t voxel_data_size = voxel_data.size() * sizeof(float);
    data.reserve(header_size + voxel_data_size);
    
    // Serialize header
    data.resize(header_size);
    uint8_t* ptr = data.data();
    
    // Grid size
    std::memcpy(ptr, grid_size.data(), sizeof(int) * 3);
    ptr += sizeof(int) * 3;
    
    // Voxel size
    std::memcpy(ptr, &voxel_size, sizeof(float));
    ptr += sizeof(float);
    
    // Grid origin
    std::memcpy(ptr, grid_origin.data(), sizeof(float) * 3);
    
    // Serialize voxel data
    size_t old_size = data.size();
    data.resize(old_size + voxel_data_size);
    std::memcpy(data.data() + old_size, voxel_data.data(), voxel_data_size);
    
    return data;
}

bool VoxelGridData::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(int) * 3 + sizeof(float) * 4) {
        return false; // Insufficient data for header
    }
    
    const uint8_t* ptr = data.data();
    
    // Read grid size
    std::memcpy(grid_size.data(), ptr, sizeof(int) * 3);
    ptr += sizeof(int) * 3;
    
    // Read voxel size
    std::memcpy(&voxel_size, ptr, sizeof(float));
    ptr += sizeof(float);
    
    // Read grid origin
    std::memcpy(grid_origin.data(), ptr, sizeof(float) * 3);
    ptr += sizeof(float) * 3;
    
    // Calculate expected voxel data size
    size_t expected_voxel_count = static_cast<size_t>(grid_size[0]) * grid_size[1] * grid_size[2];
    size_t expected_voxel_bytes = expected_voxel_count * sizeof(float);
    size_t header_size = sizeof(int) * 3 + sizeof(float) * 4;
    
    if (data.size() < header_size + expected_voxel_bytes) {
        return false; // Insufficient voxel data
    }
    
    // Read voxel data
    voxel_data.resize(expected_voxel_count);
    std::memcpy(voxel_data.data(), ptr, expected_voxel_bytes);
    
    return true;
}

bool VoxelGridData::save_to_file(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    auto data = serialize();
    file.write(reinterpret_cast<const char*>(data.data()), data.size());
    return file.good();
}

bool VoxelGridData::load_from_file(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // Get file size
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    // Read all data
    std::vector<uint8_t> data(file_size);
    file.read(reinterpret_cast<char*>(data.data()), file_size);
    
    if (!file.good()) {
        return false;
    }
    
    return deserialize(data);
}

size_t VoxelGridData::get_voxel_count() const {
    return static_cast<size_t>(grid_size[0]) * grid_size[1] * grid_size[2];
}

size_t VoxelGridData::get_data_size_bytes() const {
    return sizeof(int) * 3 + sizeof(float) * 4 + voxel_data.size() * sizeof(float);
}

void VoxelGridData::clear() {
    voxel_data.clear();
    grid_size = {0, 0, 0};
    voxel_size = 0.0f;
    grid_origin = {0.0f, 0.0f, 0.0f};
}

void VoxelGridData::resize(int x, int y, int z) {
    grid_size = {x, y, z};
    voxel_data.resize(static_cast<size_t>(x) * y * z, 0.0f);
}

// VisualizationDetection implementation
std::vector<uint8_t> VisualizationDetection::serialize() const {
    std::vector<uint8_t> data;
    
    // Calculate size: id(4) + position(12) + velocity(12) + confidence(4) + uncertainty(4) + timestamp(8)
    size_t total_size = sizeof(uint32_t) + sizeof(float) * 8 + sizeof(uint64_t);
    data.resize(total_size);
    
    uint8_t* ptr = data.data();
    
    // ID
    std::memcpy(ptr, &id, sizeof(uint32_t));
    ptr += sizeof(uint32_t);
    
    // Position
    std::memcpy(ptr, position.data(), sizeof(float) * 3);
    ptr += sizeof(float) * 3;
    
    // Velocity
    std::memcpy(ptr, velocity.data(), sizeof(float) * 3);
    ptr += sizeof(float) * 3;
    
    // Confidence
    std::memcpy(ptr, &confidence, sizeof(float));
    ptr += sizeof(float);
    
    // Uncertainty
    std::memcpy(ptr, &uncertainty, sizeof(float));
    ptr += sizeof(float);
    
    // Timestamp
    auto time_since_epoch = timestamp.time_since_epoch();
    uint64_t timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch).count();
    std::memcpy(ptr, &timestamp_ms, sizeof(uint64_t));
    
    return data;
}

bool VisualizationDetection::deserialize(const std::vector<uint8_t>& data) {
    size_t expected_size = sizeof(uint32_t) + sizeof(float) * 8 + sizeof(uint64_t);
    if (data.size() < expected_size) {
        return false;
    }
    
    const uint8_t* ptr = data.data();
    
    // ID
    std::memcpy(&id, ptr, sizeof(uint32_t));
    ptr += sizeof(uint32_t);
    
    // Position
    std::memcpy(position.data(), ptr, sizeof(float) * 3);
    ptr += sizeof(float) * 3;
    
    // Velocity
    std::memcpy(velocity.data(), ptr, sizeof(float) * 3);
    ptr += sizeof(float) * 3;
    
    // Confidence
    std::memcpy(&confidence, ptr, sizeof(float));
    ptr += sizeof(float);
    
    // Uncertainty
    std::memcpy(&uncertainty, ptr, sizeof(float));
    ptr += sizeof(float);
    
    // Timestamp
    uint64_t timestamp_ms;
    std::memcpy(&timestamp_ms, ptr, sizeof(uint64_t));
    timestamp = std::chrono::system_clock::time_point(std::chrono::milliseconds(timestamp_ms));
    
    return true;
}

// DetectionBatch implementation
std::vector<uint8_t> DetectionBatch::serialize() const {
    std::vector<uint8_t> data;
    
    // Header: detection count (4 bytes) + timestamp (8 bytes)
    size_t header_size = sizeof(uint32_t) + sizeof(uint64_t);
    data.resize(header_size);
    
    uint8_t* ptr = data.data();
    
    // Detection count
    uint32_t count = static_cast<uint32_t>(detections.size());
    std::memcpy(ptr, &count, sizeof(uint32_t));
    ptr += sizeof(uint32_t);
    
    // Timestamp
    auto time_since_epoch = timestamp.time_since_epoch();
    uint64_t timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch).count();
    std::memcpy(ptr, &timestamp_ms, sizeof(uint64_t));
    
    // Serialize each detection
    for (const auto& detection : detections) {
        auto detection_data = detection.serialize();
        data.insert(data.end(), detection_data.begin(), detection_data.end());
    }
    
    return data;
}

bool DetectionBatch::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(uint32_t) + sizeof(uint64_t)) {
        return false; // Insufficient header data
    }
    
    const uint8_t* ptr = data.data();
    
    // Detection count
    uint32_t count;
    std::memcpy(&count, ptr, sizeof(uint32_t));
    ptr += sizeof(uint32_t);
    
    // Timestamp
    uint64_t timestamp_ms;
    std::memcpy(&timestamp_ms, ptr, sizeof(uint64_t));
    timestamp = std::chrono::system_clock::time_point(std::chrono::milliseconds(timestamp_ms));
    ptr += sizeof(uint64_t);
    
    // Deserialize detections
    detections.clear();
    detections.reserve(count);
    
    size_t detection_size = sizeof(uint32_t) + sizeof(float) * 8 + sizeof(uint64_t);
    size_t header_size = sizeof(uint32_t) + sizeof(uint64_t);
    
    if (data.size() < header_size + count * detection_size) {
        return false; // Insufficient detection data
    }
    
    for (uint32_t i = 0; i < count; ++i) {
        std::vector<uint8_t> detection_data(ptr, ptr + detection_size);
        
        VisualizationDetection detection;
        if (!detection.deserialize(detection_data)) {
            return false;
        }
        
        detections.push_back(std::move(detection));
        ptr += detection_size;
    }
    
    return true;
}

void DetectionBatch::clear() {
    detections.clear();
    timestamp = std::chrono::system_clock::now();
}

// VoxelVisualizer implementation
class VoxelVisualizer::Impl {
public:
    explicit Impl(const VisualizationConfig& config) : config_(config) {
        reset_statistics();
    }
    
    ~Impl() {
        stop();
    }
    
    void update_config(const VisualizationConfig& config) {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_ = config;
    }
    
    VisualizationConfig get_config() const {
        std::lock_guard<std::mutex> lock(config_mutex_);
        return config_;
    }
    
    void update_voxel_grid(const VoxelGridData& grid_data) {
        if (!running_) return;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Queue for processing
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            
            if (voxel_queue_.size() >= static_cast<size_t>(config_.max_queue_size)) {
                if (config_.drop_on_overflow) {
                    voxel_queue_.pop(); // Drop oldest
                } else {
                    return; // Skip this frame
                }
            }
            
            voxel_queue_.push(grid_data);
        }
        queue_cv_.notify_one();
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        // Update statistics
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.avg_processing_time = processing_time;
            stats_.queue_size = static_cast<int>(voxel_queue_.size());
        }
    }
    
    void update_detections(const DetectionBatch& detections) {
        if (!running_) return;
        
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            detection_queue_.push(detections);
        }
        queue_cv_.notify_one();
    }
    
    bool start() {
        if (running_) return true;
        
        running_ = true;
        
        // Start worker thread
        worker_thread_ = std::thread(&Impl::worker_thread_func, this);
        
        return true;
    }
    
    void stop() {
        if (!running_) return;
        
        running_ = false;
        queue_cv_.notify_all();
        
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        
#ifdef HAVE_ZMQ
        zmq_context_.reset();
        zmq_socket_.reset();
#endif
    }
    
    bool is_running() const {
        return running_;
    }
    
    VoxelVisualizer::Statistics get_statistics() const {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return stats_;
    }
    
    void reset_statistics() {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_ = {};
    }

private:
    void worker_thread_func() {
        init_zmq();
        
        while (running_) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { 
                return !running_ || !voxel_queue_.empty() || !detection_queue_.empty(); 
            });
            
            if (!running_) break;
            
            // Process voxel data
            while (!voxel_queue_.empty()) {
                auto voxel_data = voxel_queue_.front();
                voxel_queue_.pop();
                lock.unlock();
                
                process_voxel_data(voxel_data);
                
                lock.lock();
            }
            
            // Process detection data
            while (!detection_queue_.empty()) {
                auto detection_data = detection_queue_.front();
                detection_queue_.pop();
                lock.unlock();
                
                process_detection_data(detection_data);
                
                lock.lock();
            }
        }
    }
    
    void init_zmq() {
#ifdef HAVE_ZMQ
        if (!config_.enable_zmq_streaming) return;
        
        try {
            zmq_context_ = std::make_unique<zmq::context_t>(1);
            zmq_socket_ = std::make_unique<zmq::socket_t>(*zmq_context_, ZMQ_PUB);
            zmq_socket_->bind(config_.zmq_address);
            
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.zmq_connected = true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize ZMQ: " << e.what() << std::endl;
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.zmq_connected = false;
        }
#endif
    }
    
    void process_voxel_data(const VoxelGridData& voxel_data) {
        auto serialized = voxel_data.serialize();
        
        // Compress if enabled
        std::vector<uint8_t> final_data;
        if (config_.enable_compression) {
            final_data = compress_data(serialized);
            
            // Update compression ratio
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.compression_ratio_percent = static_cast<uint64_t>(
                (100.0 * final_data.size()) / serialized.size());
        } else {
            final_data = std::move(serialized);
        }
        
        // Send via ZMQ
        if (config_.enable_zmq_streaming && zmq_socket_) {
            send_zmq_data(final_data);
        }
        
        // Save to file
        if (config_.enable_file_output) {
            save_to_file(voxel_data);
        }
        
        // Update statistics
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.frames_sent++;
            stats_.bytes_transmitted += final_data.size();
        }
    }
    
    void process_detection_data(const DetectionBatch& detections) {
        auto serialized = detections.serialize();
        
        // Send via ZMQ (detection data as separate message)
        if (config_.enable_zmq_streaming && zmq_socket_) {
            send_zmq_data(serialized);
        }
    }
    
    std::vector<uint8_t> compress_data(const std::vector<uint8_t>& data) {
#ifdef HAVE_ZMQ
        uLongf compressed_size = compressBound(data.size());
        std::vector<uint8_t> compressed(compressed_size);
        
        int result = compress2(compressed.data(), &compressed_size, 
                             data.data(), data.size(), config_.compression_level);
        
        if (result == Z_OK) {
            compressed.resize(compressed_size);
            return compressed;
        }
#endif
        return data; // Return original if compression fails
    }
    
    void send_zmq_data(const std::vector<uint8_t>& data) {
#ifdef HAVE_ZMQ
        if (!zmq_socket_) return;
        
        try {
            zmq::message_t message(data.size());
            std::memcpy(message.data(), data.data(), data.size());
            zmq_socket_->send(message, zmq::send_flags::dontwait);
        } catch (const std::exception& e) {
            std::cerr << "ZMQ send error: " << e.what() << std::endl;
        }
#endif
    }
    
    void save_to_file(const VoxelGridData& voxel_data) {
        static auto last_save = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save).count() 
            < config_.save_interval_ms) {
            return;
        }
        
        std::string filename = generate_filename(config_.output_directory);
        
        // Create directory if it doesn't exist
        std::filesystem::create_directories(std::filesystem::path(filename).parent_path());
        
        if (voxel_data.save_to_file(filename)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.frames_saved++;
        }
        
        last_save = now;
    }
    
    static std::string generate_filename(const std::string& base_dir, 
                                       const std::string& prefix = "voxel_grid") {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << base_dir << "/" << prefix << "_" 
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
           << "_" << std::setfill('0') << std::setw(3) << ms.count() << ".bin";
        
        return ss.str();
    }

private:
    mutable std::mutex config_mutex_;
    mutable std::mutex queue_mutex_;
    mutable std::mutex stats_mutex_;
    std::condition_variable queue_cv_;
    
    VisualizationConfig config_;
    std::atomic<bool> running_{false};
    
    std::queue<VoxelGridData> voxel_queue_;
    std::queue<DetectionBatch> detection_queue_;
    
    std::thread worker_thread_;
    
    VoxelVisualizer::Statistics stats_;
    
#ifdef HAVE_ZMQ
    std::unique_ptr<zmq::context_t> zmq_context_;
    std::unique_ptr<zmq::socket_t> zmq_socket_;
#endif
};

// VoxelVisualizer public interface
VoxelVisualizer::VoxelVisualizer(const VisualizationConfig& config) 
    : pimpl_(std::make_unique<Impl>(config)) {}

VoxelVisualizer::~VoxelVisualizer() = default;

void VoxelVisualizer::update_config(const VisualizationConfig& config) {
    pimpl_->update_config(config);
}

VisualizationConfig VoxelVisualizer::get_config() const {
    return pimpl_->get_config();
}

void VoxelVisualizer::update_voxel_grid(const VoxelGridData& grid_data) {
    pimpl_->update_voxel_grid(grid_data);
}

void VoxelVisualizer::update_detections(const DetectionBatch& detections) {
    pimpl_->update_detections(detections);
}

void VoxelVisualizer::update_both(const VoxelGridData& grid_data, const DetectionBatch& detections) {
    pimpl_->update_voxel_grid(grid_data);
    pimpl_->update_detections(detections);
}

bool VoxelVisualizer::start() {
    return pimpl_->start();
}

void VoxelVisualizer::stop() {
    pimpl_->stop();
}

bool VoxelVisualizer::is_running() const {
    return pimpl_->is_running();
}

VoxelVisualizer::Statistics VoxelVisualizer::get_statistics() const {
    return pimpl_->get_statistics();
}

void VoxelVisualizer::reset_statistics() {
    pimpl_->reset_statistics();
}

std::string VoxelVisualizer::generate_filename(const std::string& base_dir, 
                                             const std::string& prefix) {
    return Impl::generate_filename(base_dir, prefix);
}

} // namespace orthanc::visualization 