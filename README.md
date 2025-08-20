# Orthanc Labs - Fast Drone Detection System
## Configuration Guide

A high-performance, real-time drone detection system optimized for speed and accuracy. This guide provides detailed documentation for configuring the system.

---

## 📋 **Configuration File Overview**

The system uses JSON configuration files to control all aspects of operation. The configuration is divided into five main sections:

- **`detector`** - Core voxel detection parameters
- **`processing`** - Video processing and motion detection settings  
- **`atak`** - ATAK integration and network settings
- **`cameras`** - Camera configuration and positioning
- **`system`** - Global system settings

---

## 🎯 **Detector Configuration (`detector`)**

Controls the core 3D voxel-based drone detection algorithm.

```json
{
  "detector": {
    "grid_size": [200, 200, 100],
    "voxel_size": 0.2,
    "grid_origin": [-20.0, -20.0, 0.0],
    "intensity_threshold": 0.3,
    "cluster_distance": 1.5,
    "min_voxels_per_cluster": 8,
    "max_voxels_per_cluster": 2000,
    "max_tracking_distance": 5.0,
    "max_tracking_age_ms": 5000,
    "use_simd": true,
    "worker_threads": 0
  }
}
```

### **Grid Parameters**

| Parameter | Type | Description | Recommended Values |
|-----------|------|-------------|-------------------|
| `grid_size` | Array[3] | 3D voxel grid dimensions [X, Y, Z] | **Desktop**: [200, 200, 100]<br>**Raspberry Pi**: [100, 100, 50] |
| `voxel_size` | Float | Size of each voxel in meters | **High Precision**: 0.1-0.2m<br>**Performance**: 0.5-1.0m |
| `grid_origin` | Array[3] | World coordinates of grid corner [X, Y, Z] | Center grid around detection area |

**📊 Memory Usage Formula:**
```
Memory (MB) = (grid_size[0] × grid_size[1] × grid_size[2] × 4 bytes) / 1,048,576
Example: 200×200×100 = 76 MB
```

### **Detection Thresholds**

| Parameter | Type | Description | Tuning Guide |
|-----------|------|-------------|--------------|
| `intensity_threshold` | Float | Minimum voxel intensity to consider for detection | **High Sensitivity**: 0.1-0.3<br>**Low False Positives**: 0.4-0.6 |
| `cluster_distance` | Float | Maximum distance (meters) between voxels in same cluster | **Tight Clustering**: 1.0-1.5m<br>**Loose Clustering**: 2.0-3.0m |
| `min_voxels_per_cluster` | Integer | Minimum voxels required to form a drone detection | **Sensitive**: 5-8<br>**Robust**: 10-15 |
| `max_voxels_per_cluster` | Integer | Maximum voxels allowed in a cluster (performance limit) | **Standard**: 1000-2000<br>**Large Objects**: 3000+ |

### **Tracking Parameters**

| Parameter | Type | Description | Impact |
|-----------|------|-------------|--------|
| `max_tracking_distance` | Float | Maximum distance (meters) a drone can move between frames | **Fast Drones**: 10.0+<br>**Hovering**: 3.0-5.0 |
| `max_tracking_age_ms` | Integer | Time (milliseconds) to keep tracking lost drones | **Persistent**: 10000ms<br>**Responsive**: 3000-5000ms |

### **Performance Settings**

| Parameter | Type | Description | Platform Recommendations |
|-----------|------|-------------|-------------------------|
| `use_simd` | Boolean | Enable SIMD optimizations | **x86**: true (AVX2)<br>**ARM**: true (NEON)<br>**Other**: false |
| `worker_threads` | Integer | Number of processing threads (0 = auto-detect) | **Desktop**: 0 (auto)<br>**Raspberry Pi**: 2-4 |

---

## 📹 **Processing Configuration (`processing`)**

Controls video processing, motion detection, and voxel accumulation.

```json
{
  "processing": {
    "motion_threshold": 15.0,
    "voxel_accumulation_rate": 0.1,
    "voxel_decay_rate": 0.95,
    "frame_skip": 1,
    "enable_gpu_acceleration": true,
    "max_ray_distance_voxels": 150,
    "background_learning_rate": 0.001,
    "background_history": 500,
    "background_threshold": 25.0
  }
}
```

### **Motion Detection**

| Parameter | Type | Description | Tuning Guide |
|-----------|------|-------------|--------------|
| `motion_threshold` | Float | Pixel intensity change threshold for motion detection | **Sensitive**: 5.0-10.0<br>**Noise Resistant**: 15.0-25.0 |
| `background_learning_rate` | Float | How quickly background model adapts to changes | **Static Scene**: 0.001<br>**Dynamic Scene**: 0.01 |
| `background_history` | Integer | Number of frames used for background modeling | **Stable**: 500-1000<br>**Adaptive**: 100-300 |
| `background_threshold` | Float | Threshold for background subtraction | **Sensitive**: 15.0-20.0<br>**Robust**: 25.0-35.0 |

### **Voxel Processing**

| Parameter | Type | Description | Effect |
|-----------|------|-------------|--------|
| `voxel_accumulation_rate` | Float | Rate at which motion adds to voxel intensity | **Fast Response**: 0.2-0.3<br>**Smooth**: 0.05-0.15 |
| `voxel_decay_rate` | Float | Rate at which voxel intensity decays over time | **Persistent**: 0.98-0.99<br>**Responsive**: 0.85-0.95 |
| `max_ray_distance_voxels` | Integer | Maximum distance rays travel through voxel grid | **Long Range**: 200+<br>**Performance**: 100-150 |

### **Performance Optimization**

| Parameter | Type | Description | Platform Settings |
|-----------|------|-------------|-------------------|
| `frame_skip` | Integer | Process every Nth frame (1 = every frame) | **Real-time**: 1<br>**Performance**: 2-3 |
| `enable_gpu_acceleration` | Boolean | Use GPU for video processing when available | **Desktop**: true<br>**Raspberry Pi**: false |

---

## 🗺️ **ATAK Configuration (`atak`)**

Controls integration with ATAK (Android Team Awareness Kit) systems.

```json
{
  "atak": {
    "server_ip": "127.0.0.1",
    "server_port": 8087,
    "sensor_callsign": "DRONE-DETECTOR-01",
    "sensor_location": [38.8895, -77.0352, 0.0],
    "update_interval_ms": 1000,
    "min_confidence_threshold": 0.3,
    "send_velocity_data": true,
    "compress_messages": false,
    "connection_timeout_ms": 5000,
    "send_timeout_ms": 1000,
    "max_retry_attempts": 3
  }
}
```

### **Network Settings**

| Parameter | Type | Description | Examples |
|-----------|------|-------------|----------|
| `server_ip` | String | IP address of ATAK server | **Local**: "127.0.0.1"<br>**Network**: "192.168.1.100" |
| `server_port` | Integer | TCP port for ATAK CoT messages | **Standard**: 8087<br>**Custom**: 4242, 8080 |
| `connection_timeout_ms` | Integer | Timeout for initial connection | **LAN**: 3000ms<br>**WAN**: 10000ms |
| `send_timeout_ms` | Integer | Timeout for sending each message | **Fast**: 500ms<br>**Reliable**: 2000ms |

### **Sensor Identity**

| Parameter | Type | Description | Guidelines |
|-----------|------|-------------|-----------|
| `sensor_callsign` | String | Unique identifier for this sensor | Use descriptive names: "TOWER-01", "MOBILE-ALPHA" |
| `sensor_location` | Array[3] | GPS coordinates [Latitude, Longitude, Altitude] | Must be accurate for coordinate conversion |

### **Data Transmission**

| Parameter | Type | Description | Recommendations |
|-----------|------|-------------|-----------------|
| `update_interval_ms` | Integer | Frequency of sending detection updates | **Real-time**: 500-1000ms<br>**Battery Saving**: 2000-5000ms |
| `min_confidence_threshold` | Float | Minimum detection confidence to transmit | **All Detections**: 0.1<br>**High Confidence**: 0.5+ |
| `send_velocity_data` | Boolean | Include velocity/speed information in messages | Recommended: true |
| `compress_messages` | Boolean | Use compression for large messages | **Low Bandwidth**: true<br>**High Performance**: false |

### **Reliability Settings**

| Parameter | Type | Description | Tuning |
|-----------|------|-------------|--------|
| `max_retry_attempts` | Integer | Number of retry attempts for failed sends | **Reliable**: 3-5<br>**Performance**: 1-2 |

---

## 📷 **Camera Configuration (`cameras`)**

Defines physical camera positions and properties. Multiple cameras can be configured as an array.

```json
{
  "cameras": [
    {
      "camera_id": 0,
      "rtsp_url": "rtsp://admin:password@192.168.1.100:554/live",
      "position": [0.0, 0.0, 3.0],
      "yaw": 0.0,
      "pitch": 0.0,
      "roll": 0.0,
      "fov_horizontal": 1.047,
      "fov_vertical": 0.785,
      "focal_length_x": 800.0,
      "focal_length_y": 800.0,
      "principal_x": 320.0,
      "principal_y": 240.0
    }
  ]
}
```

### **Camera Identity & Connection**

| Parameter | Type | Description | Examples |
|-----------|------|-------------|----------|
| `camera_id` | Integer | Unique identifier for this camera | Sequential: 0, 1, 2, ... |
| `rtsp_url` | String | RTSP stream URL | **IP Camera**: `rtsp://user:pass@192.168.1.100:554/live`<br>**Test**: `rtsp://demo.url/stream` |

### **Physical Position**

| Parameter | Type | Description | Units & Examples |
|-----------|------|-------------|------------------|
| `position` | Array[3] | Camera location in world coordinates [X, Y, Z] | Meters: [10.0, -5.0, 3.0] |
| `yaw` | Float | Horizontal rotation (left/right) | Radians: 0.0 = North, π/2 = East |
| `pitch` | Float | Vertical rotation (up/down) | Radians: 0.0 = level, -π/4 = down |
| `roll` | Float | Camera rotation around optical axis | Radians: usually 0.0 |

**🧭 Rotation Reference:**
- **Yaw**: 0° = North, 90° = East, 180° = South, 270° = West
- **Pitch**: 0° = level, -45° = looking down, +45° = looking up
- **Roll**: 0° = upright, 90° = rotated left

### **Optical Parameters**

| Parameter | Type | Description | Typical Values |
|-----------|------|-------------|----------------|
| `fov_horizontal` | Float | Horizontal field of view | **Wide**: 1.047 rad (60°)<br>**Narrow**: 0.524 rad (30°) |
| `fov_vertical` | Float | Vertical field of view | **Standard**: 0.785 rad (45°)<br>**Narrow**: 0.524 rad (30°) |

### **Camera Intrinsics** (Advanced)

| Parameter | Type | Description | Calibration Notes |
|-----------|------|-------------|-------------------|
| `focal_length_x` | Float | Horizontal focal length in pixels | From camera calibration |
| `focal_length_y` | Float | Vertical focal length in pixels | Usually ≈ focal_length_x |
| `principal_x` | Float | Optical center X coordinate | Usually ≈ image_width/2 |
| `principal_y` | Float | Optical center Y coordinate | Usually ≈ image_height/2 |

**📐 Camera Calibration:**
For accurate 3D reconstruction, calibrate cameras using tools like OpenCV's calibration utilities.

---

## ⚙️ **System Configuration (`system`)**

Global system behavior and debugging settings.

```json
{
  "system": {
    "enable_test_mode": false,
    "enable_performance_logging": true,
    "log_level": "INFO",
    "performance_report_interval_sec": 10
  }
}
```

### **Operation Modes**

| Parameter | Type | Description | When to Use |
|-----------|------|-------------|-------------|
| `enable_test_mode` | Boolean | Simulate moving drone without cameras | **Development**: true<br>**Production**: false |

### **Logging & Monitoring**

| Parameter | Type | Description | Options |
|-----------|------|-------------|---------|
| `log_level` | String | Verbosity of log output | **Debug**: "DEBUG"<br>**Normal**: "INFO"<br>**Quiet**: "ERROR" |
| `enable_performance_logging` | Boolean | Enable performance statistics output | Recommended: true |
| `performance_report_interval_sec` | Integer | Frequency of performance reports | **Development**: 5-10s<br>**Production**: 30-60s |

---

## 🎛️ **Configuration Templates**

### **High-Performance Desktop**
```json
{
  "detector": {
    "grid_size": [300, 300, 150],
    "voxel_size": 0.1,
    "use_simd": true,
    "worker_threads": 0
  },
  "processing": {
    "motion_threshold": 10.0,
    "frame_skip": 1,
    "enable_gpu_acceleration": true
  }
}
```

### **Raspberry Pi 4**
```json
{
  "detector": {
    "grid_size": [100, 100, 50],
    "voxel_size": 0.5,
    "use_simd": true,
    "worker_threads": 2
  },
  "processing": {
    "motion_threshold": 20.0,
    "frame_skip": 2,
    "enable_gpu_acceleration": false
  }
}
```

### **Edge Device (Low Power)**
```json
{
  "detector": {
    "grid_size": [80, 80, 40],
    "voxel_size": 0.8,
    "use_simd": false,
    "worker_threads": 1
  },
  "processing": {
    "motion_threshold": 25.0,
    "frame_skip": 3,
    "enable_gpu_acceleration": false
  }
}
```

---

## 🔧 **Performance Tuning Guide**

### **Memory Optimization**
1. **Reduce grid size** for lower memory usage
2. **Increase voxel size** for fewer total voxels
3. **Lower max_voxels_per_cluster** to prevent memory spikes

### **CPU Optimization**
1. **Enable SIMD** on supported platforms
2. **Set worker_threads** to match CPU cores
3. **Increase frame_skip** for lower CPU usage

### **Network Optimization**
1. **Increase update_interval_ms** for lower bandwidth
2. **Enable compression** for slow connections
3. **Raise confidence threshold** to send fewer messages

### **Accuracy vs Performance**
- **Higher Accuracy**: Smaller voxels, lower thresholds, more processing
- **Better Performance**: Larger voxels, higher thresholds, frame skipping

---

## 🚀 **Quick Start Configurations**

### **Development & Testing**
```bash
# Use test mode configuration
./fast_drone_detector config/test_config.json
```

### **Production Deployment**
```bash
# Use optimized production configuration  
./fast_drone_detector config/production_config.json
```

### **Raspberry Pi**
```bash
# Use resource-optimized configuration
./fast_drone_detector config/raspberry_pi_config.json
```

---

## 📊 **Configuration Validation**

The system automatically validates configuration on startup:

- ✅ **Valid ranges** for numerical parameters
- ✅ **Network connectivity** for ATAK integration
- ✅ **Camera accessibility** for RTSP streams
- ✅ **Memory requirements** for grid configuration

**⚠️ Common Issues:**
- Grid too large for available memory
- Invalid RTSP URLs or credentials
- Unreachable ATAK server
- Incompatible SIMD settings for architecture

---

## 🔍 **Monitoring & Debugging**

### **Performance Metrics**
The system reports key metrics during operation:
- **Processing Time**: Per-frame detection latency
- **Memory Usage**: Current voxel grid memory consumption  
- **Detection Count**: Number of active drone tracks
- **Network Status**: ATAK connection and message statistics

### **Log Levels**
- **ERROR**: Critical issues requiring attention
- **INFO**: Normal operation status and statistics
- **DEBUG**: Detailed processing information for development

---

## 🏗️ **Building & Installation**

### **Prerequisites**
```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y cmake build-essential pkg-config
sudo apt-get install -y libopencv-dev libnlohmann-json3-dev
sudo apt-get install -y libzmq3-dev zlib1g-dev

# macOS
brew install cmake opencv nlohmann-json zeromq zlib
```

### **Building**
```bash
cd "Orthanc Labs"
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### **Architecture Support**
- **✅ x86/x64**: Full AVX2 SIMD optimization
- **✅ ARM (Raspberry Pi)**: NEON SIMD optimization  
- **✅ Other**: Standard implementation

---

**For technical support and advanced configuration assistance, contact the Orthanc Labs development team.** 