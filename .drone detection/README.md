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
- **`visualization`** - Real-time and static visualization options

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

---

## 🎨 **Visualization System**

The Orthanc Labs system includes comprehensive visualization capabilities for real-time monitoring and post-analysis of drone detection data.

### **Features**

- **Real-time 3D visualization** with OpenGL-based voxel rendering
- **Cross-platform simple viewer** using matplotlib (fallback for macOS/compatibility issues)
- **Interactive analysis tools** using PyVista for detailed inspection
- **ZeroMQ streaming** for live data transmission to visualization clients
- **Drone tracking trails** showing movement history
- **Cluster analysis** with automatic object segmentation
- **High-quality screenshots** and video export capabilities

### **Quick Start**

1. **Install visualization dependencies:**
   ```bash
   chmod +x scripts/setup_visualization.sh
   ./scripts/setup_visualization.sh
   ```

2. **Activate the visualization environment:**
   ```bash
   source visualization_env.sh
   ```

3. **Start a visualizer:**
   ```bash
   # Option 1: Simple cross-platform viewer (recommended for macOS)
   python scripts/visualize_simple.py
   
   # Option 2: Advanced OpenGL viewer (Linux/Windows)
   python scripts/visualize_voxel_grid.py
   
   # Option 3: macOS-specific OpenGL viewer (if GLUT works)
   python scripts/visualize_voxel_grid_macos.py
   ```

4. **Analyze saved data:**
   ```bash
   python scripts/static_voxel_viewer.py voxel_data/voxel_grid_20231201_143022_123.bin --interactive
   ```

### **Visualization Options**

| Viewer | Platform | Features | Use Case |
|--------|----------|----------|----------|
| **visualize_simple.py** | All (macOS recommended) | Matplotlib 3D, Interactive controls, Demo mode | General use, compatibility |
| **visualize_voxel_grid.py** | Linux/Windows | OpenGL, High performance, Trails | Production monitoring |
| **static_voxel_viewer.py** | All | PyVista, Analysis tools, Screenshots | Post-analysis, presentations |

### **Visualization Configuration**

Add the following to your system configuration:

```json
{
  "visualization": {
    "enabled": true,
    "zmq_streaming": {
      "enabled": true,
      "address": "tcp://127.0.0.1:5556",
      "port": 5556,
      "compression": {
        "enabled": true,
        "level": 6
      }
    },
    "file_output": {
      "enabled": true,
      "directory": "./voxel_data",
      "save_interval_ms": 1000
    },
    "real_time_viewer": {
      "threshold": 0.3,
      "window_size": [1400, 900],
      "show_trails": true,
      "trail_length": 50
    }
  }
}
```

### **Simple Viewer Controls (visualize_simple.py)**

| Control | Action |
|---------|--------|
| **Click + Drag** | Rotate 3D view |
| **Scroll Wheel** | Zoom in/out |
| **+/-** | Adjust detection threshold |
| **R** | Reset camera view |
| **Q** | Quit application |

### **Advanced Viewer Controls (OpenGL versions)**

| Control | Action |
|---------|--------|
| **Mouse** | Look around (first-person camera) |
| **W/A/S/D** | Move camera forward/left/back/right |
| **Space/C** | Move camera up/down |
| **+/-** | Adjust detection threshold |
| **R** | Reset camera position |
| **T** | Toggle tracking trails |
| **Q** | Quit application |

### **Static Analysis Tools**

The static viewer provides advanced analysis capabilities:

```bash
# Basic usage with auto-threshold
python scripts/static_voxel_viewer.py data.bin --interactive

# Custom threshold and clustering
python scripts/static_voxel_viewer.py data.bin \
  --threshold 0.5 \
  --cluster-eps 2.0 \
  --cluster-min-samples 5 \
  --output screenshots

# Apply rotation for better viewing angle
python scripts/static_voxel_viewer.py data.bin \
  --rotation 90 270 0 \
  --interactive
```

### **Integration with Detection System**

To integrate visualization with your detection system:

```cpp
#include "visualization/voxel_visualizer.hpp"

// Configure visualization
orthanc::visualization::VisualizationConfig viz_config;
viz_config.enable_zmq_streaming = true;
viz_config.enable_file_output = true;
viz_config.output_directory = "./voxel_data";

// Create visualizer
auto visualizer = std::make_unique<orthanc::visualization::VoxelVisualizer>(viz_config);
visualizer->start();

// In your detection loop:
orthanc::visualization::VoxelGridData grid_data;
grid_data.grid_size = {200, 200, 100};
grid_data.voxel_size = 0.2f;
grid_data.grid_origin = {-20.0f, -20.0f, 0.0f};
grid_data.voxel_data = your_voxel_intensities;

visualizer->update_voxel_grid(grid_data);
```

### **Performance Considerations**

| Setting | Impact | Recommendation |
|---------|--------|---------------|
| **ZMQ Compression** | Reduces network bandwidth by ~60-80% | Enable for remote viewing |
| **File Output Interval** | Lower = more disk usage, higher = less temporal resolution | 1000ms for monitoring, 100ms for analysis |
| **Queue Size** | Higher = more memory, better for burst loads | 100 frames for most systems |
| **Voxel Threshold** | Lower = more detailed but slower rendering | 0.3 for real-time, 0.1 for analysis |

### **File Formats**

The system supports multiple file formats:

- **`.bin`** - Native Orthanc Labs format (fastest, most compact)
- **`.ply`** - Point cloud format (for external tools like MeshLab)
- **`.npy`** - NumPy format (for Python analysis)

Convert between formats:
```cpp
// Export to PLY for external visualization
orthanc::visualization::format_utils::export_to_ply(grid_data, "output.ply", 0.3f);

// Load legacy format from pixel-to-voxel projector
orthanc::visualization::VoxelGridData legacy_data;
orthanc::visualization::format_utils::load_legacy_format("old_format.bin", legacy_data);
```

### **Troubleshooting**

**Issue: "PyOpenGL not found" or "failed to open display"**
```bash
# Solution: Use the simple matplotlib-based viewer
python scripts/visualize_simple.py
```

**Issue: "No display available"**
```bash
# Solution: Enable headless mode
export PYOPENGL_PLATFORM=glx
export PYVISTA_OFF_SCREEN=true
# Or use the simple viewer:
python scripts/visualize_simple.py
```

**Issue: "ZMQ connection refused"**
```bash
# Solution: Check if the main system is running and ZMQ is enabled
netstat -an | grep 5556  # Check if port is open
# Or run in demo mode:
python scripts/visualize_simple.py 0.3 true
```

**Issue: Poor visualization performance**
```bash
# Solution: Reduce grid resolution or increase threshold
# Edit config: "grid_size": [100, 100, 50], "intensity_threshold": 0.5
# Or use simpler viewer:
python scripts/visualize_simple.py 0.5
```

**macOS-specific Issues:**
- If OpenGL viewers fail, use `python scripts/visualize_simple.py`
- The simple viewer uses matplotlib and works reliably on all macOS versions
- For best performance on macOS, ensure matplotlib backend is set correctly

---

## 🚀 **Getting Started**

### **Prerequisites**

- **C++17** compatible compiler
- **CMake** 3.16 or later
- **OpenCV** 4.x
- **Python** 3.8+ (for visualization)

### **Build Instructions**

```bash
# Clone the repository
git clone <repository-url>
cd orthanc-labs

# Build the main system
mkdir build && cd build
cmake ..
make -j$(nproc)

# Install visualization dependencies
cd ..
./scripts/setup_visualization.sh
```

### **Run Examples**

#### **Option 1: Complete System (Recommended)**
```bash
# Start everything with one command
./launch.sh

# Or with custom options
./launch.sh --visualizer simple --threshold 0.5 --demo
```

#### **Option 2: Quick Visualization Demo**
```bash
# Just test the visualization quickly
./scripts/quick_demo.sh
```

#### **Option 3: Manual Setup (Advanced)**
```bash
# Terminal 1: Start the detection system
./build/fast_drone_detector config/consensus_detection.json

# Terminal 2: Start real-time visualization
source visualization_env.sh

# Choose appropriate viewer for your platform:
# Simple (all platforms, especially macOS):
python scripts/visualize_simple.py

# Advanced (Linux/Windows):
python scripts/visualize_voxel_grid.py
```

### **Launch Script Options**

The main launcher supports multiple options for different use cases:

```bash
# Basic usage
./launch.sh                          # Start with default settings
./launch.sh --help                   # Show all options

# Visualization options
./launch.sh --visualizer simple      # Force simple visualizer
./launch.sh --visualizer opengl      # Force OpenGL visualizer
./launch.sh --visualizer auto        # Auto-detect best option (default)

# Configuration options
./launch.sh --config my_config.json  # Use custom config
./launch.sh --threshold 0.5          # Set visualization threshold
./launch.sh --demo                   # Run in demo mode

# System options
./launch.sh --no-viz                 # Detection only, no visualization
./launch.sh --port 5557              # Use custom ZMQ port
```

---

## 📊 **Performance Benchmarks**

| Configuration | Processing Time | Memory Usage | Detection Range |
|---------------|----------------|--------------|-----------------|
| Desktop (RTX 4090) | 2.3ms | 156 MB | 100m × 100m × 50m |
| Laptop (GTX 1660) | 8.7ms | 156 MB | 100m × 100m × 50m |
| Raspberry Pi 4 | 45ms | 38 MB | 50m × 50m × 25m |

---

## 📝 **License**

This project is licensed under the MIT License - see the LICENSE file for details.

---

## 🤝 **Contributing**

We welcome contributions! Please see our contributing guidelines for more information.

---

## 📧 **Support**

For technical support and questions:
- Email: support@orthanctech.com
- Documentation: [docs.orthanctech.com](https://docs.orthanctech.com)
- Issues: GitHub Issues 