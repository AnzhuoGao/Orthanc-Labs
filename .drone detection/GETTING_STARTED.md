# Getting Started with Orthanc Labs

Welcome to the Orthanc Labs Fast Drone Detection System! This guide will get you up and running quickly.

## 🚀 Quick Start (2 minutes)

### 1. **First Time Setup**
```bash
# Install visualization dependencies
./scripts/setup_visualization.sh
```

### 2. **Test Visualization (Demo Mode)**
```bash
# See the system in action immediately
./scripts/quick_demo.sh
```
This opens a 3D visualization with animated demo objects. Perfect for testing!

### 3. **Build the Detection System** (When ready for real detection)
```bash
# Build the C++ detection engine
mkdir -p build && cd build
cmake .. && make
cd ..
```

### 4. **Launch Complete System**
```bash
# Start everything with one command
./launch.sh
```

## 🎯 Launch Options

### **Quick Demo** (Visualization Only)
```bash
./scripts/quick_demo.sh
```
- **Best for:** Testing, demonstrations, first-time users
- **Features:** Animated 3D objects, no detection system needed
- **Platform:** Works on all platforms (especially good for macOS)

### **Full System** (Detection + Visualization)
```bash
./launch.sh                    # Default settings
./launch.sh --demo             # Demo mode with detection system
./launch.sh --visualizer simple # Force simple visualizer
```
- **Best for:** Production use, real drone detection
- **Features:** Complete system with detection and visualization
- **Requires:** Built detection system

### **Detection Only** (No Visualization)
```bash
./launch.sh --no-viz
```
- **Best for:** Server deployments, headless operation
- **Features:** Detection system only, minimal resources

## 🎨 Visualization Options

| Command | Platform | Best For | Features |
|---------|----------|----------|----------|
| `./scripts/quick_demo.sh` | **All (macOS recommended)** | Testing, demos | Animated objects, matplotlib |
| `./launch.sh --visualizer simple` | **All platforms** | General use | 3D plotting, interactive |
| `./launch.sh --visualizer opengl` | **Linux/Windows** | High performance | Real-time, trails, advanced |

## 🔧 Configuration Examples

### **High Performance Setup**
```bash
./launch.sh --visualizer opengl --threshold 0.2 --config config/fast_drone_detector.json
```

### **macOS Friendly Setup**
```bash
./launch.sh --visualizer simple --threshold 0.3 --demo
```

### **Development/Testing Setup**
```bash
./launch.sh --demo --threshold 0.5
```

### **Server Deployment**
```bash
./launch.sh --no-viz --config config/production.json
```

## 🎮 Controls

### Simple Visualizer (Matplotlib)
- **Rotate:** Click and drag
- **Zoom:** Scroll wheel
- **Reset view:** R key
- **Adjust threshold:** +/- keys
- **Quit:** Q key

### Advanced Visualizer (OpenGL)
- **Camera:** Mouse to look around
- **Move:** W/A/S/D keys
- **Up/Down:** Space/C keys
- **Reset camera:** R key
- **Toggle trails:** T key
- **Adjust threshold:** +/- keys
- **Quit:** Q key

## 🐛 Troubleshooting

### **"Command not found" errors**
```bash
chmod +x launch.sh
chmod +x scripts/*.sh
```

### **Visualization not working**
```bash
# Try the simple visualizer
./launch.sh --visualizer simple
# Or just the demo
./scripts/quick_demo.sh
```

### **macOS OpenGL issues**
```bash
# Use the simple visualizer (recommended for macOS)
./launch.sh --visualizer simple
```

### **No detection system built**
```bash
# Build the system first
mkdir -p build && cd build
cmake .. && make
cd ..
```

## 📁 Key Files

- `./launch.sh` - Main system launcher
- `./scripts/quick_demo.sh` - Quick visualization demo
- `./scripts/setup_visualization.sh` - One-time setup
- `./visualization_env.sh` - Environment activation (created by setup)
- `./config/` - Configuration files
- `./build/` - Built detection system

## 🎯 Common Use Cases

### **I want to see it working immediately**
```bash
./scripts/quick_demo.sh
```

### **I want to test the complete system**
```bash
./launch.sh --demo
```

### **I want to use it for real drone detection**
```bash
# 1. Configure your cameras in config/
# 2. Run:
./launch.sh --config your_config.json
```

### **I want visualization only (no detection)**
```bash
./scripts/quick_demo.sh
```

---

## 📞 Need Help?

- **Documentation:** See `README.md` for complete configuration guide
- **Issues:** Check the troubleshooting section above
- **Logs:** System logs are saved in `./logs/` directory

**Happy drone detecting!** 🛩️ 