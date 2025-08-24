#!/bin/bash

# Orthanc Labs System Launcher
# This script starts the complete drone detection system with real-time visualization

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/build"
CONFIG_DIR="$PROJECT_ROOT/config"
LOG_DIR="$PROJECT_ROOT/logs"

# Default configuration
DEFAULT_CONFIG="$CONFIG_DIR/consensus_detection.json"
VISUALIZATION_CONFIG="$CONFIG_DIR/visualization_config.json"

# Process IDs for cleanup
DETECTOR_PID=""
VISUALIZER_PID=""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo -e "${CYAN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1" >&2
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

# Show banner
show_banner() {
    echo -e "${PURPLE}"
    cat << 'EOF'
    ╔═══════════════════════════════════════════════════════════════╗
    ║                 Orthanc Technologies                          ║
    ║              Fast Drone Detection System                      ║
    ║                   System Launcher v1.0                       ║
    ╚═══════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
    echo ""
}

# Show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -c, --config FILE       Detection system configuration file"
    echo "                         (default: $DEFAULT_CONFIG)"
    echo "  -v, --visualizer TYPE   Visualizer type: simple, opengl, or auto"
    echo "                         (default: auto - chooses best for platform)"
    echo "  -t, --threshold FLOAT   Visualization threshold (default: 0.3)"
    echo "  -p, --port PORT         ZMQ port for visualization (default: 5556)"
    echo "  -d, --demo              Run visualization in demo mode"
    echo "  -n, --no-viz            Skip visualization (detection only)"
    echo "  -l, --log-level LEVEL   Log level: DEBUG, INFO, WARN, ERROR"
    echo "  -h, --help              Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                      # Start with default settings"
    echo "  $0 -c custom.json       # Use custom configuration"
    echo "  $0 -v simple -t 0.5     # Use simple visualizer with threshold 0.5"
    echo "  $0 -d                   # Run in demo mode"
    echo "  $0 -n                   # Detection only, no visualization"
    echo ""
}

# Cleanup function
cleanup() {
    log "Shutting down Orthanc Labs system..."
    
    if [ ! -z "$VISUALIZER_PID" ]; then
        info "Stopping visualizer (PID: $VISUALIZER_PID)..."
        kill $VISUALIZER_PID 2>/dev/null || true
        wait $VISUALIZER_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$DETECTOR_PID" ]; then
        info "Stopping detector (PID: $DETECTOR_PID)..."
        kill -TERM $DETECTOR_PID 2>/dev/null || true
        sleep 2
        kill -KILL $DETECTOR_PID 2>/dev/null || true
        wait $DETECTOR_PID 2>/dev/null || true
    fi
    
    success "System shutdown complete"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM EXIT

# Check prerequisites
check_prerequisites() {
    log "Checking system prerequisites..."
    
    # Check if project is built
    if [ ! -f "$BUILD_DIR/fast_drone_detector" ]; then
        error "Detection system not built. Please run:"
        error "  mkdir -p build && cd build"
        error "  cmake .. && make"
        exit 1
    fi
    
    # Check configuration file
    if [ ! -f "$DETECTION_CONFIG" ]; then
        error "Configuration file not found: $DETECTION_CONFIG"
        exit 1
    fi
    
    # Check Python environment
    if ! command -v python3 &> /dev/null; then
        error "Python 3 not found. Please install Python 3.8+."
        exit 1
    fi
    
    # Check visualization environment
    if [ "$SKIP_VISUALIZATION" != "true" ]; then
        if [ ! -f "$PROJECT_ROOT/visualization_env.sh" ]; then
            warning "Visualization environment not set up."
            warning "Run: ./scripts/setup_visualization.sh"
        fi
    fi
    
    success "Prerequisites check passed"
}

# Determine best visualizer for platform
detect_visualizer() {
    if [ "$VISUALIZER_TYPE" = "auto" ]; then
        if [[ "$OSTYPE" == "darwin"* ]]; then
            VISUALIZER_TYPE="simple"
            info "Detected macOS - using simple visualizer"
        else
            # Test if OpenGL is available
            if command -v glxinfo &> /dev/null && glxinfo &> /dev/null; then
                VISUALIZER_TYPE="opengl"
                info "Detected OpenGL support - using advanced visualizer"
            else
                VISUALIZER_TYPE="simple"
                info "No OpenGL detected - using simple visualizer"
            fi
        fi
    fi
}

# Create log directory
setup_logging() {
    mkdir -p "$LOG_DIR"
    
    # Set up log files with timestamps
    TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
    DETECTOR_LOG="$LOG_DIR/detector_$TIMESTAMP.log"
    VISUALIZER_LOG="$LOG_DIR/visualizer_$TIMESTAMP.log"
    SYSTEM_LOG="$LOG_DIR/system_$TIMESTAMP.log"
    
    info "Logs will be written to:"
    info "  Detector: $DETECTOR_LOG"
    info "  Visualizer: $VISUALIZER_LOG"
    info "  System: $SYSTEM_LOG"
}

# Start detection system
start_detector() {
    log "Starting Orthanc Labs Detection System..."
    
    cd "$BUILD_DIR"
    
    # Start detector in background with logging
    ./fast_drone_detector "$DETECTION_CONFIG" > "$DETECTOR_LOG" 2>&1 &
    DETECTOR_PID=$!
    
    # Give it time to start
    sleep 2
    
    # Check if it's still running
    if ! kill -0 $DETECTOR_PID 2>/dev/null; then
        error "Detection system failed to start. Check log: $DETECTOR_LOG"
        tail -20 "$DETECTOR_LOG"
        exit 1
    fi
    
    success "Detection system started (PID: $DETECTOR_PID)"
    
    # Return to project root
    cd "$PROJECT_ROOT"
}

# Start visualization
start_visualizer() {
    if [ "$SKIP_VISUALIZATION" = "true" ]; then
        info "Skipping visualization as requested"
        return
    fi
    
    log "Starting visualization system..."
    
    # Source visualization environment if available
    if [ -f "$PROJECT_ROOT/visualization_env.sh" ]; then
        source "$PROJECT_ROOT/visualization_env.sh"
    fi
    
    # Determine visualizer script
    case "$VISUALIZER_TYPE" in
        "simple")
            VISUALIZER_SCRIPT="scripts/visualize_simple.py"
            VISUALIZER_ARGS="$VIZ_THRESHOLD"
            if [ "$DEMO_MODE" = "true" ]; then
                VISUALIZER_ARGS="$VISUALIZER_ARGS true"
            fi
            ;;
        "opengl")
            if [[ "$OSTYPE" == "darwin"* ]]; then
                VISUALIZER_SCRIPT="scripts/visualize_voxel_grid_macos.py"
            else
                VISUALIZER_SCRIPT="scripts/visualize_voxel_grid.py"
            fi
            VISUALIZER_ARGS="$VIZ_THRESHOLD $ZMQ_PORT"
            ;;
        *)
            error "Unknown visualizer type: $VISUALIZER_TYPE"
            exit 1
            ;;
    esac
    
    # Start visualizer
    info "Starting $VISUALIZER_TYPE visualizer..."
    python3 "$VISUALIZER_SCRIPT" $VISUALIZER_ARGS > "$VISUALIZER_LOG" 2>&1 &
    VISUALIZER_PID=$!
    
    # Give it time to start
    sleep 2
    
    success "Visualizer started (PID: $VISUALIZER_PID)"
    info "Visualizer type: $VISUALIZER_TYPE"
    info "Threshold: $VIZ_THRESHOLD"
    info "ZMQ Port: $ZMQ_PORT"
}

# Monitor system status
monitor_system() {
    log "System monitoring started. Press Ctrl+C to shutdown."
    echo ""
    
    # Show system status
    info "=== SYSTEM STATUS ==="
    info "Detection System: Running (PID: $DETECTOR_PID)"
    if [ "$SKIP_VISUALIZATION" != "true" ]; then
        info "Visualizer: Running (PID: $VISUALIZER_PID)"
        info "Visualizer Type: $VISUALIZER_TYPE"
    fi
    info "Configuration: $DETECTION_CONFIG"
    info "Logs Directory: $LOG_DIR"
    echo ""
    
    # Show controls
    info "=== CONTROLS ==="
    info "Ctrl+C: Shutdown system"
    info "View logs: tail -f $DETECTOR_LOG"
    if [ "$SKIP_VISUALIZATION" != "true" ]; then
        case "$VISUALIZER_TYPE" in
            "simple")
                info "Visualizer Controls:"
                info "  - Click+Drag: Rotate view"
                info "  - Scroll: Zoom"
                info "  - R: Reset view"
                info "  - +/-: Adjust threshold"
                info "  - Q: Quit visualizer"
                ;;
            "opengl")
                info "Visualizer Controls:"
                info "  - Mouse: Look around"
                info "  - W/A/S/D: Move camera"
                info "  - Space/C: Move up/down"
                info "  - +/-: Adjust threshold"
                info "  - R: Reset camera"
                info "  - T: Toggle trails"
                info "  - Q: Quit visualizer"
                ;;
        esac
    fi
    echo ""
    
    # Monitor loop
    while true; do
        sleep 5
        
        # Check if detector is still running
        if ! kill -0 $DETECTOR_PID 2>/dev/null; then
            error "Detection system has stopped unexpectedly!"
            error "Check log: $DETECTOR_LOG"
            cleanup
            exit 1
        fi
        
        # Check if visualizer is still running (if started)
        if [ "$SKIP_VISUALIZATION" != "true" ] && [ ! -z "$VISUALIZER_PID" ]; then
            if ! kill -0 $VISUALIZER_PID 2>/dev/null; then
                warning "Visualizer has stopped"
                VISUALIZER_PID=""
            fi
        fi
    done
}

# Parse command line arguments
DETECTION_CONFIG="$DEFAULT_CONFIG"
VISUALIZER_TYPE="auto"
VIZ_THRESHOLD="0.3"
ZMQ_PORT="5556"
DEMO_MODE="false"
SKIP_VISUALIZATION="false"
LOG_LEVEL="INFO"

while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--config)
            DETECTION_CONFIG="$2"
            shift 2
            ;;
        -v|--visualizer)
            VISUALIZER_TYPE="$2"
            shift 2
            ;;
        -t|--threshold)
            VIZ_THRESHOLD="$2"
            shift 2
            ;;
        -p|--port)
            ZMQ_PORT="$2"
            shift 2
            ;;
        -d|--demo)
            DEMO_MODE="true"
            shift
            ;;
        -n|--no-viz)
            SKIP_VISUALIZATION="true"
            shift
            ;;
        -l|--log-level)
            LOG_LEVEL="$2"
            shift 2
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Main execution
main() {
    show_banner
    
    log "Starting Orthanc Labs Drone Detection System"
    log "Configuration: $DETECTION_CONFIG"
    log "Visualizer: $VISUALIZER_TYPE"
    log "Threshold: $VIZ_THRESHOLD"
    log "Demo Mode: $DEMO_MODE"
    echo ""
    
    check_prerequisites
    detect_visualizer
    setup_logging
    
    # Start components
    start_detector
    start_visualizer
    
    # Monitor and wait
    monitor_system
}

# Run main function
main "$@" 