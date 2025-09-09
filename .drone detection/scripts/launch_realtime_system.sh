#!/bin/bash
"""
Orthanc Labs Real-Time Camera System Launcher
Starts the camera position server and visualizer for real-time operation.
"""

set -e  # Exit on any error

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "======================================================="
echo "  ORTHANC LABS - REAL-TIME CAMERA MONITORING SYSTEM"
echo "======================================================="
echo "Starting real-time camera position tracking..."
echo ""

# Activate virtual environment if it exists
if [ -d "$PROJECT_ROOT/venv_visualization" ]; then
    echo "Activating visualization environment..."
    source "$PROJECT_ROOT/venv_visualization/bin/activate"
fi

# Function to cleanup background processes
cleanup() {
    echo ""
    echo "Shutting down real-time camera system..."
    
    # Kill camera position server
    if [ ! -z "$SERVER_PID" ]; then
        echo "Stopping camera position server (PID: $SERVER_PID)"
        kill $SERVER_PID 2>/dev/null || true
        wait $SERVER_PID 2>/dev/null || true
    fi
    
    # Kill visualizer
    if [ ! -z "$VISUALIZER_PID" ]; then
        echo "Stopping visualizer (PID: $VISUALIZER_PID)"
        kill $VISUALIZER_PID 2>/dev/null || true
        wait $VISUALIZER_PID 2>/dev/null || true
    fi
    
    echo "Real-time camera system shutdown complete"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Start camera position server
echo "Starting camera position server..."
python "$SCRIPT_DIR/camera_position_server.py" --port 8888 --config-dir "$PROJECT_ROOT/config" &
SERVER_PID=$!

# Wait a moment for server to start
sleep 2

# Check if server started successfully
if ! kill -0 $SERVER_PID 2>/dev/null; then
    echo "ERROR: Camera position server failed to start"
    exit 1
fi

echo "Camera position server started (PID: $SERVER_PID)"
echo "Server listening on port 8888"
echo "Configuration directory: $PROJECT_ROOT/config"
echo ""

# Start visualizer
echo "Starting real-time visualizer..."
echo "The visualizer will load camera positions from real-time data"
echo ""

python "$SCRIPT_DIR/visualize_simple.py" &
VISUALIZER_PID=$!

# Wait a moment for visualizer to start
sleep 2

# Check if visualizer started successfully
if ! kill -0 $VISUALIZER_PID 2>/dev/null; then
    echo "ERROR: Visualizer failed to start"
    cleanup
    exit 1
fi

echo "Visualizer started (PID: $VISUALIZER_PID)"
echo ""
echo "======================================================="
echo "  REAL-TIME SYSTEM STATUS"
echo "======================================================="
echo "✓ Camera Position Server: RUNNING (Port 8888)"
echo "✓ Real-Time Visualizer: RUNNING"
echo ""
echo "CAMERA CONNECTION INSTRUCTIONS:"
echo "1. Configure Raspberry Pi clients with server IP and port 8888"
echo "2. Run camera_sensor_client.py on each Raspberry Pi"
echo "3. Camera positions will update automatically in the visualizer"
echo ""
echo "CONTROLS:"
echo "- Camera positions update automatically from sensor data"
echo "- Use visualizer controls: C=cameras, G=FOV, V/A=vectors"
echo "- Click on cameras to view detailed information"
echo "- Press Ctrl+C to stop the entire system"
echo ""
echo "Monitoring system... (Press Ctrl+C to stop)"

# Monitor both processes
while true do
    # Check if server is still running
    if ! kill -0 $SERVER_PID 2>/dev/null; then
        echo "ERROR: Camera position server has stopped"
        cleanup
        exit 1
    fi
    
    # Check if visualizer is still running
    if ! kill -0 $VISUALIZER_PID 2>/dev/null; then
        echo "ERROR: Visualizer has stopped"
        cleanup
        exit 1
    fi
    
    sleep 5
done
