#!/bin/bash

# Orthanc Labs - Unified Launch Script
# Choose between drone detection and bird detection systems

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Orthanc Labs - Detection Systems"
echo "================================"
echo "Available systems:"
echo "  1) Drone Detection System"
echo "  2) Bird Detection System"
echo ""

if [ "$1" == "drone" ]; then
    echo "Launching Drone Detection System..."
    cd "$SCRIPT_DIR/drone detection"
    exec ./scripts/launch_realtime_system.sh
elif [ "$1" == "bird" ]; then
    echo "Launching Bird Detection System..."
    cd "$SCRIPT_DIR/bird detection"
    exec ./scripts/launch_realtime_system.sh
else
    echo "Usage: $0 [drone|bird]"
    echo ""
    echo "Examples:"
    echo "  $0 drone    # Launch drone detection system"
    echo "  $0 bird     # Launch bird detection system"
    echo ""
    echo "Or navigate to the specific folder and run:"
    echo "  cd 'drone detection' && ./scripts/launch_realtime_system.sh"
    echo "  cd 'bird detection' && ./scripts/launch_realtime_system.sh"
    exit 1
fi 