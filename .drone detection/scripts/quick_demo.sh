#!/bin/bash

# Quick Demo Script for Orthanc Labs Visualization
# This script quickly starts just the visualization in demo mode for testing

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}"
cat << 'EOF'
   ╔════════════════════════════════════════════╗
   ║           Orthanc Labs Quick Demo          ║
   ║        Visualization System Test           ║
   ╚════════════════════════════════════════════╝
EOF
echo -e "${NC}"

echo -e "${GREEN}Starting visualization demo...${NC}"
echo ""

# Change to project root
cd "$PROJECT_ROOT"

# Source visualization environment if it exists
if [ -f "visualization_env.sh" ]; then
    echo -e "${BLUE}Activating visualization environment...${NC}"
    source visualization_env.sh
else
    echo -e "${YELLOW}Visualization environment not found. Run setup first:${NC}"
    echo -e "${YELLOW}  ./scripts/setup_visualization.sh${NC}"
    echo ""
fi

# Determine best visualizer for platform
if [[ "$OSTYPE" == "darwin"* ]]; then
    VISUALIZER="scripts/visualize_simple.py"
    echo -e "${BLUE}Using simple visualizer (macOS compatible)${NC}"
else
    # Check if OpenGL is available
    if command -v glxinfo &> /dev/null && glxinfo &> /dev/null 2>&1; then
        VISUALIZER="scripts/visualize_voxel_grid.py"
        echo -e "${BLUE}Using OpenGL visualizer${NC}"
    else
        VISUALIZER="scripts/visualize_simple.py"
        echo -e "${BLUE}Using simple visualizer (OpenGL not available)${NC}"
    fi
fi

echo ""
echo -e "${GREEN}Controls:${NC}"
if [[ "$VISUALIZER" == *"simple"* ]]; then
    echo "  • Click and drag: Rotate view"
    echo "  • Scroll: Zoom in/out"
    echo "  • R key: Reset view"
    echo "  • +/- keys: Adjust threshold"
    echo "  • Q key: Quit"
else
    echo "  • Mouse: Look around"
    echo "  • W/A/S/D: Move camera"
    echo "  • Space/C: Move up/down"
    echo "  • +/- keys: Adjust threshold"
    echo "  • R key: Reset camera"
    echo "  • T key: Toggle trails"
    echo "  • Q key: Quit"
fi

echo ""
echo -e "${GREEN}Starting demo with animated objects...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo ""

# Start the visualizer in demo mode
python3 "$VISUALIZER" 0.3 true 