#!/bin/bash

# Setup script for Orthanc Labs Visualization System
# This script installs all required dependencies for the visualization components

set -e

echo "================================================="
echo "Orthanc Labs - Visualization Setup Script"
echo "================================================="

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is not installed. Please install Python 3.8 or higher."
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
echo "Found Python $PYTHON_VERSION"

# Check Python version (require 3.8+)
python3 -c 'import sys; exit(0 if sys.version_info >= (3, 8) else 1)' || {
    echo "Error: Python 3.8 or higher is required. Found Python $PYTHON_VERSION"
    exit 1
}

# Create virtual environment if it doesn't exist
VENV_DIR="venv_visualization"
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
fi

# Activate virtual environment
echo "Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install system dependencies (Ubuntu/Debian)
if command -v apt-get &> /dev/null; then
    echo "Installing system dependencies (Ubuntu/Debian)..."
    sudo apt-get update
    sudo apt-get install -y \
        libgl1-mesa-glx \
        libglu1-mesa \
        libegl1-mesa \
        libxrandr2 \
        libxss1 \
        libxcursor1 \
        libxcomposite1 \
        libasound2 \
        libxi6 \
        libxtst6 \
        libzmq3-dev \
        zlib1g-dev
        
# Install system dependencies (CentOS/RHEL/Fedora)
elif command -v yum &> /dev/null || command -v dnf &> /dev/null; then
    echo "Installing system dependencies (CentOS/RHEL/Fedora)..."
    if command -v dnf &> /dev/null; then
        PKG_MANAGER="dnf"
    else
        PKG_MANAGER="yum"
    fi
    
    sudo $PKG_MANAGER install -y \
        mesa-libGL \
        mesa-libGLU \
        libXrandr \
        libXScrnSaver \
        libXcursor \
        libXcomposite \
        libXi \
        libXtst \
        zeromq-devel \
        zlib-devel

# macOS with Homebrew
elif command -v brew &> /dev/null; then
    echo "Installing system dependencies (macOS)..."
    brew install zmq zlib
    
    # Install additional OpenGL dependencies for macOS
    echo "Setting up macOS OpenGL environment..."
    # Create the macOS-specific visualizer script
    if [ ! -f "scripts/visualize_voxel_grid_macos.py" ]; then
        echo "Warning: macOS-specific visualizer not found. Using cross-platform version."
    fi
    
else
    echo "Warning: Unable to detect package manager. Please install the following manually:"
    echo "  - OpenGL development libraries"
    echo "  - ZeroMQ development libraries"
    echo "  - zlib development libraries"
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip install -r scripts/requirements.txt

# Additional PyVista setup for headless servers
echo "Configuring PyVista for headless operation..."
python3 -c "
import pyvista as pv
pv.OFF_SCREEN = True
print('PyVista configured for headless operation')
"

# Create necessary directories
echo "Creating visualization directories..."
mkdir -p voxel_data screenshots

# Set up environment variables
echo "Setting up environment variables..."

# Detect platform and create appropriate environment script
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    cat > visualization_env.sh << 'EOF'
#!/bin/bash
# Orthanc Labs Visualization Environment (macOS)

# Activate virtual environment
source venv_visualization/bin/activate

# Set OpenGL platform for macOS
export PYOPENGL_PLATFORM=darwin

# PyVista settings for macOS
export PYVISTA_OFF_SCREEN=false
export PYVISTA_USE_PANEL=false

# ZMQ settings
export ZMQ_LINGER=1000

echo "Orthanc Labs visualization environment activated (macOS)"
echo "Use 'deactivate' to exit the virtual environment"
echo "For real-time viewer on macOS, use: python scripts/visualize_voxel_grid_macos.py"
EOF
else
    # Linux and others
    cat > visualization_env.sh << 'EOF'
#!/bin/bash
# Orthanc Labs Visualization Environment (Linux)

# Activate virtual environment
source venv_visualization/bin/activate

# Set OpenGL platform for headless operation
export PYOPENGL_PLATFORM=glx
export DISPLAY=${DISPLAY:-:0}

# PyVista settings
export PYVISTA_OFF_SCREEN=true
export PYVISTA_USE_PANEL=false

# ZMQ settings
export ZMQ_LINGER=1000

echo "Orthanc Labs visualization environment activated (Linux)"
echo "Use 'deactivate' to exit the virtual environment"
EOF
fi

chmod +x visualization_env.sh

# Test the installation
echo "Testing installation..."
python3 -c "
import sys
print('Testing Python packages...')

try:
    import numpy
    print('✓ NumPy:', numpy.__version__)
except ImportError as e:
    print('✗ NumPy import failed:', e)
    sys.exit(1)

try:
    import zmq
    print('✓ PyZMQ:', zmq.zmq_version())
except ImportError as e:
    print('✗ PyZMQ import failed:', e)
    sys.exit(1)

try:
    from OpenGL import GL
    print('✓ PyOpenGL: Available')
except ImportError as e:
    print('✗ PyOpenGL import failed:', e)
    sys.exit(1)

try:
    import pyvista as pv
    print('✓ PyVista:', pv.__version__)
except ImportError as e:
    print('✗ PyVista import failed:', e)
    sys.exit(1)

try:
    from sklearn.cluster import DBSCAN
    print('✓ scikit-learn: Available')
except ImportError as e:
    print('✗ scikit-learn import failed:', e)
    sys.exit(1)

print('\\nAll packages installed successfully!')
"

if [ $? -eq 0 ]; then
    echo ""
    echo "================================================="
    echo "✓ Visualization setup completed successfully!"
    echo "================================================="
    echo ""
    echo "To use the visualization tools:"
    echo "1. Activate the environment: source visualization_env.sh"
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "2. Run real-time viewer: python scripts/visualize_voxel_grid_macos.py"
    else
        echo "2. Run real-time viewer: python scripts/visualize_voxel_grid.py"
    fi
    echo "3. Run static analyzer: python scripts/static_voxel_viewer.py <file.bin> --interactive"
    echo ""
    echo "Configuration files:"
    echo "- Main config: config/visualization_config.json"
    echo "- Requirements: scripts/requirements.txt"
    echo ""
    echo "Directories created:"
    echo "- voxel_data/ (for saved voxel grids)"
    echo "- screenshots/ (for static analysis output)"
    echo ""
else
    echo ""
    echo "================================================="
    echo "✗ Setup failed during testing phase"
    echo "================================================="
    echo "Please check the error messages above and retry."
    exit 1
fi 