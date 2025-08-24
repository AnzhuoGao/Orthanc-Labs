#!/bin/bash
# Orthanc Labs Visualization Environment

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

echo "Orthanc Labs visualization environment activated"
echo "Use 'deactivate' to exit the virtual environment"
