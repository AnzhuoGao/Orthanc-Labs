#!/bin/bash

# Orthanc Labs - Main Launch Script
# Convenient wrapper for the full system launcher

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Pass all arguments to the main launcher
exec "$SCRIPT_DIR/scripts/launch_system.sh" "$@" 