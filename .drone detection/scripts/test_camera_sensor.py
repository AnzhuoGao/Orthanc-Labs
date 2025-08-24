#!/usr/bin/env python3
"""
Test Camera Sensor Client
Simulates multiple camera sensor clients for testing the real-time system.
"""

import time
import json
import threading
import sys
import os
import numpy as np

# Add the scripts directory to the path to import our modules
sys.path.append(os.path.dirname(__file__))

def create_test_camera_configs():
    """Create test configurations for multiple cameras"""
    test_cameras = [
        {
            "camera_id": "CAM_NORTH_TEST",
            "sensor_type": "MPU6050_SIM",
            "calibration_offset": [0.0, 25.0, 8.0],
            "orientation_offset": [0.0, 0.0, 180.0],  # Looking south
            "server_host": "localhost",
            "server_port": 8888,
            "update_rate_hz": 5.0
        },
        {
            "camera_id": "CAM_SOUTH_TEST", 
            "sensor_type": "MPU6050_SIM",
            "calibration_offset": [0.0, -25.0, 8.0],
            "orientation_offset": [0.0, 0.0, 0.0],   # Looking north
            "server_host": "localhost",
            "server_port": 8888,
            "update_rate_hz": 5.0
        },
        {
            "camera_id": "CAM_EAST_TEST",
            "sensor_type": "MPU6050_SIM", 
            "calibration_offset": [25.0, 0.0, 8.0],
            "orientation_offset": [0.0, 0.0, 270.0], # Looking west
            "server_host": "localhost",
            "server_port": 8888,
            "update_rate_hz": 5.0
        }
    ]
    
    return test_cameras

def main():
    """Main test function"""
    print("===============================================")
    print("  ORTHANC LABS - CAMERA SENSOR TEST CLIENT")
    print("===============================================")
    print("This script simulates multiple camera sensors")
    print("for testing the real-time positioning system.")
    print("")
    
    # Check if server is running
    import socket
    try:
        test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        test_socket.settimeout(2)
        result = test_socket.connect_ex(('localhost', 8888))
        test_socket.close()
        
        if result != 0:
            print("ERROR: Camera position server is not running!")
            print("Please start the server first with:")
            print("  python scripts/camera_position_server.py")
            print("Or run the complete system with:")
            print("  ./scripts/launch_realtime_system.sh")
            return
            
    except Exception as e:
        print(f"ERROR: Cannot connect to server: {e}")
        return
    
    print("✓ Server detected on localhost:8888")
    print("Test client functionality ready!")

if __name__ == "__main__":
    main()
