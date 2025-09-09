#!/usr/bin/env python3
"""
Raspberry Pi Camera Sensor Data Collection Client
Collects position and orientation data from sensors and transmits to main server.

This script will be deployed on each camera's Raspberry Pi to:
1. Read position/orientation data from sensors
2. Transmit real-time updates to the main visualization server
3. Handle sensor initialization and error recovery

Supported sensors (to be selected later):
- IMU/Gyroscope: MPU-6050, MPU-9250, BNO055
- GPS: NEO-8M, NEO-6M modules  
- Magnetometer: HMC5883L, QMC5883L
- Accelerometer: ADXL345, LSM6DS3
"""

import sys
import os
import time
import json
import socket
import threading
import numpy as np
from dataclasses import dataclass, asdict
from typing import Optional, Dict, List, Tuple
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/orthanc_camera_sensor.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('CameraSensor')

@dataclass
class SensorReading:
    """Sensor data structure"""
    timestamp: float
    camera_id: str
    position: Tuple[float, float, float]  # (x, y, z) in meters
    orientation: Tuple[float, float, float]  # (roll, pitch, yaw) in degrees
    confidence: float  # Sensor reading confidence 0-1
    sensor_status: str  # 'ACTIVE', 'DEGRADED', 'ERROR'

@dataclass
class CameraConfig:
    """Camera configuration"""
    camera_id: str
    sensor_type: str
    calibration_offset: Tuple[float, float, float]
    orientation_offset: Tuple[float, float, float]
    server_host: str
    server_port: int
    update_rate_hz: float

class SensorInterface:
    """Abstract sensor interface - will be implemented for specific sensors"""
    
    def __init__(self, config: CameraConfig):
        self.config = config
        self.is_initialized = False
        self.last_reading = None
        
    def initialize(self) -> bool:
        """Initialize sensor hardware"""
        # Placeholder - will implement for specific sensors
        logger.info(f"Initializing {self.config.sensor_type} sensor...")
        
        # Simulated initialization
        time.sleep(1)
        self.is_initialized = True
        logger.info("Sensor initialized successfully")
        return True
    
    def read_position(self) -> Optional[Tuple[float, float, float]]:
        """Read position from GPS/positioning sensor"""
        if not self.is_initialized:
            return None
            
        # Placeholder implementation - returns simulated position
        # This will be replaced with actual sensor reading code
        base_x, base_y, base_z = self.config.calibration_offset
        
        # Add small random variation to simulate sensor noise
        noise_factor = 0.1
        x = base_x + np.random.normal(0, noise_factor)
        y = base_y + np.random.normal(0, noise_factor)
        z = base_z + np.random.normal(0, noise_factor)
        
        return (x, y, z)
    
    def read_orientation(self) -> Optional[Tuple[float, float, float]]:
        """Read orientation from IMU/gyroscope"""
        if not self.is_initialized:
            return None
            
        # Placeholder implementation - returns simulated orientation
        # This will be replaced with actual IMU reading code
        base_roll, base_pitch, base_yaw = self.config.orientation_offset
        
        # Add small random variation
        noise_factor = 1.0  # degrees
        roll = base_roll + np.random.normal(0, noise_factor)
        pitch = base_pitch + np.random.normal(0, noise_factor)
        yaw = base_yaw + np.random.normal(0, noise_factor)
        
        return (roll, pitch, yaw)
    
    def get_sensor_status(self) -> str:
        """Get current sensor health status"""
        if not self.is_initialized:
            return "ERROR"
        
        # Simple health check - can be expanded
        if self.last_reading and (time.time() - self.last_reading) > 5.0:
            return "DEGRADED"
        
        return "ACTIVE"

class CameraSensorClient:
    """Main camera sensor client for Raspberry Pi"""
    
    def __init__(self, config_file: str = "/etc/orthanc/camera_config.json"):
        self.config_file = config_file
        self.config = None
        self.sensor = None
        self.running = False
        self.connection = None
        self.last_transmission = 0
        
        # Load configuration
        self.load_config()
        
        # Initialize sensor
        if self.config:
            self.sensor = SensorInterface(self.config)
            
    def load_config(self):
        """Load camera configuration from file"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    config_data = json.load(f)
                    
                self.config = CameraConfig(**config_data)
                logger.info(f"Loaded config for camera {self.config.camera_id}")
                
            else:
                # Create default configuration
                logger.warning("No config file found, creating default...")
                self.create_default_config()
                
        except Exception as e:
            logger.error(f"Failed to load config: {e}")
            self.create_default_config()
    
    def create_default_config(self):
        """Create default configuration file"""
        default_config = {
            "camera_id": f"CAM_{socket.gethostname().upper()}",
            "sensor_type": "MPU6050",  # Default IMU
            "calibration_offset": [0.0, 0.0, 8.0],  # Default position
            "orientation_offset": [0.0, 0.0, 0.0],  # Default orientation
            "server_host": "192.168.1.100",  # Main server IP
            "server_port": 8888,
            "update_rate_hz": 10.0
        }
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
        
        with open(self.config_file, 'w') as f:
            json.dump(default_config, f, indent=2)
            
        self.config = CameraConfig(**default_config)
        logger.info(f"Created default config at {self.config_file}")
    
    def connect_to_server(self) -> bool:
        """Establish connection to main server"""
        try:
            self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connection.connect((self.config.server_host, self.config.server_port))
            
            # Send identification message
            identify_msg = {
                "type": "CAMERA_IDENTIFY",
                "camera_id": self.config.camera_id,
                "sensor_type": self.config.sensor_type
            }
            
            self.send_message(identify_msg)
            logger.info(f"Connected to server {self.config.server_host}:{self.config.server_port}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to server: {e}")
            return False
    
    def send_message(self, message: dict):
        """Send JSON message to server"""
        if not self.connection:
            return False
            
        try:
            json_data = json.dumps(message).encode('utf-8')
            length = len(json_data)
            
            # Send length header followed by data
            self.connection.sendall(length.to_bytes(4, byteorder='big'))
            self.connection.sendall(json_data)
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to send message: {e}")
            return False
    
    def collect_sensor_data(self) -> Optional[SensorReading]:
        """Collect data from all sensors"""
        if not self.sensor or not self.sensor.is_initialized:
            return None
            
        try:
            # Read position and orientation
            position = self.sensor.read_position()
            orientation = self.sensor.read_orientation()
            
            if not position or not orientation:
                return None
            
            # Create sensor reading
            reading = SensorReading(
                timestamp=time.time(),
                camera_id=self.config.camera_id,
                position=position,
                orientation=orientation,
                confidence=0.95,  # Could be calculated based on sensor quality
                sensor_status=self.sensor.get_sensor_status()
            )
            
            self.sensor.last_reading = reading.timestamp
            return reading
            
        except Exception as e:
            logger.error(f"Error collecting sensor data: {e}")
            return None
    
    def run_sensor_loop(self):
        """Main sensor data collection loop"""
        logger.info("Starting sensor data collection loop...")
        
        update_interval = 1.0 / self.config.update_rate_hz
        
        while self.running:
            try:
                # Collect sensor data
                reading = self.collect_sensor_data()
                
                if reading:
                    # Send to server
                    message = {
                        "type": "SENSOR_UPDATE",
                        "data": asdict(reading)
                    }
                    
                    if self.send_message(message):
                        self.last_transmission = time.time()
                        
                        if time.time() % 10 < 1:  # Log every 10 seconds
                            logger.info(f"Sensor update sent: {reading.camera_id} "
                                      f"Pos: {reading.position} Ori: {reading.orientation}")
                    else:
                        logger.warning("Failed to send sensor update")
                
                time.sleep(update_interval)
                
            except KeyboardInterrupt:
                logger.info("Sensor loop interrupted by user")
                break
            except Exception as e:
                logger.error(f"Error in sensor loop: {e}")
                time.sleep(1)  # Brief pause before retry
    
    def start(self):
        """Start the camera sensor client"""
        logger.info(f"Starting Camera Sensor Client for {self.config.camera_id}")
        
        # Initialize sensor
        if not self.sensor.initialize():
            logger.error("Failed to initialize sensor")
            return False
        
        # Connect to server
        if not self.connect_to_server():
            logger.error("Failed to connect to server")
            return False
        
        # Start sensor data collection
        self.running = True
        self.run_sensor_loop()
        
        return True
    
    def stop(self):
        """Stop the sensor client"""
        logger.info("Stopping Camera Sensor Client...")
        self.running = False
        
        if self.connection:
            try:
                self.connection.close()
            except:
                pass
        
        logger.info("Camera Sensor Client stopped")

def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Orthanc Labs Camera Sensor Client")
    parser.add_argument("--config", default="/etc/orthanc/camera_config.json",
                       help="Configuration file path")
    parser.add_argument("--camera-id", help="Override camera ID")
    parser.add_argument("--server-host", help="Override server host")
    parser.add_argument("--server-port", type=int, help="Override server port")
    
    args = parser.parse_args()
    
    # Create sensor client
    client = CameraSensorClient(args.config)
    
    # Override config if specified
    if args.camera_id:
        client.config.camera_id = args.camera_id
    if args.server_host:
        client.config.server_host = args.server_host
    if args.server_port:
        client.config.server_port = args.server_port
    
    try:
        # Start client
        client.start()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        client.stop()

if __name__ == "__main__":
    main()
