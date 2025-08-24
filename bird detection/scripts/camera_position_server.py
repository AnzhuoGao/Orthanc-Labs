#!/usr/bin/env python3
"""
Bird Detection Camera Position Server
Receives real-time sensor data from camera Raspberry Pis and updates camera configuration.

This server:
1. Listens for connections from bird detection camera sensor clients
2. Receives real-time position/orientation updates
3. Updates camera configuration JSON files
4. Provides real-time data feed to bird detection visualizers
5. Handles multiple camera connections simultaneously
"""

import sys
import os
import json
import socket
import threading
import time
import numpy as np
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
import logging
from queue import Queue, Empty
import signal

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('BirdCameraPositionServer')

@dataclass
class CameraState:
    """Current state of a camera"""
    camera_id: str
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float]  # roll, pitch, yaw in degrees
    last_update: float
    connection_status: str
    sensor_status: str
    confidence: float

class CameraPositionServer:
    """Server for receiving and managing camera position data"""
    
    def __init__(self, port: int = 8888, config_dir: str = "/Users/andrewgao/Desktop/Orthanc Labs/config"):
        self.port = port
        self.config_dir = config_dir
        self.running = False
        self.server_socket = None
        
        # Camera management
        self.cameras: Dict[str, CameraState] = {}
        self.camera_connections: Dict[str, socket.socket] = {}
        self.data_queue = Queue()
        
        # Configuration files
        self.camera_config_file = os.path.join(config_dir, "camera_positions.json")
        self.backup_config_file = os.path.join(config_dir, "camera_positions_backup.json")
        
        # Ensure config directory exists
        os.makedirs(config_dir, exist_ok=True)
        
        # Load existing camera configuration
        self.load_camera_config()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        logger.info(f"Received signal {signum}, shutting down...")
        self.stop()
        sys.exit(0)
    
    def load_camera_config(self):
        """Load existing camera configuration"""
        try:
            if os.path.exists(self.camera_config_file):
                with open(self.camera_config_file, 'r') as f:
                    config_data = json.load(f)
                
                # Convert to CameraState objects
                for camera_id, camera_data in config_data.items():
                    self.cameras[camera_id] = CameraState(
                        camera_id=camera_id,
                        position=tuple(camera_data.get('position', [0, 0, 8])),
                        orientation=tuple(camera_data.get('orientation', [0, 0, 0])),
                        last_update=time.time(),
                        connection_status='OFFLINE',
                        sensor_status='UNKNOWN',
                        confidence=0.0
                    )
                
                logger.info(f"Loaded configuration for {len(self.cameras)} cameras")
            else:
                logger.info("No existing camera configuration found")
                
        except Exception as e:
            logger.error(f"Failed to load camera configuration: {e}")
    
    def save_camera_config(self, backup: bool = True):
        """Save current camera configuration to JSON"""
        try:
            # Create backup if requested
            if backup and os.path.exists(self.camera_config_file):
                import shutil
                shutil.copy2(self.camera_config_file, self.backup_config_file)
            
            # Convert camera states to serializable format
            config_data = {}
            for camera_id, camera_state in self.cameras.items():
                # Convert orientation from euler angles to direction vector for visualizer
                roll, pitch, yaw = camera_state.orientation
                
                # Convert to radians
                roll_rad = np.radians(roll)
                pitch_rad = np.radians(pitch)
                yaw_rad = np.radians(yaw)
                
                # Calculate direction vector from euler angles
                # Standard aerospace convention: yaw (Z), pitch (Y), roll (X)
                direction_x = np.cos(pitch_rad) * np.cos(yaw_rad)
                direction_y = np.cos(pitch_rad) * np.sin(yaw_rad)
                direction_z = -np.sin(pitch_rad)
                
                config_data[camera_id] = {
                    'position': list(camera_state.position),
                    'direction': [direction_x, direction_y, direction_z],
                    'orientation': list(camera_state.orientation),  # Keep original for reference
                    'fov_degrees': 60,  # Default FOV
                    'type': 'realtime',
                    'max_range': 40,
                    'last_update': camera_state.last_update,
                    'sensor_status': camera_state.sensor_status,
                    'confidence': camera_state.confidence
                }
            
            # Write to file
            with open(self.camera_config_file, 'w') as f:
                json.dump(config_data, f, indent=2)
            
            logger.debug(f"Saved configuration for {len(config_data)} cameras")
            
        except Exception as e:
            logger.error(f"Failed to save camera configuration: {e}")
    
    def receive_message(self, connection: socket.socket) -> Optional[dict]:
        """Receive JSON message from client"""
        try:
            # Receive length header (4 bytes)
            length_data = connection.recv(4)
            if len(length_data) != 4:
                return None
            
            message_length = int.from_bytes(length_data, byteorder='big')
            
            # Receive message data
            message_data = b''
            while len(message_data) < message_length:
                chunk = connection.recv(message_length - len(message_data))
                if not chunk:
                    return None
                message_data += chunk
            
            # Parse JSON
            return json.loads(message_data.decode('utf-8'))
            
        except Exception as e:
            logger.debug(f"Error receiving message: {e}")
            return None
    
    def handle_camera_connection(self, connection: socket.socket, address: Tuple[str, int]):
        """Handle connection from a camera client"""
        camera_id = None
        logger.info(f"New camera connection from {address}")
        
        try:
            while self.running:
                message = self.receive_message(connection)
                
                if not message:
                    break
                
                message_type = message.get('type', '')
                
                if message_type == 'CAMERA_IDENTIFY':
                    # Camera identification
                    camera_id = message.get('camera_id', 'UNKNOWN')
                    sensor_type = message.get('sensor_type', 'UNKNOWN')
                    
                    logger.info(f"Camera identified: {camera_id} ({sensor_type}) from {address}")
                    
                    # Register camera connection
                    self.camera_connections[camera_id] = connection
                    
                    # Initialize camera state if new
                    if camera_id not in self.cameras:
                        self.cameras[camera_id] = CameraState(
                            camera_id=camera_id,
                            position=(0.0, 0.0, 8.0),  # Default position
                            orientation=(0.0, 0.0, 0.0),  # Default orientation
                            last_update=time.time(),
                            connection_status='ONLINE',
                            sensor_status='INITIALIZING',
                            confidence=0.0
                        )
                    else:
                        self.cameras[camera_id].connection_status = 'ONLINE'
                
                elif message_type == 'SENSOR_UPDATE':
                    # Sensor data update
                    if camera_id:
                        sensor_data = message.get('data', {})
                        
                        # Update camera state
                        self.cameras[camera_id].position = tuple(sensor_data.get('position', [0, 0, 8]))
                        self.cameras[camera_id].orientation = tuple(sensor_data.get('orientation', [0, 0, 0]))
                        self.cameras[camera_id].last_update = sensor_data.get('timestamp', time.time())
                        self.cameras[camera_id].sensor_status = sensor_data.get('sensor_status', 'UNKNOWN')
                        self.cameras[camera_id].confidence = sensor_data.get('confidence', 0.0)
                        
                        # Add to data queue for real-time processing
                        self.data_queue.put({
                            'camera_id': camera_id,
                            'data': sensor_data,
                            'timestamp': time.time()
                        })
                        
                        logger.debug(f"Updated {camera_id}: Pos={self.cameras[camera_id].position}, "
                                   f"Ori={self.cameras[camera_id].orientation}")
                
        except Exception as e:
            logger.error(f"Error handling camera {camera_id}: {e}")
        
        finally:
            # Cleanup connection
            if camera_id:
                if camera_id in self.camera_connections:
                    del self.camera_connections[camera_id]
                
                if camera_id in self.cameras:
                    self.cameras[camera_id].connection_status = 'OFFLINE'
                
                logger.info(f"Camera {camera_id} disconnected")
            
            try:
                connection.close()
            except:
                pass
    
    def config_update_loop(self):
        """Background loop to update configuration files"""
        logger.info("Starting configuration update loop...")
        
        last_save = 0
        save_interval = 2.0  # Save every 2 seconds if there are updates
        
        while self.running:
            try:
                # Process data queue
                updates_count = 0
                
                while not self.data_queue.empty():
                    try:
                        self.data_queue.get_nowait()
                        updates_count += 1
                    except Empty:
                        break
                
                # Save configuration if there were updates
                if updates_count > 0 and (time.time() - last_save) > save_interval:
                    self.save_camera_config()
                    last_save = time.time()
                    logger.debug(f"Processed {updates_count} camera updates")
                
                # Check for offline cameras
                current_time = time.time()
                for camera_id, camera_state in self.cameras.items():
                    if (current_time - camera_state.last_update) > 30:  # 30 seconds timeout
                        if camera_state.connection_status != 'OFFLINE':
                            camera_state.connection_status = 'OFFLINE'
                            camera_state.sensor_status = 'TIMEOUT'
                            logger.warning(f"Camera {camera_id} timed out")
                
                time.sleep(0.5)  # Check every 500ms
                
            except Exception as e:
                logger.error(f"Error in config update loop: {e}")
                time.sleep(1)
    
    def status_loop(self):
        """Background loop to print status information"""
        while self.running:
            try:
                time.sleep(10)  # Print status every 10 seconds
                
                if self.cameras:
                    logger.info(f"Camera Status - Total: {len(self.cameras)}")
                    for camera_id, camera_state in self.cameras.items():
                        status = f"{camera_id}: {camera_state.connection_status}"
                        if camera_state.connection_status == 'ONLINE':
                            status += f" (Pos: {camera_state.position}, Conf: {camera_state.confidence:.2f})"
                        logger.info(f"  {status}")
                else:
                    logger.info("No cameras connected")
                    
            except Exception as e:
                logger.error(f"Error in status loop: {e}")
    
    def start(self):
        """Start the camera position server"""
        logger.info(f"Starting Camera Position Server on port {self.port}")
        
        try:
            # Create server socket
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.port))
            self.server_socket.listen(10)
            
            self.running = True
            
            # Start background threads
            config_thread = threading.Thread(target=self.config_update_loop, daemon=True)
            config_thread.start()
            
            status_thread = threading.Thread(target=self.status_loop, daemon=True)
            status_thread.start()
            
            logger.info(f"Server listening on port {self.port}")
            logger.info(f"Camera configuration will be saved to: {self.camera_config_file}")
            
            # Accept connections
            while self.running:
                try:
                    connection, address = self.server_socket.accept()
                    
                    # Handle each camera in a separate thread
                    camera_thread = threading.Thread(
                        target=self.handle_camera_connection,
                        args=(connection, address),
                        daemon=True
                    )
                    camera_thread.start()
                    
                except socket.error as e:
                    if self.running:  # Only log if we're supposed to be running
                        logger.error(f"Socket error: {e}")
                    break
                
        except Exception as e:
            logger.error(f"Failed to start server: {e}")
        
        finally:
            self.stop()
    
    def stop(self):
        """Stop the server"""
        logger.info("Stopping Camera Position Server...")
        self.running = False
        
        # Close all camera connections
        for camera_id, connection in list(self.camera_connections.items()):
            try:
                connection.close()
            except:
                pass
        
        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        # Final save
        self.save_camera_config()
        
        logger.info("Camera Position Server stopped")

def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Orthanc Labs Camera Position Server")
    parser.add_argument("--port", type=int, default=8888, help="Server port")
    parser.add_argument("--config-dir", default="/Users/andrewgao/Desktop/Orthanc Labs/config",
                       help="Configuration directory")
    
    args = parser.parse_args()
    
    # Create and start server
    server = CameraPositionServer(port=args.port, config_dir=args.config_dir)
    
    try:
        server.start()
    except KeyboardInterrupt:
        logger.info("Server interrupted by user")
    finally:
        server.stop()

if __name__ == "__main__":
    main()
