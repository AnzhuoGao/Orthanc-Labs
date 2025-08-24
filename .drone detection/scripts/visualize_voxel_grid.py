#!/usr/bin/env python3
"""
Real-time 3D Voxel Grid Visualizer for Orthanc Labs Drone Detection System

This script monitors voxel grid data from the drone detection system and 
visualizes detected objects in real-time using OpenGL.

Usage:
    python visualize_voxel_grid.py [threshold] [zmq_port]
    
    - threshold: Optional brightness threshold for visualization (default: 0.3)
    - zmq_port: Optional ZMQ port for live data (default: 5556)

Controls:
    - Mouse: Look around
    - W/A/S/D: Move camera
    - Space/C: Move up/down
    - +/-: Adjust threshold
    - R: Reset camera position
    - T: Toggle tracking trails
    - Q: Quit

TO RUN:
    export PYOPENGL_PLATFORM=glx
    export DISPLAY=${DISPLAY:-:0}
    python scripts/visualize_voxel_grid.py
"""

import sys
import os
import time
import numpy as np
import struct
import zlib
import zmq
from threading import Thread, Lock
from collections import deque

# OpenGL visualization
try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    from OpenGL.GLUT import *
except ImportError:
    print("ERROR: PyOpenGL not installed. Install with:")
    print("pip install PyOpenGL PyOpenGL_accelerate")
    sys.exit(1)

# Camera control
class Camera:
    def __init__(self):
        self.pos = np.array([0.0, 0.0, 50.0])
        self.yaw = 0.0
        self.pitch = -20.0
        self.speed = 2.0
        self.mouse_sensitivity = 0.2
        self.last_x = 0
        self.last_y = 0
        self.first_mouse = True
        
    def reset(self):
        """Reset camera to default position"""
        self.pos = np.array([0.0, 0.0, 50.0])
        self.yaw = 0.0
        self.pitch = -20.0
        
    def update_view(self):
        # Convert to radians
        yaw_rad = np.radians(self.yaw)
        pitch_rad = np.radians(self.pitch)
        
        # Calculate the camera's direction vector
        direction = np.array([
            np.cos(yaw_rad) * np.cos(pitch_rad),
            np.sin(pitch_rad),
            np.sin(yaw_rad) * np.cos(pitch_rad)
        ])
        
        # Normalize
        direction = direction / np.linalg.norm(direction)
        
        # Set the view matrix using gluLookAt
        target = self.pos + direction
        gluLookAt(
            self.pos[0], self.pos[1], self.pos[2],
            target[0], target[1], target[2],
            0.0, 1.0, 0.0
        )

# Detection tracking for trails
class DetectionTracker:
    def __init__(self, max_trail_length=50):
        self.tracks = {}  # track_id -> deque of positions
        self.max_trail_length = max_trail_length
        self.show_trails = True
        self.mutex = Lock()
    
    def update_detection(self, detection_id, position):
        with self.mutex:
            if detection_id not in self.tracks:
                self.tracks[detection_id] = deque(maxlen=self.max_trail_length)
            self.tracks[detection_id].append(position.copy())
    
    def clear_old_tracks(self, active_ids):
        with self.mutex:
            # Remove tracks that are no longer active
            inactive_ids = set(self.tracks.keys()) - set(active_ids)
            for track_id in inactive_ids:
                del self.tracks[track_id]
    
    def draw_trails(self):
        if not self.show_trails:
            return
        
        with self.mutex:
            glLineWidth(2.0)
            for track_id, trail in self.tracks.items():
                if len(trail) < 2:
                    continue
                
                # Color based on track ID
                hue = (track_id * 137.5) % 360  # Golden angle for good separation
                r = 0.5 + 0.5 * np.cos(np.radians(hue))
                g = 0.5 + 0.5 * np.cos(np.radians(hue + 120))
                b = 0.5 + 0.5 * np.cos(np.radians(hue + 240))
                
                glColor3f(r, g, b)
                glBegin(GL_LINE_STRIP)
                for i, pos in enumerate(trail):
                    # Fade older positions
                    alpha = (i + 1) / len(trail)
                    glColor4f(r, g, b, alpha)
                    glVertex3f(pos[0], pos[1], pos[2])
                glEnd()

# Voxel Grid data with drone detection info
class VoxelGrid:
    def __init__(self):
        self.grid = None
        self.grid_size = [0, 0, 0]
        self.voxel_size = 0.0
        self.grid_origin = [0.0, 0.0, 0.0]
        self.threshold = 0.3
        self.detections = []
        self.last_modified_time = 0
        self.mutex = Lock()
        self.display_list = None
        self.display_list_valid = False
        
    def update_from_zmq(self, voxel_data, detection_data=None):
        """Update voxel grid from ZMQ data"""
        with self.mutex:
            try:
                # Parse voxel data
                if len(voxel_data) >= 20:  # Header: 3 ints (grid_size) + 1 float (voxel_size) + 3 floats (origin)
                    header = struct.unpack('iiifff', voxel_data[:20])
                    self.grid_size = [header[0], header[1], header[2]]
                    self.voxel_size = header[3]
                    self.grid_origin = [header[4], header[5], header[6]]
                    
                    # Extract grid data
                    grid_data_size = self.grid_size[0] * self.grid_size[1] * self.grid_size[2]
                    grid_bytes = voxel_data[20:20 + grid_data_size * 4]
                    
                    if len(grid_bytes) >= grid_data_size * 4:
                        self.grid = np.frombuffer(grid_bytes, dtype=np.float32)
                        self.grid = self.grid.reshape(self.grid_size)
                        self.display_list_valid = False
                        
                # Parse detection data if available
                if detection_data:
                    self.detections = self._parse_detections(detection_data)
                    
                return True
            except Exception as e:
                print(f"Error updating voxel grid: {e}")
                return False
    
    def _parse_detections(self, detection_data):
        """Parse detection data from binary format"""
        detections = []
        try:
            offset = 0
            while offset < len(detection_data):
                if offset + 32 > len(detection_data):  # Minimum detection size
                    break
                
                # Parse detection: id, position (3 floats), confidence, velocity (3 floats)
                detection = struct.unpack('iffffff', detection_data[offset:offset+32])
                detections.append({
                    'id': detection[0],
                    'position': np.array([detection[1], detection[2], detection[3]]),
                    'confidence': detection[4],
                    'velocity': np.array([detection[5], detection[6], detection[7]])
                })
                offset += 32
        except Exception as e:
            print(f"Error parsing detections: {e}")
        
        return detections
    
    def create_display_list(self):
        if self.grid is None:
            return
        
        with self.mutex:
            # Delete previous display list if it exists
            if self.display_list is not None:
                glDeleteLists(self.display_list, 1)
            
            # Create a new display list
            self.display_list = glGenLists(1)
            glNewList(self.display_list, GL_COMPILE)
            
            # Draw voxels
            for i in range(self.grid_size[0]):
                for j in range(self.grid_size[1]):
                    for k in range(self.grid_size[2]):
                        val = self.grid[i, j, k]
                        if val > self.threshold:
                            # Normalize value for color intensity
                            normalized_val = min(1.0, val / (self.threshold * 3.0))
                            
                            # Position in world space
                            x = self.grid_origin[0] + i * self.voxel_size
                            y = self.grid_origin[1] + j * self.voxel_size
                            z = self.grid_origin[2] + k * self.voxel_size
                            
                            # Color based on intensity (red-orange-yellow)
                            if normalized_val > 0.7:
                                glColor4f(1.0, 1.0, 0.2, 0.8)  # Bright yellow
                            elif normalized_val > 0.4:
                                glColor4f(1.0, 0.6, 0.0, 0.6)  # Orange
                            else:
                                glColor4f(1.0, 0.2, 0.0, 0.4)  # Red
                            
                            glPushMatrix()
                            glTranslatef(x, y, z)
                            
                            # Use different shapes based on intensity
                            if normalized_val > 0.6:
                                glutSolidCube(self.voxel_size * 0.8)
                            else:
                                glPointSize(3.0)
                                glBegin(GL_POINTS)
                                glVertex3f(0, 0, 0)
                                glEnd()
                                
                            glPopMatrix()
            
            glEndList()
            self.display_list_valid = True
    
    def draw_detections(self, tracker):
        """Draw detected drones with enhanced visualization"""
        if not self.detections:
            return
        
        for detection in self.detections:
            pos = detection['position']
            confidence = detection['confidence']
            detection_id = detection['id']
            
            # Update tracker
            tracker.update_detection(detection_id, pos)
            
            # Draw detection as a highlighted object
            glPushMatrix()
            glTranslatef(pos[0], pos[1], pos[2])
            
            # Color based on confidence
            if confidence > 0.8:
                glColor4f(0.0, 1.0, 0.0, 0.9)  # High confidence - green
            elif confidence > 0.5:
                glColor4f(1.0, 1.0, 0.0, 0.8)  # Medium confidence - yellow
            else:
                glColor4f(1.0, 0.5, 0.0, 0.7)  # Low confidence - orange
            
            # Draw detection marker
            glutSolidSphere(self.voxel_size * 2, 8, 8)
            
            # Draw velocity vector if available
            velocity = detection['velocity']
            if np.linalg.norm(velocity) > 0.1:  # Only draw if significant velocity
                glColor3f(0.0, 0.0, 1.0)  # Blue for velocity
                glLineWidth(3.0)
                glBegin(GL_LINES)
                glVertex3f(0, 0, 0)
                glVertex3f(velocity[0], velocity[1], velocity[2])
                glEnd()
            
            glPopMatrix()

# Global variables
voxel_grid = VoxelGrid()
camera = Camera()
tracker = DetectionTracker()
running = True
width, height = 1400, 900
keys_pressed = set()

# ZeroMQ live listener
def zmq_listener(port=5556):
    """Listen for live voxel data via ZeroMQ"""
    try:
        ctx = zmq.Context.instance()
        sub = ctx.socket(zmq.SUB)
        sub.connect(f"tcp://127.0.0.1:{port}")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        sub.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout

        global running
        while running:
            try:
                # Receive voxel data
                voxel_msg = sub.recv(flags=zmq.NOBLOCK)
                
                # Try to receive detection data (optional)
                detection_msg = None
                try:
                    detection_msg = sub.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass  # No detection data available
                
                # Update voxel grid
                voxel_grid.update_from_zmq(voxel_msg, detection_msg)
                
            except zmq.Again:
                time.sleep(0.01)
                continue
            except Exception as e:
                print(f"ZMQ listener error: {e}")
                time.sleep(1.0)
                
    except Exception as e:
        print(f"Failed to start ZMQ listener: {e}")

# Keyboard and mouse handlers
def keyboard(key, x, y):
    global keys_pressed, running
    
    if key == b'q' or key == b'Q':
        running = False
        glutLeaveMainLoop()
    elif key == b'+' or key == b'=':
        with voxel_grid.mutex:
            voxel_grid.threshold *= 0.8
            voxel_grid.display_list_valid = False
            print(f"Threshold: {voxel_grid.threshold:.3f}")
    elif key == b'-' or key == b'_':
        with voxel_grid.mutex:
            voxel_grid.threshold *= 1.25
            voxel_grid.display_list_valid = False
            print(f"Threshold: {voxel_grid.threshold:.3f}")
    elif key == b'r' or key == b'R':
        camera.reset()
        print("Camera reset")
    elif key == b't' or key == b'T':
        tracker.show_trails = not tracker.show_trails
        print(f"Tracking trails: {'ON' if tracker.show_trails else 'OFF'}")
    else:
        keys_pressed.add(key)

def keyboard_up(key, x, y):
    global keys_pressed
    if key in keys_pressed:
        keys_pressed.remove(key)

def mouse_motion(x, y):
    global camera
    
    if camera.first_mouse:
        camera.last_x = x
        camera.last_y = y
        camera.first_mouse = False
        return
    
    # Calculate offset
    x_offset = x - camera.last_x
    y_offset = camera.last_y - y  # Reversed since y-coordinates go from bottom to top
    
    camera.last_x = x
    camera.last_y = y
    
    # Apply sensitivity
    x_offset *= camera.mouse_sensitivity
    y_offset *= camera.mouse_sensitivity
    
    # Update camera angles
    camera.yaw += x_offset
    camera.pitch += y_offset
    
    # Clamp pitch to avoid flipping
    if camera.pitch > 89.0:
        camera.pitch = 89.0
    if camera.pitch < -89.0:
        camera.pitch = -89.0

def reshape(w, h):
    global width, height
    width, height = w, h
    
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, width / height, 0.1, 1000.0)
    glMatrixMode(GL_MODELVIEW)

# Process keyboard input and move camera
def process_input():
    speed = camera.speed
    
    # Get forward direction
    yaw_rad = np.radians(camera.yaw)
    pitch_rad = np.radians(camera.pitch)
    
    forward = np.array([
        np.cos(yaw_rad) * np.cos(pitch_rad),
        np.sin(pitch_rad),
        np.sin(yaw_rad) * np.cos(pitch_rad)
    ])
    forward = forward / np.linalg.norm(forward)
    
    # Get right direction
    right = np.cross(forward, np.array([0.0, 1.0, 0.0]))
    right = right / np.linalg.norm(right)
    
    # Get up direction
    up = np.cross(right, forward)
    
    # Process keys
    if b'w' in keys_pressed:
        camera.pos += forward * speed
    if b's' in keys_pressed:
        camera.pos -= forward * speed
    if b'a' in keys_pressed:
        camera.pos -= right * speed
    if b'd' in keys_pressed:
        camera.pos += right * speed
    if b' ' in keys_pressed:  # Space for up
        camera.pos += up * speed
    if b'c' in keys_pressed:  # 'c' for down
        camera.pos -= up * speed

# Draw coordinate axes and reference grid
def draw_reference():
    # Draw axes
    glLineWidth(3.0)
    
    # X axis (red)
    glColor3f(1.0, 0.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(10.0, 0.0, 0.0)
    glEnd()
    
    # Y axis (green)
    glColor3f(0.0, 1.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 10.0, 0.0)
    glEnd()
    
    # Z axis (blue)
    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 10.0)
    glEnd()
    
    # Draw ground grid
    glLineWidth(1.0)
    glColor3f(0.3, 0.3, 0.3)
    
    grid_size = 100
    step = 5
    
    glBegin(GL_LINES)
    for i in range(-grid_size, grid_size + 1, step):
        glVertex3f(i, 0.0, -grid_size)
        glVertex3f(i, 0.0, grid_size)
        glVertex3f(-grid_size, 0.0, i)
        glVertex3f(grid_size, 0.0, i)
    glEnd()

# OpenGL display function
def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    
    # Process keyboard input
    process_input()
    
    # Update view based on camera
    camera.update_view()
    
    # Draw reference grid and axes
    draw_reference()
    
    # Draw voxel grid
    with voxel_grid.mutex:
        if voxel_grid.grid is not None:
            if not voxel_grid.display_list_valid:
                voxel_grid.create_display_list()
            
            if voxel_grid.display_list_valid:
                # Enable blending for transparency
                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
                
                # Draw the voxel grid
                glCallList(voxel_grid.display_list)
                
                # Draw detections and trails
                voxel_grid.draw_detections(tracker)
                tracker.draw_trails()
                
                # Update tracker
                active_ids = [det['id'] for det in voxel_grid.detections]
                tracker.clear_old_tracks(active_ids)
                
                # Disable blending
                glDisable(GL_BLEND)
    
    glutSwapBuffers()

def idle():
    glutPostRedisplay()

# Main function
def main():
    global running
    
    # Parse command line arguments
    threshold = 0.3
    zmq_port = 5556
    
    if len(sys.argv) >= 2:
        try:
            threshold = float(sys.argv[1])
        except ValueError:
            print(f"Invalid threshold: {sys.argv[1]}, using default 0.3")
    
    if len(sys.argv) >= 3:
        try:
            zmq_port = int(sys.argv[2])
        except ValueError:
            print(f"Invalid port: {sys.argv[2]}, using default 5556")
    
    voxel_grid.threshold = threshold
    
    # Initialize GLUT
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(width, height)
    glutCreateWindow("Orthanc Labs - 3D Drone Detection Visualizer")
    
    # Set callbacks
    glutDisplayFunc(display)
    glutReshapeFunc(reshape)
    glutKeyboardFunc(keyboard)
    glutKeyboardUpFunc(keyboard_up)
    glutPassiveMotionFunc(mouse_motion)
    glutIdleFunc(idle)
    
    # Set OpenGL state
    glClearColor(0.05, 0.05, 0.1, 1.0)  # Dark blue background
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_POINT_SMOOTH)
    
    # Start ZeroMQ listener thread
    listener = Thread(target=zmq_listener, args=(zmq_port,), daemon=True)
    listener.start()
    
    print("Orthanc Labs - 3D Drone Detection Visualizer")
    print("=" * 50)
    print("Controls:")
    print("  Mouse: Look around")
    print("  W/A/S/D: Move camera")
    print("  Space/C: Move up/down")
    print("  +/-: Adjust detection threshold")
    print("  R: Reset camera position")
    print("  T: Toggle tracking trails")
    print("  Q: Quit")
    print(f"Settings:")
    print(f"  Detection threshold: {threshold}")
    print(f"  ZMQ port: {zmq_port}")
    print("=" * 50)
    
    # Start main loop
    glutMainLoop()
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 