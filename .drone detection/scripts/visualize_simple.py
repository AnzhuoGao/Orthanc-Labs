#!/usr/bin/env python3
"""
Simple Cross-Platform Voxel Grid Visualizer for Orthanc Labs

This script provides a lightweight visualization solution that works on macOS
without requiring X11 or complex OpenGL setup. It uses matplotlib for 3D plotting.
Now includes velocity and acceleration vector visualization for tracked drones.

Usage:
    python visualize_simple.py [threshold] [demo_mode]
    
    - threshold: Optional brightness threshold for visualization (default: 0.3)
    - demo_mode: Run with animated demo data (default: true if no ZMQ data)

Controls:
    - Rotate: Click and drag
    - Zoom: Scroll wheel or +/- keys
    - Reset: 'r' key
    - Quit: 'q' key or close window
    - 'v': Toggle velocity vectors
    - 'a': Toggle acceleration vectors
"""

import sys
import os
import time
import json
import numpy as np
import struct
import zmq
import zlib
import logging
from threading import Thread, Lock
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Optional, List, Tuple
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, FancyBboxPatch
from matplotlib.gridspec import GridSpec
from datetime import datetime

try:
    from matplotlib.widgets import Slider
    HAS_WIDGETS = True
except ImportError:
    HAS_WIDGETS = False

@dataclass
class DroneState:
    """Represents a drone's state at a specific time"""
    position: np.ndarray  # [x, y, z]
    timestamp: float
    intensity: float
    
@dataclass
class DroneVectors:
    """Calculated velocity and acceleration vectors for a drone"""
    velocity: np.ndarray  # [vx, vy, vz] in m/s
    acceleration: np.ndarray  # [ax, ay, az] in m/s²
    position: np.ndarray  # Current position [x, y, z]
    confidence: float  # 0-1, confidence in the calculation

class DroneTracker:
    """Tracks drone positions over time to calculate velocity and acceleration"""
    
    def __init__(self, max_history=10, dt=0.1):
        self.max_history = max_history  # Maximum number of historical states to keep
        self.dt = dt  # Time step between measurements
        self.drone_histories = defaultdict(lambda: deque(maxlen=max_history))
        self.next_drone_id = 0
        
    def update_drones(self, drone_positions: List[Tuple[np.ndarray, float]], current_time: float) -> List[DroneVectors]:
        """
        Update drone positions and calculate velocity/acceleration vectors
        
        Args:
            drone_positions: List of (position, intensity) tuples
            current_time: Current timestamp
            
        Returns:
            List of DroneVectors with calculated velocity and acceleration
        """
        # Simple nearest-neighbor tracking (for demo purposes)
        drone_vectors = []
        
        # Update existing drones or create new ones
        used_positions = set()
        
        for drone_id, history in list(self.drone_histories.items()):
            if len(history) == 0:
                continue
                
            # Find closest position to last known position
            last_pos = history[-1].position
            best_match = None
            best_distance = float('inf')
            best_idx = -1
            
            for i, (pos, intensity) in enumerate(drone_positions):
                if i in used_positions:
                    continue
                distance = np.linalg.norm(pos - last_pos)
                if distance < best_distance and distance < 10.0:  # Max tracking distance
                    best_match = (pos, intensity)
                    best_distance = distance
                    best_idx = i
            
            if best_match is not None:
                # Update existing drone
                used_positions.add(best_idx)
                new_state = DroneState(best_match[0], current_time, best_match[1])
                history.append(new_state)
                
                # Calculate vectors
                vectors = self._calculate_vectors(history)
                if vectors is not None:
                    drone_vectors.append(vectors)
        
        # Create new drones for unmatched positions
        for i, (pos, intensity) in enumerate(drone_positions):
            if i not in used_positions:
                drone_id = self.next_drone_id
                self.next_drone_id += 1
                
                new_state = DroneState(pos, current_time, intensity)
                self.drone_histories[drone_id].append(new_state)
        
        return drone_vectors
    
    def _calculate_vectors(self, history: deque) -> Optional[DroneVectors]:
        """Calculate velocity and acceleration from position history"""
        if len(history) < 2:
            return None
            
        current_state = history[-1]
        
        # Calculate velocity (simple finite difference)
        if len(history) >= 2:
            prev_state = history[-2]
            dt = current_state.timestamp - prev_state.timestamp
            if dt > 0:
                velocity = (current_state.position - prev_state.position) / dt
            else:
                velocity = np.zeros(3)
        else:
            velocity = np.zeros(3)
        
        # Calculate acceleration (requires at least 3 points)
        if len(history) >= 3:
            prev2_state = history[-3]
            prev_state = history[-2]
            
            dt1 = prev_state.timestamp - prev2_state.timestamp
            dt2 = current_state.timestamp - prev_state.timestamp
            
            if dt1 > 0 and dt2 > 0:
                # Calculate velocity at two time points
                v1 = (prev_state.position - prev2_state.position) / dt1
                v2 = (current_state.position - prev_state.position) / dt2
                
                # Acceleration is change in velocity over time
                acceleration = (v2 - v1) / ((dt1 + dt2) / 2)
            else:
                acceleration = np.zeros(3)
        else:
            acceleration = np.zeros(3)
        
        # Calculate confidence based on history length and measurement stability
        confidence = min(1.0, len(history) / 5.0)  # Full confidence with 5+ measurements
        
        # Reduce confidence if acceleration is unrealistically high
        accel_magnitude = np.linalg.norm(acceleration)
        if accel_magnitude > 20.0:  # m/s² threshold
            confidence *= 0.5
        
        return DroneVectors(
            velocity=velocity,
            acceleration=acceleration,
            position=current_state.position.copy(),
            confidence=confidence
        )

class SimpleDemoData:
    """Optimized demo data generator for testing with realistic trajectories"""
    
    def __init__(self):
        self.grid_size = [50, 50, 25]
        self.voxel_size = 1.0
        self.grid_origin = [-25.0, -25.0, 0.0]
        self.time_offset = 0
        self.cached_noise = None
        self.noise_cache_lifetime = 0
        
        # Pre-calculate distance matrices for performance
        self._object_cache = {}
        
        # Enhanced trajectory parameters for more realistic motion
        self.drone_params = [
            {'type': 'circular', 'radius': 15, 'speed': 0.8, 'z_oscillation': 3},
            {'type': 'figure8', 'radius': 12, 'speed': 1.2, 'z_oscillation': 2},
            {'type': 'hovering', 'drift_radius': 3, 'speed': 0.3, 'z_oscillation': 1}
        ]
        
    def generate_frame(self):
        """Generate a frame of demo data with realistic drone trajectories"""
        self.time_offset += 0.1
        t = self.time_offset
        
        # Create empty grid
        grid = np.zeros(self.grid_size, dtype=np.float32)
        
        # Store drone positions for tracking
        self.current_drone_positions = []
        
        # Generate drones with realistic trajectories
        for i, params in enumerate(self.drone_params):
            position = self._calculate_drone_position(t, params, i)
            x, y, z = position
            
            # Convert to grid coordinates
            grid_x = int(x + 25)  # Offset for grid centering
            grid_y = int(y + 25)
            grid_z = int(z)
            
            intensity = 0.9 - i * 0.1  # Decreasing intensity
            size = 2 if i == 0 else 1
            
            self._add_object_optimized(grid, grid_x, grid_y, grid_z, intensity=intensity, size=size)
            
            # Store world position for tracking
            world_pos = np.array([x, y, z])
            self.current_drone_positions.append((world_pos, intensity))
        

        
        # Add cached noise occasionally
        if self.cached_noise is None or self.noise_cache_lifetime > 20:
            self.cached_noise = self._generate_background_noise()
            self.noise_cache_lifetime = 0
        
        grid += self.cached_noise
        self.noise_cache_lifetime += 1
        
        # Clamp grid values to valid intensity range [0, 1]
        grid = np.clip(grid, 0.0, 1.0)
        
        return grid
    
    def _calculate_drone_position(self, t: float, params: dict, drone_id: int) -> np.ndarray:
        """Calculate realistic drone position based on trajectory type"""
        offset = drone_id * 0.5  # Phase offset for each drone
        
        if params['type'] == 'circular':
            # Circular motion with varying speed
            angle = t * params['speed'] + offset
            x = params['radius'] * np.cos(angle)
            y = params['radius'] * np.sin(angle)
            z = 10 + params['z_oscillation'] * np.sin(t * 0.5 + offset)
            
        elif params['type'] == 'figure8':
            # Figure-8 motion
            angle = t * params['speed'] + offset
            x = params['radius'] * np.cos(angle)
            y = params['radius'] * np.sin(2 * angle) / 2
            z = 8 + params['z_oscillation'] * np.cos(t * 0.7 + offset)
            
        elif params['type'] == 'hovering':
            # Hovering with small random drift
            base_x = 15 + drone_id * 5
            base_y = 15 + drone_id * 3
            base_z = 12
            
            # Small oscillations around base position
            x = base_x + params['drift_radius'] * np.sin(t * params['speed'] + offset)
            y = base_y + params['drift_radius'] * np.cos(t * params['speed'] * 1.3 + offset)
            z = base_z + params['z_oscillation'] * np.sin(t * 0.3 + offset)
        
        else:
            # Default to simple motion
            x = 10 * np.cos(t + offset)
            y = 10 * np.sin(t + offset)
            z = 10
        
        return np.array([x, y, z])
    
    def get_current_drone_positions(self) -> List[Tuple[np.ndarray, float]]:
        """Get the current frame's drone positions for tracking"""
        return getattr(self, 'current_drone_positions', [])
    
    def _add_object_optimized(self, grid, cx, cy, cz, intensity=0.8, size=2):
        """Add a drone-like object to the grid (vectorized for performance)"""
        # Create coordinate ranges
        x_range = np.arange(max(0, cx - size), min(self.grid_size[0], cx + size + 1))
        y_range = np.arange(max(0, cy - size), min(self.grid_size[1], cy + size + 1))
        z_range = np.arange(max(0, cz - 1), min(self.grid_size[2], cz + 2))
        
        # Use meshgrid for vectorized operations
        if len(x_range) > 0 and len(y_range) > 0 and len(z_range) > 0:
            xx, yy, zz = np.meshgrid(x_range, y_range, z_range, indexing='ij')
            
            # Calculate distances vectorized
            dx = xx - cx
            dy = yy - cy  
            dz = zz - cz
            distances = np.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Calculate intensities
            obj_intensities = intensity * np.maximum(0.1, 1.0 - distances / (size + 1))
            
            # Apply to grid
            grid[xx, yy, zz] = np.maximum(grid[xx, yy, zz], obj_intensities)
    
    def _generate_background_noise(self):
        """Generate background noise (cached for performance)"""
        noise = np.random.random(self.grid_size).astype(np.float32) * 0.05
        # Only keep sparse noise
        noise[noise < 0.04] = 0
        return noise

class VoxelVisualizer:
    """Simple matplotlib-based voxel visualizer with drone tracking"""
    
    def __init__(self, threshold=0.3, demo_mode=True):
        self.threshold = max(0.0, min(1.0, threshold))  # Clamp initial threshold
        self.demo_mode = demo_mode
        self.demo_data = SimpleDemoData()
        self.drone_tracker = DroneTracker(max_history=8, dt=0.1)
        self.mutex = Lock()
        self.current_grid = None
        self.current_detections = []
        
        # Vector visualization settings
        self.show_velocity = True
        self.show_acceleration = True
        self.velocity_scale = 0.5  # Scale factor for velocity vectors (reduced)
        self.acceleration_scale = 0.2  # Scale factor for acceleration vectors (reduced)
        
        # Camera visualization settings
        self.show_cameras = True
        self.show_fov = False
        self.camera_config = self._load_camera_config()
        
        # Real-time camera config monitoring
        self.camera_config_file = os.path.join(os.path.dirname(__file__), "../config/camera_positions.json")
        self.last_config_mtime = 0
        self._update_config_mtime()
        
        # Current frame vectors for display
        self.current_vectors = []
        self._vector_update_counter = 0
        self._vector_update_interval = 3  # Update vectors every N frames for smoother display
        
        # Setup matplotlib with tactical command interface theme
        plt.ion()  # Interactive mode
        plt.style.use('dark_background')  # Enable dark background
        
        # Create figure with tactical command layout
        self.fig = plt.figure(figsize=(16, 10), facecolor='#0d1117')
        self.fig.patch.set_facecolor('#0d1117')
        
        # Create tactical grid layout
        self.gs = GridSpec(3, 4, figure=self.fig, 
                          left=0.02, right=0.98, top=0.95, bottom=0.08,
                          hspace=0.15, wspace=0.08,
                          width_ratios=[0.25, 1.5, 0.25, 0.25],
                          height_ratios=[0.15, 1, 0.2])
        
        # Main 3D visualization area (center)
        self.ax = self.fig.add_subplot(self.gs[1, 1], projection='3d', facecolor='#161b22')
        
        # Tactical panels
        self.setup_tactical_panels()
        
        # Apply tactical theme
        self._apply_tactical_theme()
        
        # Setup UI components
        self.setup_tactical_ui()
        
        # Data source
        self.zmq_thread = None
        self.running = True
        
    def setup_tactical_panels(self):
        """Setup tactical command interface panels"""
        # Status panel (top left)
        self.status_ax = self.fig.add_subplot(self.gs[0, 0])
        self.status_ax.set_facecolor('#21262d')
        self.status_ax.set_title('◤ SYSTEM STATUS ◥', fontsize=10, color='#58a6ff', 
                                fontweight='bold', fontfamily='monospace', pad=8)
        
        # Threat panel (left)
        self.threat_ax = self.fig.add_subplot(self.gs[1, 0])
        self.threat_ax.set_facecolor('#21262d')
        self.threat_ax.set_title('◤ THREAT DETECTION ◥', fontsize=10, color='#f85149', 
                                fontweight='bold', fontfamily='monospace', pad=8)
        
        # Intelligence panel (top right) 
        self.intel_ax = self.fig.add_subplot(self.gs[0, 2])
        self.intel_ax.set_facecolor('#21262d')
        self.intel_ax.set_title('◤ INTELLIGENCE ◥', fontsize=10, color='#a5a5a5', 
                               fontweight='bold', fontfamily='monospace', pad=8)
        
        # Operations panel (right)
        self.ops_ax = self.fig.add_subplot(self.gs[1, 2])
        self.ops_ax.set_facecolor('#21262d')
        self.ops_ax.set_title('◤ OPERATIONS ◥', fontsize=10, color='#56d364', 
                             fontweight='bold', fontfamily='monospace', pad=8)
        
        # Mission panel (top right corner)
        self.mission_ax = self.fig.add_subplot(self.gs[0, 3])
        self.mission_ax.set_facecolor('#21262d')
        self.mission_ax.set_title('◤ MISSION ◥', fontsize=10, color='#e3b341', 
                                 fontweight='bold', fontfamily='monospace', pad=8)
        
        # Alerts panel (bottom right)
        self.alerts_ax = self.fig.add_subplot(self.gs[1, 3])
        self.alerts_ax.set_facecolor('#21262d')
        self.alerts_ax.set_title('◤ ALERTS ◥', fontsize=10, color='#ff7b72', 
                                fontweight='bold', fontfamily='monospace', pad=8)
        
        # Remove axes for panels
        for ax in [self.status_ax, self.threat_ax, self.intel_ax, 
                   self.ops_ax, self.mission_ax, self.alerts_ax]:
            ax.set_xticks([])
            ax.set_yticks([])
            for spine in ax.spines.values():
                spine.set_color('#30363d')
                spine.set_linewidth(1)
    
    def setup_tactical_ui(self):
        """Setup the tactical UI components"""
        # Main visualization title
        self.ax.set_title("ORTHANC LABS - TACTICAL AIRSPACE MONITORING", 
                         fontsize=14, color='#58a6ff', fontweight='bold', 
                         fontfamily='monospace', pad=15)
        self.ax.set_xlabel("X COORDINATE (METERS)", fontsize=10, color='#a5a5a5', fontfamily='monospace')
        self.ax.set_ylabel("Y COORDINATE (METERS)", fontsize=10, color='#a5a5a5', fontfamily='monospace')
        self.ax.set_zlabel("ALTITUDE (METERS)", fontsize=10, color='#a5a5a5', fontfamily='monospace')
        
        # Set equal aspect ratio
        self.ax.set_box_aspect([1,1,0.5])
        
        # Add tactical control banner
        control_text = "◤ TACTICAL CONTROLS: ROTATE=DRAG • ZOOM=SCROLL • RESET=R • VECTORS=V/A • CAMERAS=C/G • QUIT=Q ◥"
        self.fig.text(0.5, 0.02, control_text, fontsize=9, ha='center', 
                     color='#7d8590', fontweight='bold', fontfamily='monospace')
        
        # Connect events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_click)
        
        # Populate tactical panels
        self._populate_tactical_panels()
        
        # Add threshold control in bottom area
        if HAS_WIDGETS:
            # Position slider in bottom area
            ax_thresh = plt.axes([0.25, 0.04, 0.5, 0.02], facecolor='#21262d')
            self.thresh_slider = Slider(ax_thresh, 'SIGNAL THRESHOLD', 0.0, 1.0, 
                                      valinit=min(1.0, max(0.0, self.threshold)), valfmt='%.2f',
                                      color='#58a6ff', alpha=0.8)
            
            # Style slider
            for spine in ax_thresh.spines.values():
                spine.set_color('#30363d')
            ax_thresh.tick_params(colors='#7d8590', labelsize=8)
            
            self.thresh_slider.on_changed(self.update_threshold)
    
    def _populate_tactical_panels(self):
        """Populate tactical panels with operational information"""
        # System Status Panel
        status_text = [
            "╔══ SYSTEM STATUS ══╗",
            "├ RADAR: ONLINE",
            "├ CAMERAS: 3/3 ACTIVE", 
            "├ DETECTION: ACTIVE",
            "├ TRACKING: ENABLED",
            "└ STATUS: OPERATIONAL"
        ]
        
        # Clear and add status text
        self.status_ax.clear()
        self.status_ax.set_facecolor('#21262d')
        for spine in self.status_ax.spines.values():
            spine.set_color('#30363d')
        self.status_ax.set_xticks([])
        self.status_ax.set_yticks([])
        
        for i, text in enumerate(status_text):
            color = '#58a6ff' if i == 0 else '#56d364'
            self.status_ax.text(0.05, 0.9 - i*0.12, text, fontsize=8, color=color,
                               fontfamily='monospace', fontweight='bold', 
                               transform=self.status_ax.transAxes)
        
        # Threat Detection Panel  
        threat_text = [
            "╔═ THREAT ANALYSIS ═╗",
            "├ ACTIVE TRACKS: 3",
            "├ THREAT LEVEL: LOW",
            "├ CONFIDENCE: 94%",
            "├ LAST DETECT: 0.1s",
            "└ VECTOR CALC: ON"
        ]
        
        self.threat_ax.clear()
        self.threat_ax.set_facecolor('#21262d')
        for spine in self.threat_ax.spines.values():
            spine.set_color('#30363d')
        self.threat_ax.set_xticks([])
        self.threat_ax.set_yticks([])
        
        for i, text in enumerate(threat_text):
            color = '#f85149' if i == 0 else '#ff7b72'
            self.threat_ax.text(0.05, 0.9 - i*0.12, text, fontsize=8, color=color,
                               fontfamily='monospace', fontweight='bold',
                               transform=self.threat_ax.transAxes)
        
        # Intelligence Panel
        intel_text = [
            "╔═══ INTEL ═══╗",
            f"├ TIME: {datetime.now().strftime('%H:%M:%S')}",
            "├ AREA: SECURED",
            "├ WEATHER: CLEAR",
            "└ VISIBILITY: GOOD"
        ]
        
        self.intel_ax.clear()
        self.intel_ax.set_facecolor('#21262d')
        for spine in self.intel_ax.spines.values():
            spine.set_color('#30363d')
        self.intel_ax.set_xticks([])
        self.intel_ax.set_yticks([])
        
        for i, text in enumerate(intel_text):
            color = '#a5a5a5' if i == 0 else '#7d8590'
            self.intel_ax.text(0.05, 0.8 - i*0.15, text, fontsize=8, color=color,
                               fontfamily='monospace', fontweight='bold',
                               transform=self.intel_ax.transAxes)
        
        # Operations Panel
        ops_text = [
            "╔═ OPERATIONS ═╗",
            "├ MODE: ACTIVE", 
            "├ SENSORS: ALL",
            "├ COVERAGE: 100%",
            "├ RESPONSE: READY",
            "└ ENGAGEMENT: AUTO"
        ]
        
        self.ops_ax.clear()
        self.ops_ax.set_facecolor('#21262d')
        for spine in self.ops_ax.spines.values():
            spine.set_color('#30363d')
        self.ops_ax.set_xticks([])
        self.ops_ax.set_yticks([])
        
        for i, text in enumerate(ops_text):
            color = '#56d364' if i == 0 else '#39d353'
            self.ops_ax.text(0.05, 0.9 - i*0.12, text, fontsize=8, color=color,
                               fontfamily='monospace', fontweight='bold',
                               transform=self.ops_ax.transAxes)
        
        # Mission Panel
        mission_text = [
            "╔═ MISSION ═╗",
            "├ AIRSPACE GUARD",
            "├ PRIORITY: HIGH",
            "└ STATUS: ACTIVE"
        ]
        
        self.mission_ax.clear()
        self.mission_ax.set_facecolor('#21262d')
        for spine in self.mission_ax.spines.values():
            spine.set_color('#30363d')
        self.mission_ax.set_xticks([])
        self.mission_ax.set_yticks([])
        
        for i, text in enumerate(mission_text):
            color = '#e3b341' if i == 0 else '#d29922'
            self.mission_ax.text(0.05, 0.8 - i*0.2, text, fontsize=8, color=color,
                               fontfamily='monospace', fontweight='bold',
                               transform=self.mission_ax.transAxes)
        
        # Alerts Panel
        alerts_text = [
            "╔═══ ALERTS ═══╗",
            "├ NO ACTIVE ALERTS",
            "├ LAST: NONE",
            "└ SYSTEM: GREEN"
        ]
        
        self.alerts_ax.clear()
        self.alerts_ax.set_facecolor('#21262d') 
        for spine in self.alerts_ax.spines.values():
            spine.set_color('#30363d')
        self.alerts_ax.set_xticks([])
        self.alerts_ax.set_yticks([])
        
        for i, text in enumerate(alerts_text):
            color = '#ff7b72' if i == 0 else '#56d364'  # Green for no alerts
            self.alerts_ax.text(0.05, 0.8 - i*0.2, text, fontsize=8, color=color,
                               fontfamily='monospace', fontweight='bold',
                               transform=self.alerts_ax.transAxes)
    
    def update_threshold(self, val):
        """Update visualization threshold"""
        # Clamp threshold between 0 and 1
        val = max(0.0, min(1.0, val))
        
        # Only print if threshold change is significant (reduce spam)
        if abs(val - self.threshold) > 0.01:
            self.threshold = val
            print(f"Threshold updated to: {val:.3f}")
        else:
            self.threshold = val
    
    def on_key_press(self, event):
        """Handle keyboard events"""
        if event.key == 'q':
            self.running = False
            plt.close('all')
        elif event.key == 'r':
            self.reset_view()
        elif event.key == '+' or event.key == '=':
            self.threshold = max(0.0, min(1.0, self.threshold * 0.8))
            print(f"Threshold: {self.threshold:.3f}")
            if HAS_WIDGETS:
                self.thresh_slider.set_val(self.threshold)
        elif event.key == '-':
            self.threshold = max(0.0, min(1.0, self.threshold * 1.25))
            print(f"Threshold: {self.threshold:.3f}")
            if HAS_WIDGETS:
                self.thresh_slider.set_val(self.threshold)
        elif event.key == 'v':
            self.show_velocity = not self.show_velocity
            print(f"Velocity vectors: {'ON' if self.show_velocity else 'OFF'}")
            self._force_vector_update()
        elif event.key == 'a':
            self.show_acceleration = not self.show_acceleration
            print(f"Acceleration vectors: {'ON' if self.show_acceleration else 'OFF'}")
            self._force_vector_update()
        elif event.key == 'c':
            self.show_cameras = not self.show_cameras
            print(f"Camera positions: {'ON' if self.show_cameras else 'OFF'}")
            self._force_camera_update()
        elif event.key == 'g':
            self.show_fov = not self.show_fov
            print(f"Camera FOV: {'ON' if self.show_fov else 'OFF'}")
            self._force_camera_update()
    
    def on_scroll(self, event):
        """Handle mouse scroll for zooming"""
        if event.inaxes != self.ax:
            return
        
        # Get current limits
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        zlim = self.ax.get_zlim()
        
        # Calculate zoom factor
        zoom_factor = 1.1 if event.step < 0 else 1/1.1
        
        # Calculate center points
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        z_center = (zlim[0] + zlim[1]) / 2
        
        # Calculate new ranges
        x_range = (xlim[1] - xlim[0]) * zoom_factor / 2
        y_range = (ylim[1] - ylim[0]) * zoom_factor / 2
        z_range = (zlim[1] - zlim[0]) * zoom_factor / 2
        
        # Set new limits
        self.ax.set_xlim(x_center - x_range, x_center + x_range)
        self.ax.set_ylim(y_center - y_range, y_center + y_range)
        self.ax.set_zlim(z_center - z_range, z_center + z_range)
        
        # Redraw
        self.fig.canvas.draw_idle()
    
    def on_mouse_click(self, event):
        """Handle mouse click events for camera interaction"""
        if event.inaxes != self.ax or not self.show_cameras:
            return
        
        # Only handle left mouse button clicks
        if event.button != 1:
            return
            
        # Get click coordinates in 3D space
        if event.xdata is None or event.ydata is None:
            return
        
        # Check if click is near any camera
        clicked_camera = self._find_clicked_camera(event.xdata, event.ydata)
        if clicked_camera:
            self._show_camera_info(clicked_camera)
    
    def _find_clicked_camera(self, click_x, click_y):
        """Find if click is near any camera position"""
        if not hasattr(self, 'camera_config') or not self.camera_config:
            return None
        
        # Convert 3D camera positions to 2D screen coordinates
        click_tolerance = 2.0  # meters tolerance for clicking
        
        for camera_id, camera_info in self.camera_config.items():
            cam_x, cam_y, cam_z = camera_info['position']
            
            # Calculate distance from click to camera position (2D projection)
            distance = ((click_x - cam_x)**2 + (click_y - cam_y)**2)**0.5
            
            if distance <= click_tolerance:
                return camera_id
        
        return None
    
    def _show_camera_info(self, camera_id):
        """Display camera information in a popup-style text box"""
        if not hasattr(self, 'camera_config') or camera_id not in self.camera_config:
            return
        
        camera_info = self.camera_config[camera_id]
        pos = camera_info['position']
        direction = camera_info['direction']
        
        # Calculate orientation angles from direction vector
        import math
        azimuth = math.degrees(math.atan2(direction[1], direction[0]))
        elevation = math.degrees(math.atan2(direction[2], 
                                          math.sqrt(direction[0]**2 + direction[1]**2)))
        
        # Create info text
        info_text = f"""╔═══ CAMERA {camera_id.upper()} INFO ═══╗
├ POSITION: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})m
├ AZIMUTH: {azimuth:.1f}°
├ ELEVATION: {elevation:.1f}°
├ TYPE: {camera_info.get('type', 'TACTICAL').upper()}
├ FOV: {camera_info.get('fov_degrees', 60)}°
├ RANGE: {camera_info.get('max_range', 100)}m
└ STATUS: ACTIVE"""
        
        # Remove previous camera info if it exists
        if hasattr(self, '_camera_info_text'):
            self._camera_info_text.remove()
        
        # Display info text box near camera position
        self._camera_info_text = self.fig.text(0.78, 0.7, info_text, fontsize=8,
                                              fontfamily='monospace', fontweight='bold',
                                              color='#f0f6fc', ha='left', va='top',
                                              bbox=dict(boxstyle="round,pad=0.5",
                                                       facecolor='#21262d',
                                                       edgecolor='#58a6ff',
                                                       linewidth=2, alpha=0.95))
        
        print(f"Camera {camera_id.upper()} info displayed - Position: {pos}, Azimuth: {azimuth:.1f}°, Elevation: {elevation:.1f}°")
        
        # Auto-hide after 5 seconds
        def hide_info():
            if hasattr(self, '_camera_info_text'):
                self._camera_info_text.remove()
        
        # Use matplotlib's timer to auto-hide
        self.fig.canvas.mpl_connect('draw_event', lambda event: None)  # Ensure redraw
        timer = self.fig.canvas.new_timer(interval=5000)  # 5 seconds
        timer.single_shot = True
        timer.add_callback(hide_info)
        timer.start()
    
    def reset_view(self):
        """Reset the 3D view"""
        self.ax.view_init(elev=20, azim=45)
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        self.ax.set_zlim(0, 25)
        print("View reset")
    
    def start_zmq_listener(self, port=5556):
        """Start ZMQ listener in background thread"""
        self.zmq_thread = Thread(target=self.zmq_listener, args=(port,), daemon=True)
        self.zmq_thread.start()
    
    def zmq_listener(self, port):
        """Listen for ZMQ data"""
        try:
            ctx = zmq.Context.instance()
            sub = ctx.socket(zmq.SUB)
            sub.connect(f"tcp://127.0.0.1:{port}")
            sub.setsockopt_string(zmq.SUBSCRIBE, "")
            sub.setsockopt(zmq.RCVTIMEO, 100)
            
            print(f"Listening for ZMQ data on port {port}...")
            zmq_connected = False
            
            while self.running:
                try:
                    # Try to receive data
                    voxel_msg = sub.recv(flags=zmq.NOBLOCK)
                    self.parse_zmq_data(voxel_msg)
                    zmq_connected = True
                    self.demo_mode = False
                    
                except zmq.Again:
                    if not zmq_connected:
                        # No data, use demo if enabled
                        if self.demo_mode:
                            with self.mutex:
                                self.current_grid = self.demo_data.generate_frame()
                    time.sleep(0.1)
                except Exception as e:
                    print(f"ZMQ error: {e}")
                    time.sleep(1.0)
                    
        except Exception as e:
            print(f"ZMQ not available: {e}")
            print("Running in demo mode only")
    
    def parse_zmq_data(self, data):
        """Parse ZMQ voxel data"""
        try:
            if len(data) >= 20:
                header = struct.unpack('iiifff', data[:20])
                grid_size = [header[0], header[1], header[2]]
                voxel_size = header[3]
                grid_origin = [header[4], header[5], header[6]]
                
                grid_data_size = grid_size[0] * grid_size[1] * grid_size[2]
                grid_bytes = data[20:20 + grid_data_size * 4]
                
                if len(grid_bytes) >= grid_data_size * 4:
                    grid_data = np.frombuffer(grid_bytes, dtype=np.float32)
                    grid = grid_data.reshape(grid_size)
                    
                    with self.mutex:
                        self.current_grid = grid
                        
        except Exception as e:
            print(f"Error parsing ZMQ data: {e}")
    
    def extract_points_from_grid(self, grid):
        """Extract significant points from voxel grid (optimized)"""
        if grid is None:
            return None, None, None, None
        
        # Apply threshold with optimized operations
        significant_mask = grid > self.threshold
        
        # Early exit if no significant points
        if not np.any(significant_mask):
            return None, None, None, None
        
        coords = np.argwhere(significant_mask)
        
        # Limit points for performance (keep only strongest signals)
        max_points = 800  # Reduced from unlimited to prevent slowdown
        if len(coords) > max_points:
            intensities_all = grid[coords[:, 0], coords[:, 1], coords[:, 2]]
            # Keep only the strongest points
            top_indices = np.argpartition(intensities_all, -max_points)[-max_points:]
            coords = coords[top_indices]
        
        intensities = grid[coords[:, 0], coords[:, 1], coords[:, 2]]
        
        # Vectorized coordinate transformation
        coords_float = coords.astype(np.float32)  # More efficient than repeated indexing
        x_world = self.demo_data.grid_origin[0] + coords_float[:, 0] * self.demo_data.voxel_size
        y_world = self.demo_data.grid_origin[1] + coords_float[:, 1] * self.demo_data.voxel_size
        z_world = self.demo_data.grid_origin[2] + coords_float[:, 2] * self.demo_data.voxel_size
        
        return x_world, y_world, z_world, intensities
    
    def update_plot(self):
        """Update the 3D plot"""
        with self.mutex:
            current_grid = self.current_grid
        
        if self.demo_mode and current_grid is None:
            current_grid = self.demo_data.generate_frame()
        
        # Update drone tracking for demo mode (regardless of grid state)
        if self.demo_mode:
            if hasattr(self.demo_data, 'current_drone_positions'):
                current_time = time.time()
                drone_positions = self.demo_data.get_current_drone_positions()
                if len(drone_positions) > 0:  # Only track if we have positions
                    with self.mutex:
                        self.current_vectors = self.drone_tracker.update_drones(drone_positions, current_time)
        
        # Extract points
        result = self.extract_points_from_grid(current_grid)
        if result[0] is None:
            x, y, z, intensities = None, None, None, None
        else:
            x, y, z, intensities = result
        
        # Only clear if we have data or if this is the first frame
        if not hasattr(self, '_initialized'):
            self.ax.clear()
            self._initialized = True
        elif x is not None and len(x) > 0:
            # Only clear the scatter plot, keep axes
            if hasattr(self, '_scatter'):
                self._scatter.remove()
        
        if x is not None and len(x) > 0:
            # Create scatter plot with modern dark theme
            self._scatter = self.ax.scatter(x, y, z, c=intensities, cmap='plasma', 
                                          s=15, alpha=0.8, vmin=0, vmax=1, 
                                          edgecolors='none', linewidths=0,
                                          rasterized=True)
            
            # Add modern colorbar only once
            if not hasattr(self, 'colorbar'):
                self.colorbar = plt.colorbar(self._scatter, ax=self.ax, shrink=0.6, pad=0.1)
                self.colorbar.set_label('SIGNAL INTENSITY', fontsize=9, 
                                      color=self.colors['text_secondary'], 
                                      fontweight='bold', fontfamily='monospace')
                self.colorbar.ax.tick_params(colors=self.colors['text_secondary'], labelsize=8)
                self.colorbar.ax.set_facecolor(self.colors['bg_secondary'])
        
        # Draw velocity and acceleration vectors
        self._draw_drone_vectors()
        
        # Draw camera positions and FOV
        self._draw_cameras()
        
        # Configure axes only on first run
        if not hasattr(self, '_axes_configured'):
            self._configure_axes()
            self._axes_configured = True
        
        # Update status text efficiently
        if hasattr(self, '_status_text'):
            self._status_text.remove()
        
        mode_text = "◢ DEMO MODE ◣" if self.demo_mode else "◢ LIVE FEED ◣"
        point_count = len(x) if x is not None else 0
        vector_count = len(getattr(self, 'current_vectors', []))
        vel_status = "ACTIVE" if self.show_velocity else "STANDBY"
        accel_status = "ACTIVE" if self.show_acceleration else "STANDBY"
        cam_status = "ACTIVE" if self.show_cameras else "STANDBY"
        fov_status = "ACTIVE" if self.show_fov else "STANDBY"
        
        status = f"{mode_text} • TARGETS: {point_count} • TRACKED: {vector_count} • VEL:{vel_status} • ACC:{accel_status} • CAM:{cam_status} • FOV:{fov_status}"
        
        self._status_text = self.fig.text(0.02, 0.96, status, fontsize=9, 
                                        fontweight='bold', fontfamily='monospace',
                                        color=self.colors['text_primary'],
                                        bbox=dict(boxstyle="round,pad=0.3", 
                                                facecolor=self.colors['bg_secondary'], 
                                                edgecolor=self.colors['accent_primary'],
                                                linewidth=1, alpha=0.9))
        
        # Update tactical panels periodically
        if not hasattr(self, '_panel_update_counter'):
            self._panel_update_counter = 0
        self._panel_update_counter += 1
        
        # Update panels every 30 frames (about 1 second)
        if self._panel_update_counter % 30 == 0:
            self._update_tactical_data(point_count, vector_count)
            
        # Check for camera configuration updates every 60 frames (about 2 seconds)
        if self._panel_update_counter % 60 == 0:
            self._check_camera_config_updates()
    
    def _update_tactical_data(self, point_count, vector_count):
        """Update tactical panels with live data"""
        current_time = datetime.now()
        
        # Update Threat Detection Panel with live data
        threat_level = "LOW" if point_count < 5 else ("MEDIUM" if point_count < 10 else "HIGH")
        confidence = max(85, min(99, 85 + (point_count * 2)))
        
        threat_text = [
            "╔═ THREAT ANALYSIS ═╗",
            f"├ ACTIVE TRACKS: {vector_count}",
            f"├ THREAT LEVEL: {threat_level}",
            f"├ CONFIDENCE: {confidence}%",
            "├ LAST DETECT: 0.1s",
            f"└ VECTOR CALC: {'ON' if self.show_velocity or self.show_acceleration else 'OFF'}"
        ]
        
        self.threat_ax.clear()
        self.threat_ax.set_facecolor('#21262d')
        for spine in self.threat_ax.spines.values():
            spine.set_color('#30363d')
        self.threat_ax.set_xticks([])
        self.threat_ax.set_yticks([])
        
        for i, text in enumerate(threat_text):
            if i == 0:
                color = '#f85149'
            elif "THREAT LEVEL" in text:
                color = '#f85149' if threat_level == "HIGH" else ('#e3b341' if threat_level == "MEDIUM" else '#56d364')
            else:
                color = '#ff7b72'
            self.threat_ax.text(0.05, 0.9 - i*0.12, text, fontsize=8, color=color,
                               fontfamily='monospace', fontweight='bold',
                               transform=self.threat_ax.transAxes)
        
        # Update Intelligence Panel with live time
        intel_text = [
            "╔═══ INTEL ═══╗",
            f"├ TIME: {current_time.strftime('%H:%M:%S')}",
            "├ AREA: SECURED",
            "├ WEATHER: CLEAR",
            "└ VISIBILITY: GOOD"
        ]
        
        self.intel_ax.clear()
        self.intel_ax.set_facecolor('#21262d')
        for spine in self.intel_ax.spines.values():
            spine.set_color('#30363d')
        self.intel_ax.set_xticks([])
        self.intel_ax.set_yticks([])
        
        for i, text in enumerate(intel_text):
            color = '#a5a5a5' if i == 0 else '#7d8590'
            self.intel_ax.text(0.05, 0.8 - i*0.15, text, fontsize=8, color=color,
                               fontfamily='monospace', fontweight='bold',
                               transform=self.intel_ax.transAxes)
        
        # Update Status Panel with current state
        camera_count = len(self.camera_config) if hasattr(self, 'camera_config') and self.camera_config else 3
        status_text = [
            "╔══ SYSTEM STATUS ══╗",
            "├ RADAR: ONLINE",
            f"├ CAMERAS: {camera_count}/{camera_count} ACTIVE", 
            "├ DETECTION: ACTIVE",
            f"├ TRACKING: {'ENABLED' if vector_count > 0 else 'STANDBY'}",
            "└ STATUS: OPERATIONAL"
        ]
        
        self.status_ax.clear()
        self.status_ax.set_facecolor('#21262d')
        for spine in self.status_ax.spines.values():
            spine.set_color('#30363d')
        self.status_ax.set_xticks([])
        self.status_ax.set_yticks([])
        
        for i, text in enumerate(status_text):
            color = '#58a6ff' if i == 0 else '#56d364'
            self.status_ax.text(0.05, 0.9 - i*0.12, text, fontsize=8, color=color,
                               fontfamily='monospace', fontweight='bold', 
                               transform=self.status_ax.transAxes)
        
        # Update Alerts Panel based on activity
        if vector_count > 5:
            alert_status = "HIGH ACTIVITY"
            alert_color = '#e3b341'
        elif vector_count > 0:
            alert_status = "TRACKING ACTIVE"
            alert_color = '#56d364'
        else:
            alert_status = "NO ACTIVE ALERTS"
            alert_color = '#56d364'
            
        alerts_text = [
            "╔═══ ALERTS ═══╗",
            f"├ {alert_status}",
            f"├ LAST: {current_time.strftime('%H:%M')}",
            "└ SYSTEM: GREEN"
        ]
        
        self.alerts_ax.clear()
        self.alerts_ax.set_facecolor('#21262d') 
        for spine in self.alerts_ax.spines.values():
            spine.set_color('#30363d')
        self.alerts_ax.set_xticks([])
        self.alerts_ax.set_yticks([])
        
        for i, text in enumerate(alerts_text):
            color = '#ff7b72' if i == 0 else alert_color
            self.alerts_ax.text(0.05, 0.8 - i*0.2, text, fontsize=8, color=color,
                               fontfamily='monospace', fontweight='bold',
                               transform=self.alerts_ax.transAxes)
    
    def _apply_tactical_theme(self):
        """Apply modern dark military/aerospace theme"""
        # Tactical command interface colors (inspired by reference)
        self.colors = {
            'bg_primary': '#0d1117',      # GitHub dark background
            'bg_secondary': '#21262d',    # Panel backgrounds
            'accent_primary': '#58a6ff',   # Blue accent (tactical)
            'accent_secondary': '#f85149', # Red alerts
            'text_primary': '#f0f6fc',    # High contrast white text
            'text_secondary': '#7d8590',  # Dimmed text
            'grid_color': '#30363d',      # Subtle grid lines
            'axis_color': '#484f58',      # Axis lines
            'highlight': '#39c5cf',       # Cyan highlights
            'warning': '#e3b341',         # Amber warnings
            'success': '#56d364',         # Green success
            'border': '#30363d',          # Border color
        }
        
        # Set figure background
        self.fig.patch.set_facecolor(self.colors['bg_primary'])
        self.ax.set_facecolor(self.colors['bg_primary'])
        
        # Remove default matplotlib styling
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        
        # Set transparent pane backgrounds
        self.ax.xaxis.pane.set_alpha(0.0)
        self.ax.yaxis.pane.set_alpha(0.0)
        self.ax.zaxis.pane.set_alpha(0.0)
        
        # Configure pane edges
        self.ax.xaxis.pane.set_edgecolor(self.colors['grid_color'])
        self.ax.yaxis.pane.set_edgecolor(self.colors['grid_color'])
        self.ax.zaxis.pane.set_edgecolor(self.colors['grid_color'])
        self.ax.xaxis.pane.set_linewidth(0.3)
        self.ax.yaxis.pane.set_linewidth(0.3)
        self.ax.zaxis.pane.set_linewidth(0.3)

    def _configure_axes(self):
        """Configure axes styling and properties (called once)"""
        # Modern title styling
        self.ax.set_title("ORTHANC LABS  •  AUTONOMOUS THREAT DETECTION", 
                         fontsize=14, fontweight='bold', color=self.colors['text_primary'],
                         pad=20, fontfamily='monospace')
        
        # Axis labels with modern styling
        self.ax.set_xlabel("X COORDINATE (m)", fontsize=10, color=self.colors['text_secondary'], 
                          fontweight='bold', fontfamily='monospace')
        self.ax.set_ylabel("Y COORDINATE (m)", fontsize=10, color=self.colors['text_secondary'],
                          fontweight='bold', fontfamily='monospace')
        self.ax.set_zlabel("ALTITUDE (m)", fontsize=10, color=self.colors['text_secondary'],
                          fontweight='bold', fontfamily='monospace')
        
        # Center axes at origin (0,0,0) with symmetric range
        range_x = 30
        range_y = 30
        range_z = 25
        self.ax.set_xlim(-range_x, range_x)
        self.ax.set_ylim(-range_y, range_y)
        self.ax.set_zlim(0, range_z)
        
        # Modern grid styling - disable z-axis grid, keep x and y
        self.ax.grid(True, alpha=0.2, linewidth=0.4, color=self.colors['grid_color'])
        
        # Remove z-axis grid lines specifically
        self.ax.zaxis.set_pane_color((0, 0, 0, 0))  # Transparent z-pane
        self.ax.zaxis._axinfo['grid']['color'] = (0, 0, 0, 0)  # Transparent z-grid
        
        # Axis line styling
        self.ax.xaxis.line.set_color(self.colors['axis_color'])
        self.ax.yaxis.line.set_color(self.colors['axis_color'])
        self.ax.zaxis.line.set_color(self.colors['axis_color'])
        self.ax.xaxis.line.set_linewidth(0.6)
        self.ax.yaxis.line.set_linewidth(0.6)
        self.ax.zaxis.line.set_linewidth(0.6)
        
        # Tick styling
        self.ax.tick_params(axis='x', colors=self.colors['text_secondary'], labelsize=8, 
                           width=0.4, length=2, which='major')
        self.ax.tick_params(axis='y', colors=self.colors['text_secondary'], labelsize=8,
                           width=0.4, length=2, which='major')
        self.ax.tick_params(axis='z', colors=self.colors['text_secondary'], labelsize=8,
                           width=0.4, length=2, which='major')
        
        # Add origin markers
        self._add_origin_markers()
        
        # Set viewing angle for optimal perspective
        self.ax.view_init(elev=20, azim=45)

    def _add_origin_markers(self):
        """Add subtle origin coordinate markers"""
        # Draw thin lines through origin
        origin_alpha = 0.3
        origin_width = 0.8
        
        # X-axis line through origin
        self.ax.plot3D([-30, 30], [0, 0], [0, 0], 
                      color=self.colors['accent_primary'], alpha=origin_alpha, 
                      linewidth=origin_width, linestyle='-')
        
        # Y-axis line through origin  
        self.ax.plot3D([0, 0], [-30, 30], [0, 0],
                      color=self.colors['accent_primary'], alpha=origin_alpha,
                      linewidth=origin_width, linestyle='-')
        
        # Z-axis line through origin
        self.ax.plot3D([0, 0], [0, 0], [0, 25],
                      color=self.colors['accent_primary'], alpha=origin_alpha,
                      linewidth=origin_width, linestyle='-')
        
        # Origin point marker
        self.ax.scatter([0], [0], [0], color=self.colors['highlight'], 
                       s=30, alpha=0.8, marker='o', edgecolors='white', linewidth=0.5)
    
    def _draw_drone_vectors(self):
        """Draw velocity and acceleration vectors for tracked drones (smooth updates)"""
        if not hasattr(self, 'current_vectors'):
            return
            
        # Initialize vector storage if needed
        if not hasattr(self, '_velocity_arrows'):
            self._velocity_arrows = []
        if not hasattr(self, '_acceleration_arrows'):
            self._acceleration_arrows = []
        if not hasattr(self, '_last_vector_count'):
            self._last_vector_count = 0
        if not hasattr(self, '_legend_created'):
            self._legend_created = False
        
        # Get current vectors safely
        with self.mutex:
            vectors = self.current_vectors.copy()
        
        # Filter valid vectors
        valid_vectors = []
        for drone_vector in vectors:
            pos = drone_vector.position
            vel = drone_vector.velocity
            accel = drone_vector.acceleration
            confidence = drone_vector.confidence
            
            # More lenient thresholds for demo
            if confidence >= 0.2:  # Require some confidence
                valid_vectors.append(drone_vector)
        
        # Update counter
        self._vector_update_counter += 1
        
        # Only update vectors if count changed significantly, visibility toggled, or at regular intervals
        vector_count = len(valid_vectors)
        should_update = (
            abs(vector_count - self._last_vector_count) > 0 or  # Vector count changed
            not hasattr(self, '_last_show_velocity') or self._last_show_velocity != self.show_velocity or
            not hasattr(self, '_last_show_acceleration') or self._last_show_acceleration != self.show_acceleration or
            self._vector_update_counter % self._vector_update_interval == 0  # Regular updates
        )
        
        if should_update:
            # Clear previous arrows only when updating
            for arrow in self._velocity_arrows:
                try:
                    arrow.remove()
                except:
                    pass
            for arrow in self._acceleration_arrows:
                try:
                    arrow.remove()
                except:
                    pass
            
            self._velocity_arrows = []
            self._acceleration_arrows = []
            
            # Draw new vectors
            for drone_vector in valid_vectors:
                pos = drone_vector.position
                vel = drone_vector.velocity
                accel = drone_vector.acceleration
                
                # Draw velocity vector (cyber green)
                if self.show_velocity and np.linalg.norm(vel) > 0.1:
                    vel_scaled = vel * self.velocity_scale
                    arrow = self.ax.quiver(pos[0], pos[1], pos[2],
                                         vel_scaled[0], vel_scaled[1], vel_scaled[2],
                                         color=self.colors['accent_primary'], alpha=0.9, linewidth=2.5,
                                         arrow_length_ratio=0.15)
                    self._velocity_arrows.append(arrow)
                
                # Draw acceleration vector (alert red)
                if self.show_acceleration and np.linalg.norm(accel) > 0.5:
                    accel_scaled = accel * self.acceleration_scale
                    arrow = self.ax.quiver(pos[0], pos[1], pos[2],
                                         accel_scaled[0], accel_scaled[1], accel_scaled[2],
                                         color=self.colors['accent_secondary'], alpha=0.8, linewidth=2,
                                         arrow_length_ratio=0.2)
                    self._acceleration_arrows.append(arrow)
            
            # Update legend only when vectors change
            self._update_vector_legend()
            
            # Track state for next frame
            self._last_vector_count = vector_count
            self._last_show_velocity = self.show_velocity
            self._last_show_acceleration = self.show_acceleration
    
    def _update_vector_legend(self):
        """Update legend for vectors with modern styling"""
        if (self.show_velocity or self.show_acceleration):
            if len(self._velocity_arrows) > 0 or len(self._acceleration_arrows) > 0:
                legend_elements = []
                if self.show_velocity and len(self._velocity_arrows) > 0:
                    legend_elements.append(plt.Line2D([0], [0], color=self.colors['accent_primary'], 
                                                    lw=2.5, label='VELOCITY VECTOR'))
                if self.show_acceleration and len(self._acceleration_arrows) > 0:
                    legend_elements.append(plt.Line2D([0], [0], color=self.colors['accent_secondary'], 
                                                    lw=2, label='ACCELERATION VECTOR'))
                
                if legend_elements:
                    legend = self.ax.legend(handles=legend_elements, loc='upper left', 
                                          fontsize=9, frameon=True, 
                                          facecolor=self.colors['bg_secondary'], 
                                          edgecolor=self.colors['grid_color'],
                                          framealpha=0.9)
                    legend.get_frame().set_linewidth(0.5)
                    
                    # Style legend text
                    for text in legend.get_texts():
                        text.set_color(self.colors['text_primary'])
                        text.set_fontfamily('monospace')
                        text.set_fontweight('bold')
                    
                    self._legend_created = True
            else:
                # Remove legend if no vectors
                if hasattr(self.ax, 'legend_') and self.ax.legend_:
                    self.ax.legend_.remove()
                self._legend_created = False
        else:
            # Remove legend if vectors are disabled
            if hasattr(self.ax, 'legend_') and self.ax.legend_:
                self.ax.legend_.remove()
            self._legend_created = False
    
    def _force_vector_update(self):
        """Force vectors to update on next frame (for immediate response to toggles)"""
        if hasattr(self, '_last_show_velocity'):
            del self._last_show_velocity
        if hasattr(self, '_last_show_acceleration'):
            del self._last_show_acceleration
    
    def _force_camera_update(self):
        """Force camera visualization to update on next frame"""
        if hasattr(self, '_last_show_cameras'):
            del self._last_show_cameras
        if hasattr(self, '_last_show_fov'):
            del self._last_show_fov
    
    def _load_camera_config(self):
        """Load camera configuration from real-time file or defaults"""
        # Try to load from real-time configuration file first
        realtime_config_file = os.path.join(os.path.dirname(__file__), "../config/camera_positions.json")
        
        if os.path.exists(realtime_config_file):
            try:
                with open(realtime_config_file, 'r') as f:
                    realtime_cameras = json.load(f)
                
                logger.info(f"Loaded {len(realtime_cameras)} cameras from real-time config")
                return realtime_cameras
                
            except Exception as e:
                logger.warning(f"Failed to load real-time camera config: {e}")
        
        # Fall back to default tactical camera configuration
        default_cameras = [
            {
                'id': 'CAM_NORTH',
                'position': [0, 25, 8],      # North perimeter
                'orientation': [0, -1, -0.3], # Looking south and down
                'fov_h': 60,                 # Horizontal FOV in degrees
                'fov_v': 45,                 # Vertical FOV in degrees
                'range': 40,                 # Maximum effective range
                'type': 'perimeter'
            },
            {
                'id': 'CAM_SOUTH',
                'position': [0, -25, 8],     # South perimeter
                'orientation': [0, 1, -0.3], # Looking north and down
                'fov_h': 60,
                'fov_v': 45,
                'range': 40,
                'type': 'perimeter'
            },
            {
                'id': 'CAM_EAST',
                'position': [25, 0, 8],      # East perimeter
                'orientation': [-1, 0, -0.3], # Looking west and down
                'fov_h': 60,
                'fov_v': 45,
                'range': 40,
                'type': 'perimeter'
            },
            {
                'id': 'CAM_WEST',
                'position': [-25, 0, 8],     # West perimeter
                'orientation': [1, 0, -0.3], # Looking east and down
                'fov_h': 60,
                'fov_v': 45,
                'range': 40,
                'type': 'perimeter'
            },
            {
                'id': 'CAM_CENTER',
                'position': [0, 0, 15],      # Central elevated position
                'orientation': [0, 0, -1],   # Looking straight down
                'fov_h': 90,
                'fov_v': 90,
                'range': 30,
                'type': 'overview'
            },
            {
                'id': 'CAM_NE',
                'position': [15, 15, 10],    # Northeast corner
                'orientation': [-0.7, -0.7, -0.4], # Looking southwest and down
                'fov_h': 50,
                'fov_v': 40,
                'range': 35,
                'type': 'corner'
            }
        ]
        
        # Convert list to dictionary for easier access
        camera_dict = {}
        for camera in default_cameras:
            camera_dict[camera['id']] = {
                'position': camera['position'],
                'direction': camera['orientation'],  # Rename for consistency
                'fov_degrees': camera['fov_h'],
                'type': camera['type'],
                'max_range': camera['range']
            }
        
        return camera_dict
    
    def _update_config_mtime(self):
        """Update the last modification time of the camera config file"""
        try:
            if os.path.exists(self.camera_config_file):
                self.last_config_mtime = os.path.getmtime(self.camera_config_file)
        except:
            self.last_config_mtime = 0
    
    def _check_camera_config_updates(self):
        """Check if camera config file has been updated and reload if necessary"""
        try:
            if not os.path.exists(self.camera_config_file):
                return False
            
            current_mtime = os.path.getmtime(self.camera_config_file)
            
            if current_mtime > self.last_config_mtime:
                # File has been updated, reload configuration
                new_config = self._load_camera_config()
                if new_config:
                    self.camera_config = new_config
                    self.last_config_mtime = current_mtime
                    
                    # Force camera visualization update
                    self._force_camera_update()
                    
                    print(f"Camera configuration reloaded: {len(self.camera_config)} cameras")
                    return True
                    
        except Exception as e:
            print(f"Error checking camera config updates: {e}")
        
        return False
    
    def _draw_cameras(self):
        """Draw camera positions and FOV visualization"""
        if not self.show_cameras and not self.show_fov:
            return
        
        # Initialize camera visualization storage
        if not hasattr(self, '_camera_markers'):
            self._camera_markers = []
        if not hasattr(self, '_fov_lines'):
            self._fov_lines = []
        if not hasattr(self, '_last_show_cameras'):
            self._last_show_cameras = False
        if not hasattr(self, '_last_show_fov'):
            self._last_show_fov = False
        
        # Check if update is needed
        should_update = (
            self._last_show_cameras != self.show_cameras or
            self._last_show_fov != self.show_fov
        )
        
        if should_update:
            # Clear previous camera visualization
            for marker in self._camera_markers:
                try:
                    marker.remove()
                except:
                    pass
            for line in self._fov_lines:
                try:
                    line.remove()
                except:
                    pass
            
            self._camera_markers = []
            self._fov_lines = []
            
            # Draw cameras if enabled
            if self.show_cameras:
                self._draw_camera_positions()
            
            # Draw FOV if enabled
            if self.show_fov:
                self._draw_camera_fov()
            
            # Update state tracking
            self._last_show_cameras = self.show_cameras
            self._last_show_fov = self.show_fov
    
    def _draw_camera_positions(self):
        """Draw camera position markers"""
        for camera_id, camera in self.camera_config.items():
            pos = camera['position']
            cam_type = camera.get('type', 'standard')
            
            # Choose color and marker based on camera type
            if cam_type == 'perimeter':
                color = self.colors['warning']  # Orange for perimeter
                marker = '^'
                size = 80
            elif cam_type == 'overview':
                color = self.colors['highlight']  # Cyan for overview
                marker = 's'
                size = 120
            elif cam_type == 'corner':
                color = self.colors['accent_primary']  # Green for corner
                marker = 'D'
                size = 60
            else:
                color = self.colors['text_primary']
                marker = 'o'
                size = 60
            
            # Draw camera marker
            marker_obj = self.ax.scatter(pos[0], pos[1], pos[2], 
                                       color=color, s=size, alpha=0.9,
                                       marker=marker, edgecolors='white', 
                                       linewidth=1.5)
            self._camera_markers.append(marker_obj)
            
            # Add camera label
            label = self.ax.text(pos[0], pos[1], pos[2] + 1, camera_id,
                               color=self.colors['text_primary'], fontsize=8,
                               fontweight='bold', fontfamily='monospace',
                               ha='center', va='bottom')
            self._camera_markers.append(label)
    
    def _draw_camera_fov(self):
        """Draw camera field of view cones"""
        for camera_id, camera in self.camera_config.items():
            pos = np.array(camera['position'])
            orientation = np.array(camera['direction'])  # Updated key name
            fov_h = np.radians(camera.get('fov_degrees', 60))  # Updated key name
            fov_v = np.radians(camera.get('fov_degrees', 60) * 0.75)  # Approximate vertical FOV
            cam_range = camera['max_range']  # Updated key name
            cam_type = camera.get('type', 'standard')
            
            # Choose FOV color based on camera type
            if cam_type == 'perimeter':
                fov_color = self.colors['warning']
                alpha = 0.15
            elif cam_type == 'overview':
                fov_color = self.colors['highlight']
                alpha = 0.2
            else:
                fov_color = self.colors['accent_primary']
                alpha = 0.1
            
            # Normalize orientation vector
            orientation = orientation / np.linalg.norm(orientation)
            
            # Create FOV cone
            self._draw_fov_cone(pos, orientation, fov_h, fov_v, cam_range, fov_color, alpha)
    
    def _draw_fov_cone(self, pos, direction, fov_h, fov_v, cam_range, color, alpha):
        """Draw a single camera's FOV cone"""
        # Create local coordinate system
        # Forward vector is the direction
        forward = direction
        
        # Create perpendicular vectors for horizontal and vertical FOV
        if abs(forward[2]) < 0.9:
            right = np.cross(forward, [0, 0, 1])
        else:
            right = np.cross(forward, [1, 0, 0])
        right = right / np.linalg.norm(right)
        
        up = np.cross(right, forward)
        up = up / np.linalg.norm(up)
        
        # Calculate FOV corner points at maximum range
        half_h = np.tan(fov_h / 2) * cam_range
        half_v = np.tan(fov_v / 2) * cam_range
        
        # Four corners of the FOV at max range
        end_center = pos + forward * cam_range
        corner1 = end_center + right * half_h + up * half_v      # Top right
        corner2 = end_center - right * half_h + up * half_v      # Top left
        corner3 = end_center - right * half_h - up * half_v      # Bottom left
        corner4 = end_center + right * half_h - up * half_v      # Bottom right
        
        # Draw FOV edges from camera to corners
        for corner in [corner1, corner2, corner3, corner4]:
            line = self.ax.plot3D([pos[0], corner[0]], [pos[1], corner[1]], [pos[2], corner[2]],
                                color=color, alpha=alpha * 2, linewidth=1)
            self._fov_lines.extend(line)
        
        # Draw FOV boundary at max range
        corners = [corner1, corner2, corner3, corner4, corner1]  # Close the loop
        for i in range(len(corners) - 1):
            line = self.ax.plot3D([corners[i][0], corners[i+1][0]], 
                                [corners[i][1], corners[i+1][1]], 
                                [corners[i][2], corners[i+1][2]],
                                color=color, alpha=alpha * 1.5, linewidth=1)
            self._fov_lines.extend(line)
    
    def run(self):
        """Main visualization loop"""
        print("Starting Orthanc Labs Simple Visualizer...")
        print("=" * 50)
        print("Controls:")
        print("  Rotate: Click and drag")
        print("  Zoom: Scroll wheel") 
        print("  Reset view: 'r'")
        print("  Adjust threshold: +/-")
        print("  Toggle velocity vectors: 'v'")
        print("  Toggle acceleration vectors: 'a'")
        print("  Toggle camera positions: 'c'")
        print("  Toggle camera FOV: 'g'")
        print("  Quit: 'q' or close window")
        print(f"Mode: {'Demo' if self.demo_mode else 'Live ZMQ'}")
        print("=" * 50)
        
        # Start ZMQ listener
        self.start_zmq_listener()
        
        # Initial view setup
        self.reset_view()
        
        # Animation loop with adaptive performance
        try:
            frame_count = 0
            last_data_hash = None
            skip_frames = 0
            
            while self.running:
                # Check if data actually changed (avoid unnecessary updates)
                current_data_hash = None
                if self.demo_mode:
                    current_data_hash = frame_count  # Always "changed" in demo
                else:
                    with self.mutex:
                        if self.current_grid is not None:
                            current_data_hash = hash(self.current_grid.data.tobytes())
                
                # Only update if data changed or in demo mode
                data_changed = (current_data_hash != last_data_hash)
                
                if data_changed or self.demo_mode:
                    self.update_plot()
                    last_data_hash = current_data_hash
                    skip_frames = 0
                else:
                    skip_frames += 1
                    
                # Adaptive frame rate based on activity
                if self.demo_mode:
                    frame_count += 1
                    if frame_count % 3 == 0:  # Update every 3rd frame for better performance
                        self.fig.canvas.draw_idle()
                        plt.pause(0.033)  # ~30 FPS cap
                    else:
                        plt.pause(0.033)
                elif data_changed:
                    self.fig.canvas.draw_idle()
                    plt.pause(0.1)  # Normal rate for live data changes
                else:
                    # No data change, sleep longer
                    plt.pause(0.2 if skip_frames < 10 else 0.5)
                
                # Check if window was closed
                if not plt.get_fignums():
                    break
                    
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.running = False
            plt.close('all')

def main():
    """Main function"""
    # Parse command line arguments
    threshold = 0.3
    demo_mode = True
    
    if len(sys.argv) >= 2:
        try:
            threshold = float(sys.argv[1])
        except ValueError:
            print(f"Invalid threshold: {sys.argv[1]}, using default 0.3")
    
    if len(sys.argv) >= 3:
        demo_mode = sys.argv[2].lower() in ('true', '1', 'yes', 'on')
    
    # Create and run visualizer
    visualizer = VoxelVisualizer(threshold=threshold, demo_mode=demo_mode)
    visualizer.run()
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 