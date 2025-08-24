#!/usr/bin/env python3
"""
Static Voxel Grid Viewer for Orthanc Labs Drone Detection System

This script loads and analyzes voxel grid data files, providing static
visualization with PyVista for detailed analysis and high-quality screenshots.

Usage:
    python static_voxel_viewer.py <voxel_grid.bin> [options]
    
Options:
    --threshold FLOAT     Brightness threshold (default: 0.3)
    --percentile FLOAT    Top percentile to show (default: 99.0)
    --output DIR          Screenshot output directory (default: screenshots)
    --interactive         Show interactive window
    --rotation X Y Z      Apply rotation in degrees (default: 0 0 0)
    --center X Y Z        Grid center coordinates (default: auto-detect)

Requirements:
    pip install pyvista numpy
"""

import os
import sys
import argparse
import struct
import numpy as np
import pyvista as pv
from pathlib import Path
import re
import math

def load_voxel_grid(filename):
    """
    Load voxel grid from Orthanc Labs binary format.
    
    Format:
    - 3 x int32: grid_size [X, Y, Z]
    - 1 x float32: voxel_size
    - 3 x float32: grid_origin [X, Y, Z]
    - N x float32: voxel data (N = X*Y*Z)
    
    Returns:
        tuple: (voxel_grid, voxel_size, grid_origin)
    """
    with open(filename, "rb") as f:
        # Read header
        header_data = f.read(32)  # 3*4 + 4 + 3*4 = 32 bytes
        if len(header_data) < 32:
            raise ValueError("Invalid file format: insufficient header data")
        
        # Unpack header
        grid_size = struct.unpack('iii', header_data[:12])
        voxel_size = struct.unpack('f', header_data[12:16])[0]
        grid_origin = struct.unpack('fff', header_data[16:32])
        
        # Read voxel data
        total_voxels = grid_size[0] * grid_size[1] * grid_size[2]
        voxel_bytes = f.read(total_voxels * 4)
        
        if len(voxel_bytes) < total_voxels * 4:
            raise ValueError("Invalid file format: insufficient voxel data")
        
        voxel_data = np.frombuffer(voxel_bytes, dtype=np.float32)
        voxel_grid = voxel_data.reshape(grid_size)
        
        return voxel_grid, voxel_size, grid_origin

def extract_significant_voxels(voxel_grid, voxel_size, grid_origin, 
                             threshold=None, percentile=99.0):
    """
    Extract voxels above threshold or percentile.
    
    Returns:
        tuple: (points, intensities) where points is Nx3 array
    """
    if threshold is None:
        flat_vals = voxel_grid.ravel()
        threshold = np.percentile(flat_vals[flat_vals > 0], percentile)
    
    # Find voxels above threshold
    coords = np.argwhere(voxel_grid > threshold)
    if coords.size == 0:
        return None, None, threshold
    
    intensities = voxel_grid[coords[:, 0], coords[:, 1], coords[:, 2]]
    
    # Convert to world coordinates
    x_world = grid_origin[0] + (coords[:, 0] + 0.5) * voxel_size
    y_world = grid_origin[1] + (coords[:, 1] + 0.5) * voxel_size
    z_world = grid_origin[2] + (coords[:, 2] + 0.5) * voxel_size
    
    points = np.column_stack((x_world, y_world, z_world))
    return points, intensities, threshold

def apply_rotation(points, rx_deg, ry_deg, rz_deg):
    """Apply Euler rotation to points (X->Y->Z order)."""
    rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
    
    # Rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])
    
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation: Rz * Ry * Rx
    R = Rz @ Ry @ Rx
    return points @ R.T

def cluster_analysis(points, intensities, eps=2.0, min_samples=5):
    """
    Perform DBSCAN clustering on voxel points.
    
    Returns:
        dict: Analysis results with cluster information
    """
    try:
        from sklearn.cluster import DBSCAN
    except ImportError:
        print("Warning: scikit-learn not available, skipping cluster analysis")
        return None
    
    # Perform clustering
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = clustering.labels_
    
    # Analyze clusters
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise = list(labels).count(-1)
    
    clusters = {}
    for i in range(n_clusters):
        mask = labels == i
        cluster_points = points[mask]
        cluster_intensities = intensities[mask]
        
        # Calculate cluster properties
        centroid = np.mean(cluster_points, axis=0)
        total_intensity = np.sum(cluster_intensities)
        max_intensity = np.max(cluster_intensities)
        size = len(cluster_points)
        
        # Calculate bounding box
        bbox_min = np.min(cluster_points, axis=0)
        bbox_max = np.max(cluster_points, axis=0)
        bbox_size = bbox_max - bbox_min
        
        clusters[i] = {
            'centroid': centroid,
            'size': size,
            'total_intensity': total_intensity,
            'max_intensity': max_intensity,
            'bbox_min': bbox_min,
            'bbox_max': bbox_max,
            'bbox_size': bbox_size,
            'points': cluster_points,
            'intensities': cluster_intensities
        }
    
    return {
        'n_clusters': n_clusters,
        'n_noise': n_noise,
        'labels': labels,
        'clusters': clusters
    }

def get_next_screenshot_index(folder, prefix="voxel_", suffix=".png"):
    """Find the next available screenshot index."""
    if not os.path.exists(folder):
        return 1
    
    pattern = re.compile(rf"^{prefix}(\d+){suffix}$")
    max_index = 0
    for fname in os.listdir(folder):
        match = pattern.match(fname)
        if match:
            idx = int(match.group(1))
            if idx > max_index:
                max_index = idx
    return max_index + 1

def create_visualization(points, intensities, analysis=None, 
                        show_clusters=True, show_interactive=False):
    """Create PyVista visualization."""
    # Create plotter
    plotter = pv.Plotter(off_screen=not show_interactive)
    plotter.set_background("white")
    plotter.enable_terrain_style()
    
    if points is None or len(points) == 0:
        print("No data to visualize")
        return plotter
    
    # Main point cloud
    cloud = pv.PolyData(points)
    cloud["intensity"] = intensities
    
    # Add main point cloud
    plotter.add_points(
        cloud,
        scalars="intensity",
        cmap="hot",
        point_size=8.0,
        render_points_as_spheres=True,
        opacity=0.7,
        show_scalar_bar=True
    )
    
    # Add cluster analysis if available
    if analysis and show_clusters:
        colors = ['red', 'green', 'blue', 'yellow', 'cyan', 'magenta', 'orange', 'purple']
        
        for cluster_id, cluster in analysis['clusters'].items():
            color = colors[cluster_id % len(colors)]
            
            # Add cluster points
            cluster_cloud = pv.PolyData(cluster['points'])
            plotter.add_points(
                cluster_cloud,
                color=color,
                point_size=10.0,
                render_points_as_spheres=True,
                opacity=0.9
            )
            
            # Add cluster centroid
            centroid_cloud = pv.PolyData([cluster['centroid']])
            plotter.add_points(
                centroid_cloud,
                color='black',
                point_size=15.0,
                render_points_as_spheres=True
            )
            
            # Add bounding box
            bbox = pv.Box(
                bounds=[
                    cluster['bbox_min'][0], cluster['bbox_max'][0],
                    cluster['bbox_min'][1], cluster['bbox_max'][1],
                    cluster['bbox_min'][2], cluster['bbox_max'][2]
                ]
            )
            plotter.add_mesh(bbox, style='wireframe', color=color, line_width=3)
            
            # Add text label
            plotter.add_point_labels(
                [cluster['centroid']],
                [f"Cluster {cluster_id}\nSize: {cluster['size']}\nIntensity: {cluster['total_intensity']:.1f}"],
                point_size=0,
                font_size=12,
                text_color='black'
            )
    
    # Add coordinate axes
    plotter.add_axes(
        xlabel='X (m)',
        ylabel='Y (m)',
        zlabel='Z (m)',
        line_width=3,
        labels_off=False
    )
    
    return plotter

def print_analysis_summary(analysis, threshold):
    """Print analysis summary to console."""
    if not analysis:
        print("No cluster analysis available")
        return
    
    print("\n" + "="*60)
    print("VOXEL GRID ANALYSIS SUMMARY")
    print("="*60)
    print(f"Detection threshold: {threshold:.3f}")
    print(f"Number of clusters found: {analysis['n_clusters']}")
    print(f"Number of noise points: {analysis['n_noise']}")
    print()
    
    if analysis['n_clusters'] > 0:
        print("CLUSTER DETAILS:")
        print("-" * 40)
        for cluster_id, cluster in analysis['clusters'].items():
            print(f"Cluster {cluster_id}:")
            print(f"  Position: ({cluster['centroid'][0]:.2f}, {cluster['centroid'][1]:.2f}, {cluster['centroid'][2]:.2f})")
            print(f"  Size: {cluster['size']} voxels")
            print(f"  Total intensity: {cluster['total_intensity']:.2f}")
            print(f"  Max intensity: {cluster['max_intensity']:.2f}")
            print(f"  Bounding box: {cluster['bbox_size'][0]:.2f} x {cluster['bbox_size'][1]:.2f} x {cluster['bbox_size'][2]:.2f} m")
            print()
    
    print("="*60)

def main():
    parser = argparse.ArgumentParser(description="Static Voxel Grid Viewer for Orthanc Labs")
    parser.add_argument("input_file", help="Voxel grid binary file")
    parser.add_argument("--threshold", type=float, help="Brightness threshold")
    parser.add_argument("--percentile", type=float, default=99.0, help="Top percentile to show")
    parser.add_argument("--output", default="screenshots", help="Screenshot output directory")
    parser.add_argument("--interactive", action="store_true", help="Show interactive window")
    parser.add_argument("--rotation", nargs=3, type=float, default=[0, 0, 0], 
                      metavar=('X', 'Y', 'Z'), help="Rotation in degrees")
    parser.add_argument("--center", nargs=3, type=float, metavar=('X', 'Y', 'Z'), 
                      help="Grid center coordinates")
    parser.add_argument("--no-clusters", action="store_true", help="Disable cluster analysis")
    parser.add_argument("--cluster-eps", type=float, default=2.0, help="DBSCAN epsilon parameter")
    parser.add_argument("--cluster-min-samples", type=int, default=5, help="DBSCAN min samples")
    
    args = parser.parse_args()
    
    if not os.path.isfile(args.input_file):
        print(f"Error: File '{args.input_file}' not found")
        return 1
    
    try:
        # Load voxel grid
        print(f"Loading voxel grid from: {args.input_file}")
        voxel_grid, voxel_size, grid_origin = load_voxel_grid(args.input_file)
        print(f"Grid size: {voxel_grid.shape}")
        print(f"Voxel size: {voxel_size:.3f} m")
        print(f"Grid origin: ({grid_origin[0]:.2f}, {grid_origin[1]:.2f}, {grid_origin[2]:.2f})")
        print(f"Max voxel value: {voxel_grid.max():.3f}")
        
        # Override grid center if specified
        if args.center:
            grid_origin = args.center
        
        # Extract significant voxels
        points, intensities, threshold = extract_significant_voxels(
            voxel_grid, voxel_size, grid_origin, 
            threshold=args.threshold, percentile=args.percentile
        )
        
        if points is None:
            print("No significant voxels found to visualize")
            return 1
        
        print(f"Found {len(points)} significant voxels (threshold: {threshold:.3f})")
        
        # Apply rotation if specified
        if any(r != 0 for r in args.rotation):
            print(f"Applying rotation: {args.rotation}")
            points = apply_rotation(points, *args.rotation)
        
        # Perform cluster analysis
        analysis = None
        if not args.no_clusters:
            print("Performing cluster analysis...")
            analysis = cluster_analysis(points, intensities, 
                                      eps=args.cluster_eps, 
                                      min_samples=args.cluster_min_samples)
            if analysis:
                print_analysis_summary(analysis, threshold)
        
        # Create visualization
        print("Creating visualization...")
        plotter = create_visualization(points, intensities, analysis, 
                                     show_clusters=not args.no_clusters,
                                     show_interactive=args.interactive)
        
        # Save screenshot
        if not args.interactive:
            output_dir = Path(args.output)
            output_dir.mkdir(exist_ok=True)
            
            next_idx = get_next_screenshot_index(output_dir)
            screenshot_path = output_dir / f"voxel_{next_idx:04d}.png"
            
            print(f"Saving screenshot to: {screenshot_path}")
            plotter.show(window_size=[1920, 1080], screenshot=str(screenshot_path))
            print("Screenshot saved successfully")
        else:
            # Show interactive window
            print("Opening interactive window...")
            print("Controls:")
            print("  - Left mouse: Rotate")
            print("  - Right mouse: Pan")
            print("  - Scroll: Zoom")
            print("  - 'r': Reset view")
            print("  - 'q': Quit")
            plotter.show(window_size=[1400, 900])
        
        return 0
        
    except Exception as e:
        print(f"Error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 