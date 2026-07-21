import argparse
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

def read_pcd(filepath):
    # Try using open3d if available (faster and supports binary)
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(filepath)
        return np.asarray(pcd.points)
    except ImportError:
        pass
    
    # Fallback to manual parsing for ASCII PCD files
    points = []
    print("Open3D not found. Using fallback ASCII parser (this might be slower).")
    print("For better performance and binary support, consider installing open3d: pip install open3d")
    with open(filepath, 'r') as f:
        is_data = False
        for line in f:
            if is_data:
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                    except ValueError:
                        continue
            elif line.startswith('DATA ascii'):
                is_data = True
            elif line.startswith('DATA binary') or line.startswith('DATA binary_compressed'):
                print(f"Error: Open3D is required to read binary PCD file ({filepath}). Please run: pip install open3d")
                sys.exit(1)
    return np.array(points)

def main():
    parser = argparse.ArgumentParser(description="Plot multiple PCD files from bird's-eye view side-by-side with a shared colorbar.")
    parser.add_argument('pcd_files', nargs='+', type=str, help='Paths to the .pcd files (one or more)')
    parser.add_argument('--titles', nargs='*', type=str, default=[], help='Titles for each plot (separated by space). If titles contain spaces, enclose them in quotes.')
    parser.add_argument('--out', type=str, default='birdseye_comparison.png', help='Output image filename')
    parser.add_argument('--subsample', type=int, default=5, help='Subsample every Nth point to speed up plotting for large files')
    parser.add_argument('--point-size', type=float, default=0.01, help='Size of the points in the scatter plot')
    parser.add_argument('--show', action='store_true', help='Show the plot in a window (in addition to saving)')
    
    args = parser.parse_args()
    
    num_files = len(args.pcd_files)
    
    # Validate files exist
    for f in args.pcd_files:
        if not os.path.exists(f):
            print(f"Error: File not found: {f}")
            sys.exit(1)
            
    # Process titles
    titles = args.titles.copy()
    if len(titles) < num_files:
        # Pad with filenames if not enough titles provided
        for i in range(len(titles), num_files):
            titles.append(os.path.basename(args.pcd_files[i]))
            
    all_points = []
    global_z_min = float('inf')
    global_z_max = float('-inf')
    
    for idx, pcd_file in enumerate(args.pcd_files):
        print(f"Reading {pcd_file}...")
        points = read_pcd(pcd_file)
        
        if len(points) == 0:
            print(f"Warning: No points found in {pcd_file}.")
            all_points.append(np.empty((0, 3)))
            continue
            
        print(f"Loaded {len(points)} points from {pcd_file}.")
        
        # Subsample points if there are too many
        if args.subsample > 1 and len(points) > 100000:
            print(f"Subsampling points by a factor of {args.subsample}...")
            points = points[::args.subsample]
            print(f"Remaining points to plot for this file: {len(points)}")
            
        all_points.append(points)
        
        # Update global z limits for shared colorbar
        if points.shape[1] >= 3:
            z = points[:, 2]
            global_z_min = min(global_z_min, np.min(z))
            global_z_max = max(global_z_max, np.max(z))
            
    if global_z_min == float('inf'):
        # Fallback if no Z data
        global_z_min, global_z_max = 0.0, 1.0

    print("Generating plot...")
    
    # Create subplots side-by-side
    fig, axes = plt.subplots(1, num_files, figsize=(10 * num_files, 12), facecolor='white')
    
    # If only one file, axes is not an array, so wrap it
    if num_files == 1:
        axes = [axes]
        
    # Plot each point cloud
    scatter = None
    for i in range(num_files):
        ax = axes[i]
        points = all_points[i]
        
        if len(points) == 0:
            ax.set_title(titles[i], fontsize=16, fontweight='bold', pad=20)
            ax.axis('off')
            continue
            
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2] if points.shape[1] >= 3 else np.zeros_like(x)
        
        # Use very small point size and a colormap for Z-axis, with shared vmin and vmax
        scatter = ax.scatter(x, y, c=z, cmap='viridis', s=args.point_size, alpha=0.8, marker='.',
                             vmin=global_z_min, vmax=global_z_max)
        
        ax.set_title(titles[i], fontsize=16, fontweight='bold', pad=20)
        ax.axis('equal') # Keep aspect ratio equal to preserve geometry
        ax.axis('off')   # Hide axes for a clean white background look
        
    # Add a single shared colorbar to the right of all subplots if we plotted something
    if scatter is not None:
        cbar = fig.colorbar(scatter, ax=axes, fraction=0.046, pad=0.04)
        cbar.set_label('Höhe (Z-Achse)')
        
    print(f"Saving plot to {args.out}...")
    plt.savefig(args.out, dpi=300, bbox_inches='tight', facecolor='white')
    print("Done!")
    
    if args.show:
        plt.show()

if __name__ == '__main__':
    main()
