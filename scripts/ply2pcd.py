#!/usr/bin/env python3
"""Convert a PLY point cloud (from GLIM's offline_viewer export) to PCD and optionally generate a
topdown map image.


Usage: python3 ply2pcd.py <input.ply> <output.pcd> [voxel_size] [output.png]

Example:
    python3 ply2pcd.py \
        results/deliverables/map_run2.ply \
        results/deliverables/map_run2.pcd \
        0.05 \
        results/deliverables/map_run2_topdown.png
"""

import sys
import open3d as o3d
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.cm as cm


def save_topdown_images(pcd, output_prefix, resolution=0.05):
    """
    Generate:
      1. grayscale top-down occupancy PNG
      2. colored height top-down PNG
    """

    points = np.asarray(pcd.points)

    if len(points) == 0:
        print("No points available")
        return

    xy = points[:, :2]
    z = points[:, 2]

    min_xy = xy.min(axis=0)
    max_xy = xy.max(axis=0)

    size = np.ceil((max_xy - min_xy) / resolution).astype(int)

    print(f"Image size: {size[0]} x {size[1]} pixels")

    # Avoid huge images
    if size[0] > 12000 or size[1] > 12000:
        print("Large map detected, increasing resolution")
        resolution *= 2
        size = np.ceil((max_xy - min_xy) / resolution).astype(int)


    pixels = ((xy - min_xy) / resolution).astype(int)

    # image coordinate system
    pixels[:,1] = size[1] - pixels[:,1] - 1


    valid = (
        (pixels[:,0] >= 0) &
        (pixels[:,0] < size[0]) &
        (pixels[:,1] >= 0) &
        (pixels[:,1] < size[1])
    )

    pixels = pixels[valid]
    z = z[valid]


    ##################################
    # 1. GRAYSCALE OCCUPANCY MAP
    ##################################

    gray = np.zeros(
        (size[1], size[0]),
        dtype=np.uint8
    )

    gray[pixels[:,1], pixels[:,0]] = 255

    gray_path = output_prefix + "_gray.png"

    Image.fromarray(gray).save(gray_path)

    print(f"Wrote grayscale map: {gray_path}")


    ##################################
    # 2. COLOR HEIGHT MAP
    ##################################

    height = np.full(
        (size[1], size[0]),
        np.nan
    )

    height[pixels[:,1], pixels[:,0]] = z


    plt.figure(figsize=(10,10))

    plt.imshow(
        height,
        cmap="turbo"
    )

    plt.axis("off")

    plt.colorbar(
        label="Height (m)"
    )

    color_path = output_prefix + "_height.png"

    plt.savefig(
        color_path,
        bbox_inches="tight",
        dpi=300
    )

    plt.close()

    print(f"Wrote colored height map: {color_path}")


def main():

    if len(sys.argv) < 3:
        print(
            "Usage: python3 ply2pcd.py "
            "<input.ply> <output.pcd> [voxel_size] [output.png]"
        )
        sys.exit(1)

    ply_path = sys.argv[1]
    pcd_path = sys.argv[2]

    voxel_size = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    png_path = sys.argv[4] if len(sys.argv) > 4 else None


    print(f"Reading {ply_path} ...")

    pcd = o3d.io.read_point_cloud(ply_path)

    print(f"Loaded {len(pcd.points)} points")


    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)

        print(
            f"Downsampled to {len(pcd.points)} points "
            f"(voxel_size={voxel_size}m)"
        )


    o3d.io.write_point_cloud(pcd_path, pcd)

    print(f"Wrote {pcd_path}")


    if png_path:
        save_topdown_images(
            pcd,
            png_path.replace(".png",""),
            resolution=voxel_size if voxel_size > 0 else 0.05
        )


if __name__ == "__main__":
    main()
