#!/usr/bin/env python3
"""Colorize an existing accumulated map (`map_local.pcd`) by point height (Z).

This is a pure post-process: it reads the already-built map, maps each point's
elevation through a matplotlib colormap, and writes a new coloured PCD. No bag
read, no odometry, no ICP -- it only touches the geometry already on disk, so it
is instant and never changes the map's shape.

Height colouring is the standard "readable" SLAM-map look: ground, walls and
roofs separate cleanly by elevation. For a photographic look coloured by the
sensor's real return strength, use `rebuild_map.py --color intensity` instead.

Usage:
    python scripts/colorize_map.py                       # outputs map_local_height.pcd
    python scripts/colorize_map.py --cmap viridis
    python scripts/colorize_map.py --input outputs/test1/map_local.pcd \
                                   --output outputs/test1/map_local_height.pcd
    python scripts/colorize_map.py --percentile 2 98     # clip Z range for contrast
"""
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def colorize_by_height(input_path, output_path, cmap="turbo", percentile=(1.0, 99.0)):
    import open3d as o3d
    from matplotlib import colormaps

    pcd = o3d.io.read_point_cloud(str(input_path))
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        raise SystemExit(f"{input_path} has no points.")

    z = pts[:, 2]
    lo, hi = np.percentile(z, percentile)
    if hi <= lo:
        lo, hi = z.min(), z.max()
    norm = np.clip((z - lo) / (hi - lo + 1e-12), 0.0, 1.0)

    colors = colormaps[cmap](norm)[:, :3]  # drop alpha
    pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.io.write_point_cloud(str(output_path), pcd)
    print(f"[colorize_map] {len(pts)} points coloured by Z in "
          f"[{lo:.2f}, {hi:.2f}] m ({cmap}) -> {output_path} "
          f"({Path(output_path).stat().st_size / 1e6:.1f} MB)")
    return output_path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--input", default=None, help="input PCD (default: <output_dir>/map_local.pcd)")
    ap.add_argument("--output", default=None,
                    help="output PCD (default: <input stem>_height.pcd)")
    ap.add_argument("--cmap", default="turbo", help="matplotlib colormap name")
    ap.add_argument("--percentile", type=float, nargs=2, default=(1.0, 99.0),
                    metavar=("LO", "HI"), help="percentile clip of Z for colour range")
    args = ap.parse_args()

    if args.input:
        in_path = Path(args.input)
    else:
        cfg = yaml.safe_load(Path(args.config).read_text())
        in_path = Path(cfg["paths"]["output_dir"]) / "map_local.pcd"
    if not in_path.exists():
        raise SystemExit(f"{in_path} not found -- build the map first.")

    out_path = Path(args.output) if args.output else in_path.with_name(in_path.stem + "_height.pcd")
    colorize_by_height(in_path, out_path, args.cmap, tuple(args.percentile))


if __name__ == "__main__":
    main()
