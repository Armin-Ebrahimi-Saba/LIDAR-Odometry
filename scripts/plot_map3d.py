#!/usr/bin/env python3
"""Render the SLAM 3D point-cloud map (map_local.pcd) as a self-contained,
interactive 3D HTML (Plotly) with the odometry trajectory overlaid.

The map and `poses_local.csv` are both in the local odometry world frame, so the
path lines up with the cloud. Open the HTML in a browser and orbit/zoom.

Making objects distinguishable
------------------------------
Colouring by *absolute* height Z is swamped by the SLAM's global vertical drift
(the map ramps ~90 m end to end), so local structure gets almost no colour range.
This script instead colours by **detrended** height -- the residual after
subtracting a fitted plane -- so the ramp reads *local* relief (walls, vehicles,
kerbs) at full contrast. `--shade` additionally shades each point by its surface
normal (a cheap lambertian) for 3-D form; `--color raw` restores absolute Z.

Usage:
    python scripts/plot_map3d.py
    python scripts/plot_map3d.py --shade          # add normal-based relief
    python scripts/plot_map3d.py --color raw --stride 2
"""
import argparse
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def _detrended_height(pts):
    """Height residual after removing a fitted plane z = a*x + b*y + c, so colour
    reflects local structure instead of the SLAM's global vertical drift."""
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(len(pts))]
    coef, *_ = np.linalg.lstsq(A, pts[:, 2], rcond=None)
    return pts[:, 2] - A @ coef


def _normal_shade(pts, radius=2.0):
    """Per-point lambertian shade in [0.35, 1] from estimated surface normals."""
    import open3d as o3d
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
    n = np.asarray(pc.normals)
    light = np.array([0.3, 0.3, 1.0]); light /= np.linalg.norm(light)
    return np.clip(0.35 + 0.65 * np.abs(n @ light), 0.0, 1.0)


def _budget_downsample(pts, max_points):
    """Voxel-downsample `pts` until at most `max_points` remain, so the HTML
    stays light enough for the browser to render (a full dense map is millions of
    markers -> blank/hung page). Returns the reduced array."""
    if len(pts) <= max_points:
        return pts
    import open3d as o3d
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts)
    v = 0.2
    while True:
        red = np.asarray(pc.voxel_down_sample(v).points)
        if len(red) <= max_points or v > 20:
            return red
        v *= 1.4


def make_map3d(config_path, stride=1, point_size=1.8, color="detrended",
               shade=False, max_points=120000, output=None):
    import open3d as o3d
    import plotly.graph_objects as go

    cfg = yaml.safe_load(Path(config_path).read_text())
    out_dir = Path(cfg["paths"]["output_dir"])
    pcd_path = out_dir / "map_local.pcd"
    if not pcd_path.exists():
        raise SystemExit(f"{pcd_path} not found -- run the odometry stage first.")

    raw = np.asarray(o3d.io.read_point_cloud(str(pcd_path)).points)[::stride]
    if len(raw) == 0:
        raise SystemExit("Point-cloud map is empty.")
    pts = _budget_downsample(raw, max_points)
    if len(pts) < len(raw):
        print(f"[plot_map3d] downsampled {len(raw)} -> {len(pts)} points "
              f"(max_points={max_points}) so the HTML renders")

    scalar = pts[:, 2] if color == "raw" else _detrended_height(pts)
    label = "height Z [m]" if color == "raw" else "local height [m]"
    lo, hi = np.percentile(scalar, [2, 98])   # robust range, ignore outliers

    fig = go.Figure()
    if shade:
        # Per-point RGB: local-height hue modulated by a normal-based shade so
        # surfaces catch the light and objects gain 3-D form.
        import matplotlib.cm as cm
        from matplotlib.colors import Normalize
        base = cm.viridis(Normalize(lo, hi, clip=True)(scalar))[:, :3]
        rgb = (base * _normal_shade(pts)[:, None] * 255).astype(int)
        colors = [f"rgb({r},{g},{b})" for r, g, b in rgb]
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], mode="markers", name="SLAM map",
            marker=dict(size=point_size, color=colors, opacity=1.0),
            hovertemplate="x=%{x:.1f}<br>y=%{y:.1f}<br>z=%{z:.1f} m<extra></extra>"))
    else:
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], mode="markers", name="SLAM map",
            marker=dict(size=point_size, color=scalar, colorscale="Viridis",
                        cmin=lo, cmax=hi, opacity=0.95,
                        colorbar=dict(title=label, thickness=14, len=0.6)),
            hovertemplate="x=%{x:.1f}<br>y=%{y:.1f}<br>z=%{z:.1f} m<extra></extra>"))

    poses_path = out_dir / "poses_local.csv"
    if poses_path.exists():
        p = pd.read_csv(poses_path)
        fig.add_trace(go.Scatter3d(
            x=p.x, y=p.y, z=p.z, mode="lines", name="LiDAR odometry",
            line=dict(color="crimson", width=4)))
        fig.add_trace(go.Scatter3d(
            x=[p.x.iloc[0]], y=[p.y.iloc[0]], z=[p.z.iloc[0]], mode="markers",
            name="start", marker=dict(size=5, color="white")))
        fig.add_trace(go.Scatter3d(
            x=[p.x.iloc[-1]], y=[p.y.iloc[-1]], z=[p.z.iloc[-1]], mode="markers",
            name="end", marker=dict(size=5, color="crimson", symbol="x")))

    fig.update_layout(
        title=f"{cfg['run'].get('name', 'run')} — SLAM 3D map "
              f"({len(pts)} pts, colour={color}{', shaded' if shade else ''})",
        template="plotly_dark", legend=dict(x=0.01, y=0.99),
        margin=dict(l=0, r=0, t=40, b=0),
        scene=dict(aspectmode="data",   # true proportions, no axis stretching
                   xaxis_title="X [m]", yaxis_title="Y [m]", zaxis_title="Z [m]"))

    out = Path(output) if output else out_dir / "map_3d.html"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(out), include_plotlyjs=True, full_html=True)  # self-contained
    print(f"[plot_map3d] {len(pts)} points (stride {stride}, colour={color}, shade={shade})")
    print(f"[plot_map3d] wrote {out}  -> open in a browser (orbit/zoom, offline-ok)")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--stride", type=int, default=1, help="keep every Nth map point")
    ap.add_argument("--point-size", type=float, default=1.8)
    ap.add_argument("--color", choices=["detrended", "raw"], default="detrended",
                    help="detrended = local relief (default); raw = absolute Z")
    ap.add_argument("--shade", action="store_true",
                    help="shade points by surface normal for 3-D relief")
    ap.add_argument("--max-points", type=int, default=120000,
                    help="voxel-downsample above this so the HTML stays renderable")
    ap.add_argument("--output", default=None, help="HTML output path")
    args = ap.parse_args()
    make_map3d(args.config, args.stride, args.point_size, args.color,
               args.shade, args.max_points, args.output)


if __name__ == "__main__":
    main()
