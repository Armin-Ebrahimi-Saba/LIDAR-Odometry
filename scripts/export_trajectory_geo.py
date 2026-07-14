#!/usr/bin/env python3
"""Export the georeferenced LiDAR odometry (and GNSS ground truth) as GeoJSON
and GPX so it can be dropped onto OpenStreetMap.

Reads `<output_dir>/trajectory_latlon.csv` (produced by the align stage) plus the
run's GNSS ground truth, and writes:
  * <output_dir>/trajectory_map.geojson  -- two LineStrings (odometry, GNSS) +
    start/end points. Open at https://geojson.io (OSM background) or import into
    uMap (https://umap.openstreetmap.fr) / QGIS.
  * <output_dir>/trajectory_map.gpx      -- two tracks. Open at
    https://gpx.studio or https://www.openstreetmap.org (Traces), or in JOSM.

Usage:
    python scripts/export_trajectory_geo.py                 # uses config.yaml
    python scripts/export_trajectory_geo.py --stride 5      # thin points 5x
"""
import argparse
import json
from pathlib import Path
import sys

import pandas as pd
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sensys_slam.groundtruth import load_ground_truth_for_run  # noqa: E402


def _line_feature(lat, lon, name, color):
    return {
        "type": "Feature",
        "properties": {"name": name, "stroke": color, "stroke-width": 3},
        "geometry": {"type": "LineString",
                     "coordinates": [[float(x), float(y)] for x, y in zip(lon, lat)]},
    }


def _point_feature(lat, lon, name, color):
    return {
        "type": "Feature",
        "properties": {"name": name, "marker-color": color},
        "geometry": {"type": "Point", "coordinates": [float(lon), float(lat)]},
    }


def export_geo(config_path, stride=1, output=None):
    cfg = yaml.safe_load(Path(config_path).read_text())
    out_dir = Path(cfg["paths"]["output_dir"])
    traj = pd.read_csv(out_dir / "trajectory_latlon.csv")[::stride]
    gt = load_ground_truth_for_run(cfg)[::stride]

    # --- GeoJSON (LineStrings + start/end markers) ---
    feats = [
        _line_feature(gt.lat.values, gt.lon.values, "GNSS ground truth", "#1f77b4"),
        _line_feature(traj.lat.values, traj.lon.values, "LiDAR odometry", "#ff7f0e"),
        _point_feature(traj.lat.iloc[0], traj.lon.iloc[0], "start", "#000000"),
        _point_feature(traj.lat.iloc[-1], traj.lon.iloc[-1], "odometry end", "#ff7f0e"),
        _point_feature(gt.lat.iloc[-1], gt.lon.iloc[-1], "GNSS end", "#1f77b4"),
    ]
    geojson = {"type": "FeatureCollection", "features": feats}
    gj_path = Path(output) if output else out_dir / "trajectory_map.geojson"
    gj_path.write_text(json.dumps(geojson))

    # --- GPX (two tracks) ---
    def _trk(df, name):
        pts = "".join(f'<trkpt lat="{la:.8f}" lon="{lo:.8f}"/>'
                      for la, lo in zip(df.lat.values, df.lon.values))
        return f"<trk><name>{name}</name><trkseg>{pts}</trkseg></trk>"

    gpx = ('<?xml version="1.0" encoding="UTF-8"?>'
           '<gpx version="1.1" creator="sensys_slam" '
           'xmlns="http://www.topografix.com/GPX/1/1">'
           + _trk(gt, "GNSS ground truth")
           + _trk(traj, "LiDAR odometry") + "</gpx>")
    gpx_path = gj_path.with_suffix(".gpx")
    gpx_path.write_text(gpx)

    print(f"[export_geo] {len(traj)} odometry pts, {len(gt)} GNSS pts (stride {stride})")
    print(f"[export_geo] wrote {gj_path}")
    print(f"[export_geo] wrote {gpx_path}")
    print("[export_geo] View on OpenStreetMap:")
    print("   * GeoJSON -> https://geojson.io  (paste/open; OSM basemap)")
    print("   * GPX     -> https://gpx.studio   or openstreetmap.org (Traces)")
    return gj_path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--stride", type=int, default=1, help="keep every Nth point")
    ap.add_argument("--output", default=None, help="GeoJSON output path")
    args = ap.parse_args()
    export_geo(args.config, args.stride, args.output)


if __name__ == "__main__":
    main()
