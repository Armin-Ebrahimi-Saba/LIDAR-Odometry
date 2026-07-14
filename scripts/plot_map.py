#!/usr/bin/env python3
"""Render the final LiDAR odometry (and GNSS ground truth) on an OpenStreetMap
background as a standalone Leaflet HTML map.

Reads `<output_dir>/trajectory_latlon.csv` (from the align stage) plus the run's
GNSS ground truth, and writes `<output_dir>/trajectory_map.html`. Open it in a
browser (needs internet for the OSM tiles + Leaflet from CDN). Orange = LiDAR
odometry, blue = GNSS ground truth; toggle either via the layer control.

Usage:
    python scripts/plot_map.py
    python scripts/plot_map.py --stride 3 --output outputs/test1/map.html
"""
import argparse
import json
from pathlib import Path
import sys

import pandas as pd
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sensys_slam.groundtruth import load_ground_truth_for_run  # noqa: E402

_HTML = """<!doctype html><html><head><meta charset="utf-8">
<title>{title}</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>html,body,#map{{height:100%;margin:0}}
.info{{position:absolute;z-index:1000;top:8px;right:8px;background:#fff;
padding:6px 10px;border-radius:4px;font:13px sans-serif;box-shadow:0 1px 4px #0006}}</style>
</head><body><div id="map"></div>
<div class="info"><b>{title}</b><br>
<span style="color:#ff7f0e">&#9632;</span> LiDAR odometry &nbsp;
<span style="color:#1f77b4">&#9632;</span> GNSS ground truth</div>
<script>
var odo={odo}, gnss={gnss};
var map=L.map('map');
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',
  {{maxZoom:19, attribution:'&copy; OpenStreetMap contributors'}}).addTo(map);
var g=L.polyline(gnss,{{color:'#1f77b4',weight:3,opacity:0.9}}).addTo(map);
var o=L.polyline(odo,{{color:'#ff7f0e',weight:3,opacity:0.9}}).addTo(map);
L.circleMarker(odo[0],{{radius:6,color:'#000',fillColor:'#000',fillOpacity:1}})
  .addTo(map).bindPopup('start');
L.circleMarker(odo[odo.length-1],{{radius:6,color:'#ff7f0e',fillColor:'#ff7f0e',
  fillOpacity:1}}).addTo(map).bindPopup('odometry end');
L.circleMarker(gnss[gnss.length-1],{{radius:6,color:'#1f77b4',fillColor:'#1f77b4',
  fillOpacity:1}}).addTo(map).bindPopup('GNSS end');
map.fitBounds(g.getBounds().extend(o.getBounds()), {{padding:[20,20]}});
L.control.layers(null,{{'LiDAR odometry':o,'GNSS ground truth':g}},
  {{collapsed:false}}).addTo(map);
</script></body></html>"""


def make_map(config_path, stride=1, output=None):
    cfg = yaml.safe_load(Path(config_path).read_text())
    out_dir = Path(cfg["paths"]["output_dir"])
    traj = pd.read_csv(out_dir / "trajectory_latlon.csv")[::stride]
    gt = load_ground_truth_for_run(cfg)[::stride]

    # Leaflet wants [lat, lon] pairs.
    odo = [[float(a), float(b)] for a, b in zip(traj.lat, traj.lon)]
    gnss = [[float(a), float(b)] for a, b in zip(gt.lat, gt.lon)]

    html = _HTML.format(title=f"{cfg['run'].get('name', 'run')} — odometry on OpenStreetMap",
                        odo=json.dumps(odo), gnss=json.dumps(gnss))
    out = Path(output) if output else out_dir / "trajectory_map.html"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html)
    print(f"[plot_map] {len(odo)} odometry pts, {len(gnss)} GNSS pts (stride {stride})")
    print(f"[plot_map] wrote {out}  -> open in a browser (needs internet for OSM tiles)")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--stride", type=int, default=1, help="keep every Nth point")
    ap.add_argument("--output", default=None, help="HTML output path")
    args = ap.parse_args()
    make_map(args.config, args.stride, args.output)


if __name__ == "__main__":
    main()
