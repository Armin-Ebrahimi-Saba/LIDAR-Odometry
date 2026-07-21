#!/usr/bin/env python3
"""Extract the ground-truth rows for one run from the combined GNSS CSV.

`xtrack_global_position_t12.csv` is PX4's filtered `vehicle_global_position`
(the recommended ground truth) and covers BOTH datasets back-to-back, so it
must be cropped to a single run's time window before use (see the project
intro: "one file covers both datasets -- filter by timestamp").

This reads the run window and source path straight from /configs/config_test1.yaml, crops on
`timestamp_sample` (the key shared with the rosbag, per the project intro;
falls back to `timestamp` if absent), and writes a per-run CSV next to the
source. All columns are preserved.

Usage:
    python scripts/extract_ground_truth.py [--config /configs/config_test1.yaml] [--output PATH]
"""
import argparse
from pathlib import Path

import pandas as pd
import yaml


def extract_ground_truth(config_path: str, output_path: str | None = None) -> Path:
    cfg = yaml.safe_load(Path(config_path).read_text())
    src = Path(cfg["paths"]["gnss_csv"])
    t0 = float(cfg["run"]["start_time"])
    t1 = float(cfg["run"]["end_time"])
    run_name = str(cfg["run"].get("name", "run")).lower()

    df = pd.read_csv(src)
    key = "timestamp_sample" if "timestamp_sample" in df.columns else "timestamp"
    cropped = df[(df[key] >= t0) & (df[key] <= t1)].reset_index(drop=True)
    if cropped.empty:
        raise SystemExit(
            f"No ground-truth rows in [{t0}, {t1}] (filtered on '{key}'). "
            f"Check run.start_time/end_time and that {src} covers this window."
        )

    out = Path(output_path) if output_path else src.with_name(f"{src.stem}_{run_name}.csv")
    cropped.to_csv(out, index=False)

    span = cropped[key].max() - cropped[key].min()
    print(f"[extract_ground_truth] source: {src} ({len(df)} rows)")
    print(f"[extract_ground_truth] {run_name}: {len(cropped)} rows over {span:.1f}s "
          f"(filtered on '{key}' in [{t0}, {t1}])")
    print(f"[extract_ground_truth] wrote {out}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="/configs/config_test1.yaml", help="path to /configs/config_test1.yaml")
    ap.add_argument("--output", default=None, help="output CSV path (default: alongside source)")
    args = ap.parse_args()
    extract_ground_truth(args.config, args.output)


if __name__ == "__main__":
    main()
