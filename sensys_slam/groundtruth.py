"""Load, time-crop and validity-filter the corrected GNSS ground-truth
trajectory (`xtrack_global_position_t12.csv`).

This is PX4's filtered `vehicle_global_position` (the recommended ground
truth). One file covers both datasets, so it must be cropped to the run's time
window before use. The project intro notes `timestamp_sample` is the key shared
with the rosbag, so cropping/matching prefer that column when present.
"""
import pandas as pd


def load_global_position(csv_path: str) -> pd.DataFrame:
    return pd.read_csv(csv_path)


def _time_col(df: pd.DataFrame) -> str:
    return "timestamp_sample" if "timestamp_sample" in df.columns else "timestamp"


def crop_by_time(df: pd.DataFrame, start: float, end: float) -> pd.DataFrame:
    col = _time_col(df)
    return df[(df[col] >= start) & (df[col] <= end)].reset_index(drop=True)


def filter_valid(df: pd.DataFrame, cfg: dict) -> pd.DataFrame:
    gt_cfg = cfg.get("ground_truth", {})
    out = df
    if gt_cfg.get("require_lat_lon_valid", True) and "lat_lon_valid" in out.columns:
        out = out[out["lat_lon_valid"].astype(bool)]
    if gt_cfg.get("require_alt_valid", True) and "alt_valid" in out.columns:
        out = out[out["alt_valid"].astype(bool)]
    return out.reset_index(drop=True)


def load_ground_truth_for_run(cfg: dict) -> pd.DataFrame:
    """Load the corrected CSV, crop to the run window (+ optional margin), and
    drop rows flagged invalid. The returned frame always carries a `timestamp`
    column for downstream matching (set to the shared key if needed)."""
    paths, run = cfg["paths"], cfg["run"]
    margin = cfg.get("ground_truth", {}).get("crop_margin_s", 0.0)

    df = load_global_position(paths["gnss_csv"])
    df = crop_by_time(df, run["start_time"] + margin, run["end_time"] - margin)
    df = filter_valid(df, cfg)

    if df.empty:
        raise RuntimeError(
            "Ground truth is empty after cropping/filtering. Check that "
            "run.start_time / run.end_time fall within the CSV's timestamp "
            "range, and that the timestamp column name matches."
        )

    # Downstream (align/evaluate) matches on a 'timestamp' column; make it the
    # shared key so it lines up with the rosbag/odometry times.
    df["timestamp"] = df[_time_col(df)].values
    return df
