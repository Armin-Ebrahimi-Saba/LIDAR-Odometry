"""Load, time-crop and validity-filter the corrected GNSS ground-truth
trajectory (`xtrack_global_position_t12.csv`).

Per the project instructions, this corrected/filtered global-position CSV
-- not the raw `xtrack_gps_position_t12.csv` -- is the recommended ground
truth. The raw GPS file's `fix_type`/`eph` columns are not present in the
corrected file and are not used here; they only matter if you choose to
cross-check against the raw GPS solution separately.

The corrected file still has noise/jumps near the start and end (covering a
wider pre/post-lock window than either test run), so it must be cropped to
the run's actual time window before use.
"""
import pandas as pd


def load_global_position(csv_path: str) -> pd.DataFrame:
    return pd.read_csv(csv_path)


def crop_by_time(df: pd.DataFrame, start: float, end: float, time_col: str = "timestamp") -> pd.DataFrame:
    return df[(df[time_col] >= start) & (df[time_col] <= end)].reset_index(drop=True)


def filter_valid(df: pd.DataFrame, cfg: dict) -> pd.DataFrame:
    gt_cfg = cfg.get("ground_truth", {})
    out = df
    if gt_cfg.get("require_lat_lon_valid", True) and "lat_lon_valid" in out.columns:
        out = out[out["lat_lon_valid"].astype(bool)]
    if gt_cfg.get("require_alt_valid", True) and "alt_valid" in out.columns:
        out = out[out["alt_valid"].astype(bool)]
    return out.reset_index(drop=True)


def load_ground_truth_for_run(cfg: dict) -> pd.DataFrame:
    """Load the corrected global-position CSV, crop it to the configured run
    window (plus optional extra margin to trim residual edge noise), and
    drop rows flagged invalid."""
    paths = cfg["paths"]
    run = cfg["run"]
    margin = cfg.get("ground_truth", {}).get("crop_margin_s", 0.0)

    df = load_global_position(paths["gnss_csv"])
    df = crop_by_time(df, run["start_time"] + margin, run["end_time"] - margin)
    df = filter_valid(df, cfg)

    if df.empty:
        raise RuntimeError(
            "Ground truth is empty after cropping/filtering. Check that "
            "run.start_time / run.end_time in config.yaml fall within the "
            "CSV's actual timestamp range, and that the timestamp column "
            "name matches (expected: 'timestamp')."
        )
    return df
