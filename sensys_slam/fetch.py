"""Fetch the raw inputs the pipeline needs, if they are not already on disk.

Everything lives under `data/` and is downloaded straight from the TU Berlin
tubcloud public share (plus px4_msgs from GitHub). All of it is idempotent: each
step first checks for the file/folder it would produce and returns immediately if
it is there, so `ensure_data(...)` is cheap to call at the top of every run.

What gets fetched:

  data/test1/rosbag/          <- Test1_data.zip   (~21 GB download, ~25 GB on disk)
  data/test2/rosbag/          <- Test2_data.zip   (~16 GB download, ~19 GB on disk)
  data/xtrack_gnss_corrected/ <- the corrected GNSS reference files (~19 MB)
  data/px4_msgs/              <- PX4 message definitions (release/1.17, ~2 MB)

Only the dataset named by the config's `paths.bag_dir` is fetched, so a Test1 run
never pulls Test2's 16 GB.

The dataset zips each contain a single top-level `rosbag/` folder (the .db3 bag,
metadata.yaml and the exported `rosbag_data/` with laz_clouds, images, ...), so
they are extracted into `data/<name>/`.

Note on px4_msgs: only the plain-text `msg/*.msg` definitions are used --
`rosbags` parses them directly to decode the PX4 topics the bag records without
embedded definitions. There is **no ROS/colcon build step**; a ROS 2 install is
not required.

Downloads stream to a `.part` file and resume with an HTTP Range request if
interrupted, which matters at this size.
"""
from pathlib import Path
import shutil
import time
import urllib.request
import zipfile

# Public share on the TU Berlin cloud. The WebDAV path serves individual files
# directly; note the folder-level `?accept=zip` download does NOT work here (it
# answers with an HTML page), so the GNSS folder is fetched file by file.
TUBCLOUD = "https://tubcloud.tu-berlin.de/public.php/dav/files/eaZMa7sGCZbiyCi"

# dataset name (and data/<name>/ folder) -> zip on the share
DATASET_ZIPS = {"test1": "Test1_data.zip", "test2": "Test2_data.zip"}

# Contents of the share's xtrack_gnss_corrected/ folder. Only the first is read
# by the pipeline (`paths.gnss_csv`); the rest are the reference material that
# ships with it.
GNSS_DIR = "xtrack_gnss_corrected"
GNSS_FILES = [
    "xtrack_global_position_t12.csv",
    "xtrack_gps_position_t12.csv",
    "xtrack_globalpos_vs_gps.kml",
    "xtrack_globalpos_vs_gps_satellite_map.html",
]

# PX4 message definitions, matching the firmware that recorded these bags.
PX4_MSGS_URL = "https://codeload.github.com/PX4/px4_msgs/zip/refs/heads/release/1.17"
PX4_MSGS_ROOT = "px4_msgs-release-1.17"     # top-level folder inside that zip


def _human(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if abs(n) < 1024:
            return f"{n:.1f} {unit}"
        n /= 1024
    return f"{n:.1f} PB"


def _download(url: str, dest: Path) -> Path:
    """Stream `url` to `dest`, resuming a partial `.part` file if one is there."""
    if dest.exists():
        print(f"[fetch] have {dest} ({_human(dest.stat().st_size)})")
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    part = dest.with_name(dest.name + ".part")
    have = part.stat().st_size if part.exists() else 0

    req = urllib.request.Request(url)
    if have:
        req.add_header("Range", f"bytes={have}-")
    with urllib.request.urlopen(req) as resp:
        resuming = resp.status == 206
        if have and not resuming:       # server ignored the Range -> start over
            have = 0
        total = int(resp.headers.get("Content-Length", 0)) + have
        free = shutil.disk_usage(dest.parent).free
        if total and free < total - have:
            raise RuntimeError(
                f"Not enough free space for {dest.name}: need {_human(total - have)}, "
                f"{_human(free)} available.")
        print(f"[fetch] {'resuming' if resuming else 'downloading'} {url}\n"
              f"[fetch]   -> {dest} ({_human(total) if total else 'unknown size'})")

        mode = "ab" if resuming else "wb"
        got, last = have, time.monotonic()
        with open(part, mode) as f:
            while chunk := resp.read(1 << 22):      # 4 MiB
                f.write(chunk)
                got += len(chunk)
                if time.monotonic() - last > 5:     # progress line every 5 s
                    pct = f" ({100 * got / total:.1f}%)" if total else ""
                    print(f"[fetch]   {_human(got)}{pct}", flush=True)
                    last = time.monotonic()
    part.rename(dest)
    print(f"[fetch]   done: {dest} ({_human(dest.stat().st_size)})")
    return dest


def _extract(zip_path: Path, dest_dir: Path) -> None:
    """Extract `zip_path` into `dest_dir`, skipping members already extracted."""
    dest_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path) as z:
        members = [m for m in z.infolist()
                   if not (dest_dir / m.filename).exists() or m.is_dir()]
        if not members:
            return
        print(f"[fetch] extracting {len(members)} entries from {zip_path.name} "
              f"-> {dest_dir}/")
        z.extractall(dest_dir, members)


def ensure_dataset(name: str, data_root: Path, keep_zip: bool = False) -> Path:
    """Make sure `data/<name>/rosbag/` exists; download + unzip it if not.

    Returns the bag directory. With `keep_zip=False` the (multi-GB) zip is
    deleted once it has been extracted."""
    dest = data_root / name
    bag_dir = dest / "rosbag"
    if (bag_dir / "rosbag_0.db3").exists():
        return bag_dir
    zip_name = DATASET_ZIPS[name]
    zip_path = _download(f"{TUBCLOUD}/{zip_name}", dest / zip_name)
    _extract(zip_path, dest)
    if not keep_zip:
        zip_path.unlink()
        print(f"[fetch] removed {zip_path} (extracted)")
    return bag_dir


def ensure_gnss(data_root: Path) -> Path:
    """Make sure the corrected GNSS reference files are in
    `data/xtrack_gnss_corrected/`. Returns that directory."""
    dest = data_root / GNSS_DIR
    for name in GNSS_FILES:
        if not (dest / name).exists():
            _download(f"{TUBCLOUD}/{GNSS_DIR}/{name}", dest / name)
    return dest


def ensure_px4_msgs(data_root: Path) -> Path:
    """Make sure the PX4 message definitions are in `data/px4_msgs/`.

    Only the `msg/*.msg` text files are used (parsed by `rosbags` to decode the
    PX4 topics), so this is a plain download -- no ROS 2 workspace and no colcon
    build. Returns the px4_msgs directory."""
    dest = data_root / "px4_msgs"
    if (dest / "msg" / "VehicleAttitude.msg").exists():
        return dest
    zip_path = _download(PX4_MSGS_URL, data_root / "px4_msgs.zip")
    _extract(zip_path, data_root)
    extracted = data_root / PX4_MSGS_ROOT
    if extracted.exists():
        extracted.rename(dest)
    zip_path.unlink()
    return dest


def dataset_name_for(cfg: dict) -> str | None:
    """Which of `DATASET_ZIPS` the config's `paths.bag_dir` refers to (None if it
    points somewhere else, e.g. a hand-placed bag)."""
    parts = Path(cfg["paths"]["bag_dir"]).parts
    return next((n for n in DATASET_ZIPS if n in parts), None)


def ensure_data(cfg: dict, data_root="data", keep_zips: bool = False) -> None:
    """Fetch whatever this run needs and is missing: the config's own dataset,
    the corrected GNSS reference, and the PX4 message definitions."""
    root = Path(data_root)
    name = dataset_name_for(cfg)
    if name is None:
        print(f"[fetch] paths.bag_dir = {cfg['paths']['bag_dir']} is not one of "
              f"{sorted(DATASET_ZIPS)}; skipping the dataset download.")
    else:
        bag_dir = ensure_dataset(name, root, keep_zips)
        print(f"[fetch] {name}: {bag_dir}")
    print(f"[fetch] ground truth: {ensure_gnss(root)}")
    print(f"[fetch] px4_msgs: {ensure_px4_msgs(root)}")


if __name__ == "__main__":
    import argparse
    import yaml

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/config_test1.yaml")
    ap.add_argument("--data-root", default="data")
    ap.add_argument("--keep-zips", action="store_true",
                    help="keep the downloaded dataset zip after extracting it")
    a = ap.parse_args()
    ensure_data(yaml.safe_load(Path(a.config).read_text()), a.data_root, a.keep_zips)
