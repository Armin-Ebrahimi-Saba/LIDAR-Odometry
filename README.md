# GLIM (CPU-only) — Ouster OS0-32 + Pixhawk

CPU-only build of [GLIM](https://github.com/koide3/glim) (LiDAR-Inertial SLAM) for
the Sensor Systems project (Task 3: LIDAR-based Positioning and Map Generation).
No NVIDIA GPU required — tested on WSL2, Ubuntu 24.04, ROS2 Jazzy.

## Workspace layout

- **This repo** = the working directory. `config/` holds GLIM's config (CPU
  variants already selected for odometry/sub_mapping/global_mapping). `data/`
  (gitignored) is where datasets/bags go locally — download fresh per machine,
  never commit them.
- **`~/src/`** (outside this repo) — source checkouts of GTSAM, gtsam_points, and
  Iridescence, built and installed system-wide via `make install`. Third-party
  dependencies, not part of this repo.
- **`~/ros2_ws/`** (outside this repo) — a colcon workspace containing the `glim`
  and `glim_ros2` repos, built via `colcon build`. Also not part of this repo —
  rebuild it locally following the steps below.

## Why build from source instead of the PPA

koide3 provides prebuilt packages via a PPA (`ros-jazzy-glim-ros`,
`libgtsam-points-dev`), which is much faster to install. **As of July 2026, the
CPU-only PPA packages for Ubuntu 24.04 were broken**: running `glim_rosbag` failed
with `undefined symbol: _ZTVN5gtsam28PreintegratedImuMeasurementsE` — an ABI
mismatch between the packaged `libgtsam-notbb-dev 4.3.0` and the `gtsam_points`/
`glim_ros` binaries. This was reproducible even after pinning to older,
previously-matched package versions (1.2.0), so it isn't a version-skew issue we
could fix by picking a different combination — it's a build inconsistency in the
PPA itself. Building the full stack from source, all against the same GTSAM
headers, avoids it entirely. If you want to try the PPA route first (it's worth
a shot — may be fixed by the time you read this):

```bash
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev
sudo apt install -y libgtsam-points-dev ros-jazzy-glim-ros
```

If `ros2 run glim_ros glim_rosbag ...` throws a `symbol lookup error`, skip to the
source build below.

## 1. Install ROS2 Jazzy

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> **Certificate/TLS error on `packages.ros.org`?** This is commonly antivirus
> HTTPS-inspection (Kaspersky, ESET, corporate endpoint protection, etc.)
> intercepting the connection. Using `http://` instead of `https://` for this one
> repo works around it — apt's actual security comes from the GPG-signed
> keyring, not the transport encryption. Confirm first with:
> ```bash
> date  # check clock isn't badly skewed (WSL2 can drift after sleep/resume)
> openssl s_client -connect packages.ros.org:443 -servername packages.ros.org </dev/null 2>/dev/null | openssl x509 -noout -issuer -subject
> ```
> If the `issuer` shows an antivirus vendor instead of a normal CA, that
> confirms it.

## 2. Build GTSAM 4.3a0 from source

```bash
sudo apt install -y build-essential cmake git \
  libboost-all-dev libeigen3-dev \
  libomp-dev libmetis-dev libfmt-dev libspdlog-dev \
  libglm-dev libglfw3-dev libpng-dev libjpeg-dev

mkdir -p ~/src && cd ~/src
git clone https://github.com/borglab/gtsam
cd gtsam
git checkout 4.3a0
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

This is the slowest step (15-40+ min depending on CPU). If the build gets killed
partway through with no clear error, it's likely WSL2 running out of RAM under
full parallelism — retry with `make -j2`.

## 3. Build gtsam_points from source (CPU-only)

```bash
cd ~/src
git clone https://github.com/koide3/gtsam_points
mkdir gtsam_points/build && cd gtsam_points/build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_TBB=OFF \
  -DBUILD_WITH_OPENMP=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

Check the `cmake` output includes a line like
`GTSAM include directory: /usr/local/lib/cmake/GTSAM/../../../include` — confirms
it's linking against the GTSAM you just built in step 2, not a stale copy.

## 4. Build Iridescence (map viewer)

```bash
cd ~/src
git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
```

## 5. Build GLIM via colcon

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/koide3/glim
git clone https://github.com/koide3/glim_ros2

cd ~/ros2_ws
colcon build --cmake-args \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_VIEWER=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:
```bash
ros2 pkg list | grep glim
# should list: glim, glim_ros
```

## 6. Config

`config/` in this repo already has CPU variants selected in `config.json`:
```json
"config_odometry": "config_odometry_cpu.json",
"config_sub_mapping": "config_sub_mapping_cpu.json",
"config_global_mapping": "config_global_mapping_cpu.json",
```

Run GLIM against a rosbag, pointing explicitly at this config folder:
```bash
cd ~/LIDAR-based-Positioning   # or wherever this repo is cloned
ros2 run glim_ros glim_rosbag <path_to_bag> --ros-args -p config_path:=$(realpath config)
```

Output (trajectory files, submaps, factor graph) is saved to `/tmp/dump`.

## Test dataset (demo / sanity check)

Official GLIM demo bag, used to verify this whole pipeline works before pointing
it at our own course data:

```bash
mkdir -p data && cd data
curl -L -o os1_128_01_downsampled.tar.gz "https://zenodo.org/record/7233945/files/os1_128_01_downsampled.tar.gz?download=1"
tar -xzf os1_128_01_downsampled.tar.gz
```

(~406 MB, ROS2 `.db3` format.) Note: the link on GLIM's own docs page
(`staff.aist.go.jp`) was found to serve a truncated/corrupt file as of July 2026 —
use the Zenodo link above instead.

Run it:
```bash
ros2 run glim_ros glim_rosbag $(realpath data/os1_128_01_downsampled) --ros-args -p config_path:=$(realpath config)
```

**Verified working** (July 2026): full SLAM pipeline (odometry → local mapping →
global mapping) ran end-to-end on CPU only, viewer displayed live map + trajectory,
output written to `/tmp/dump`.

## Status / Next steps

- [x] CPU-only GLIM build working (this branch)
- [x] Verified against official demo dataset
- [ ] Configure `config_sensors.json` / `config_ros.json` for our Ouster OS0-32 +
      Pixhawk topic names and LiDAR-IMU extrinsics
- [ ] Run against course datasets (Test1/Test2)
- [ ] Export trajectory to LatLon CSV, compute RMSE vs GNSS ground truth
- [ ] Export point cloud map as `.pcd`
