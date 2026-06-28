"""Body frame definition and sensor extrinsics.

Per DESCRIPTION.md, the **Pixhawk coordinate system is taken as the body frame**:
FRD -- X forward, Y right, Z down. Expressing every sensor in this one frame is
what lets the LiDAR clouds and the PX4 attitude (which is body->NED) be combined
consistently (e.g. attitude-based deskew, heading).

The Ouster sensor frame is FLU (X forward, Y left, Z up). Assuming the Ouster is
mounted axis-aligned with the Pixhawk -- i.e. only the axis *convention* differs,
not the orientation -- the LiDAR->body rotation is the FLU->FRD flip: a 180 deg
rotation about X (negate Y and Z). If the true mounting rotation is known, set
`lidar.extrinsic_rpy_deg` in the config to override.

Note: only the rotation is modelled here. The LiDAR<->Pixhawk translation (lever
arm) is unknown and assumed zero; for odometry a constant lever arm is absorbed
by the start-anchored alignment, and at this platform's speeds the rotational
lever-arm velocity term is negligible.
"""
import numpy as np
from scipy.spatial.transform import Rotation

# Ouster FLU (X-fwd, Y-left, Z-up) -> Pixhawk FRD (X-fwd, Y-right, Z-down):
# negate Y and Z, i.e. a 180 deg rotation about X.
OUSTER_FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def build_lidar_to_body(cfg: dict):
    """Return the 3x3 LiDAR->body(FRD) rotation, or None if `lidar.body_frame`
    is off. `lidar.extrinsic_rpy_deg` (intrinsic XYZ, degrees) overrides the
    default FLU->FRD convention flip."""
    lidar = cfg.get("lidar", {})
    if not lidar.get("body_frame", False):
        return None
    rpy = lidar.get("extrinsic_rpy_deg")
    if rpy:
        return Rotation.from_euler("xyz", list(rpy), degrees=True).as_matrix()
    return OUSTER_FLU_TO_FRD.copy()
