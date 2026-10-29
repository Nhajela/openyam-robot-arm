"""interfaces.wristcam.camera: the wrist camera's lens model and pose helpers.

One ``CameraModel`` type covers the three lens models the intrinsics solver tries:

* ``fisheye``  : cv2.fisheye (equidistant + k1..k4), for the wide-angle "USB CAMERA 4K" module
* ``rational`` : cv2 standard model with CALIB_RATIONAL_MODEL (k1..k6, p1, p2)
* ``pinhole``  : cv2 standard model (k1, k2, p1, p2, k3)

``project`` (camera-frame points -> pixels) and ``rays`` (pixels -> unit rays) are what every
other module uses, so nothing downstream cares which model won.

Frames (all 4x4, metres): ``T_a_b`` maps points in frame b to frame a. Base = arm base (z up,
x forward). ``tip`` = armctl.kin ``gripper_tip``. ``cam`` = OpenCV camera (x right, y down,
z along the optical axis).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
CALIB_DIR = HERE / "calib"
DEFAULT_INTRINSICS = CALIB_DIR / "wristcam.json"
DEFAULT_HANDEYE = CALIB_DIR / "handeye.json"

# Nominal mounting until hand-eye is solved, MEASURED on the real arm 2026-10-28 (scan 1,
# experiments/2026-10-28-laptop-systems-check): the camera does NOT look along the fingers.
# Its optical axis is tilted ~60 deg from the finger axis (tip -z) toward tip +x, about the
# tip's y axis; the fingers appear at the bottom of the image and the jaws open left/right.
# Fit: the arm's base lands at pixel ~(660, 470) in all 7 fingers-down scan poses (median
# error 54 px with the uncalibrated fisheye guess). Camera ~6 cm behind the tip (palm side).
def _nominal_tip_cam() -> np.ndarray:
    a, r = np.radians(-60.0), np.radians(90.0)
    Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    Rz = np.array([[np.cos(r), -np.sin(r), 0], [np.sin(r), np.cos(r), 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Ry @ np.diag([1.0, -1.0, -1.0]) @ Rz
    T[:3, 3] = [0.0, 0.0, 0.06]
    return T


T_TIP_CAM_NOMINAL = _nominal_tip_cam()

# Image regions the jaws hide (normalised image coords, polygons). The person at the arm
# reports the jaws in the LOWER corners. Override with a measured mask in handeye.json.
JAW_POLYGONS = [
    [(0.0, 0.78), (0.17, 1.0), (0.0, 1.0)],
    [(1.0, 0.78), (0.83, 1.0), (1.0, 1.0)],
]


def make_T(R=None, t=None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.ravel(t)
    return T


def inv_T(T) -> np.ndarray:
    T = np.asarray(T, float)
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rodrigues(rvec) -> np.ndarray:
    return cv2.Rodrigues(np.asarray(rvec, float).reshape(3))[0]


def rot_angle_deg(R) -> float:
    c = np.clip((np.trace(np.asarray(R)[:3, :3]) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(c)))


def rot_err_deg(R1, R2) -> float:
    return rot_angle_deg(np.asarray(R1)[:3, :3].T @ np.asarray(R2)[:3, :3])


def rot_axis(axis, deg) -> np.ndarray:
    a = np.asarray(axis, float)
    return rodrigues(a / np.linalg.norm(a) * np.radians(deg))


def mean_pose(Ts) -> np.ndarray:
    """Average of rigid transforms (chordal rotation mean via SVD)."""
    Ts = [np.asarray(T, float) for T in Ts]
    U, _, Vt = np.linalg.svd(sum(T[:3, :3] for T in Ts))
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    return make_T(R, np.mean([T[:3, 3] for T in Ts], axis=0))


@dataclass
class CameraModel:
    model: str                       # fisheye | rational | pinhole
    K: np.ndarray
    D: np.ndarray
    size: tuple                      # (w, h)
    rms: float | None = None
    meta: dict = field(default_factory=dict)

    def __post_init__(self):
        self.K = np.asarray(self.K, float).reshape(3, 3)
        self.D = np.asarray(self.D, float).ravel()
        self.size = (int(self.size[0]), int(self.size[1]))
        if self.model not in ("fisheye", "rational", "pinhole"):
            raise ValueError(f"unknown camera model {self.model!r}")

    # ------------------------------------------------------------------ geometry
    def project(self, P_cam) -> np.ndarray:
        """Camera-frame points (N,3) -> pixels (N,2); NaN for points behind the camera or
        more than ~85 deg off-axis."""
        P = np.asarray(P_cam, float).reshape(-1, 3)
        out = np.full((len(P), 2), np.nan)
        z = P[:, 2]
        r = np.linalg.norm(P[:, :2], axis=1)
        ok = (z > 1e-6) & (np.arctan2(r, np.maximum(z, 1e-12)) < np.radians(85))
        if not ok.any():
            return out
        Q = P[ok].reshape(-1, 1, 3)
        rv, tv = np.zeros(3), np.zeros(3)
        if self.model == "fisheye":
            uv, _ = cv2.fisheye.projectPoints(Q, rv, tv, self.K, self.D[:4])
        else:
            uv, _ = cv2.projectPoints(Q, rv, tv, self.K, self.D)
        out[ok] = uv.reshape(-1, 2)
        return out

    def normalize(self, uv) -> np.ndarray:
        """Pixels (N,2) -> undistorted normalised coordinates (x/z, y/z)."""
        pts = np.asarray(uv, np.float64).reshape(-1, 1, 2)
        crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 40, 1e-9)
        if self.model == "fisheye":
            n = cv2.fisheye.undistortPoints(pts, self.K, self.D[:4], criteria=crit)
        else:
            n = cv2.undistortPointsIter(pts, self.K, self.D, None, None, crit)
        return n.reshape(-1, 2)

    def rays(self, uv) -> np.ndarray:
        n = self.normalize(uv)
        d = np.c_[n, np.ones(len(n))]
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def hfov_deg(self) -> float:
        w, h = self.size
        r = self.rays([[0, self.K[1, 2]], [w - 1, self.K[1, 2]]])
        return float(np.degrees(np.arccos(np.clip(r[0] @ r[1], -1, 1))))

    def scaled(self, size) -> "CameraModel":
        """Same lens at another stream size (same aspect): scale K."""
        sx, sy = size[0] / self.size[0], size[1] / self.size[1]
        K = np.diag([sx, sy, 1.0]) @ self.K
        return CameraModel(self.model, K, self.D.copy(), size, self.rms, dict(self.meta))

    def for_image(self, img) -> "CameraModel":
        h, w = img.shape[:2]
        return self if (w, h) == self.size else self.scaled((w, h))

    def jaw_mask(self, polygons=None) -> np.ndarray:
        """uint8 mask, 255 = usable pixel (not hidden by the jaws)."""
        w, h = self.size
        m = np.full((h, w), 255, np.uint8)
        for poly in (JAW_POLYGONS if polygons is None else polygons):
            p = np.array([[x * w, y * h] for x, y in poly], np.int32)
            cv2.fillPoly(m, [p], 0)
        return m

    # ------------------------------------------------------------------ io
    def to_dict(self) -> dict:
        return {"model": self.model, "K": self.K.round(6).tolist(), "D": self.D.round(8).tolist(),
                "size": list(self.size), "rms": self.rms, "meta": self.meta}

    @classmethod
    def from_dict(cls, d: dict) -> "CameraModel":
        return cls(d["model"], d["K"], d["D"], tuple(d["size"]), d.get("rms"), d.get("meta", {}))

    def save(self, path=DEFAULT_INTRINSICS) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=1) + "\n")
        return p

    @classmethod
    def load(cls, path=DEFAULT_INTRINSICS) -> "CameraModel":
        return cls.from_dict(json.loads(Path(path).read_text()))


def nominal_camera(size=(1280, 720)) -> CameraModel:
    """Guess for planning before calibration: equidistant fisheye, ~100 deg horizontal FOV."""
    w, h = size
    f = 0.57 * w
    return CameraModel("fisheye", [[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], np.zeros(4), size,
                       meta={"nominal": True})


def load_handeye(path=DEFAULT_HANDEYE) -> np.ndarray | None:
    p = Path(path)
    if not p.exists():
        return None
    return np.array(json.loads(p.read_text())["T_tip_cam"], float)


def finger_dir_for_cam_axis(cam_axis, at_xyz, T_tip_cam=None) -> np.ndarray:
    """Finger (approach) direction, base frame, that makes the camera look along ``cam_axis``
    with the tip near ``at_xyz``. The camera is tilted from the fingers about the tip's y axis,
    which stays horizontal and tangential while j5 ~ 0, so: rotate ``cam_axis`` about the
    tangential axis by the mounting angle, choosing the sense that points the fingers outward.
    (Camera straight down -> fingers ~30 deg below horizontal, outward.)"""
    T = T_TIP_CAM_NOMINAL if T_tip_cam is None else np.asarray(T_tip_cam, float)
    a = np.asarray(cam_axis, float) / np.linalg.norm(cam_axis)
    ang = float(np.arccos(np.clip(T[:3, 2] @ np.array([0.0, 0.0, -1.0]), -1, 1)))
    r = np.array([at_xyz[0], at_xyz[1], 0.0], float)
    r = r / (np.linalg.norm(r) or 1.0)
    t = np.cross([0.0, 0.0, 1.0], r)

    def rot(v, k, th):  # Rodrigues
        return v * np.cos(th) + np.cross(k, v) * np.sin(th) + k * (k @ v) * (1 - np.cos(th))
    cands = [rot(a, t, ang), rot(a, t, -ang)]
    f = max(cands, key=lambda v: v @ r)
    return f / np.linalg.norm(f)


def cam_pose(q, T_tip_cam=None) -> np.ndarray:
    """T_base_cam for joint angles q."""
    from armctl import kin
    return kin.fk_vision(q) @ (T_TIP_CAM_NOMINAL if T_tip_cam is None else np.asarray(T_tip_cam, float))


def ray_plane(origin, dirs, plane_point, plane_normal) -> np.ndarray:
    """Intersect rays (origin (3,), dirs (N,3)) with a plane. Returns (N,3), NaN where the
    ray is parallel or the hit is behind the origin."""
    d = np.asarray(dirs, float).reshape(-1, 3)
    n = np.asarray(plane_normal, float)
    den = d @ n
    t = ((np.asarray(plane_point) - np.asarray(origin)) @ n) / np.where(np.abs(den) < 1e-12, np.nan, den)
    t = np.where(t > 0, t, np.nan)
    return np.asarray(origin)[None, :] + t[:, None] * d
