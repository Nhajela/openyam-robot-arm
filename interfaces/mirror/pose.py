"""interfaces.mirror.pose: human body landmarks from a camera frame (MediaPipe Pose Landmarker).

``PoseTracker(model).detect(frame_bgr, t)`` -> ``Body`` (or None when nobody is seen) with
3D "world" landmarks in metres (MediaPipe's hip-centred frame: x to the image right, y down,
z away from the camera) plus normalised image landmarks for drawing.

The model file is downloaded once to ``~/.cache/openyam/`` (5.8 MB for "lite").
``SyntheticBody`` makes fake bodies (a person waving) so the whole pipeline can run with no
camera and no model.
"""
from __future__ import annotations

import math
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MODEL_URLS = {
    v: f"https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_{v}/float16/latest/pose_landmarker_{v}.task"
    for v in ("lite", "full", "heavy")
}
CACHE = Path.home() / ".cache/openyam"

# MediaPipe pose landmark indices (33-point BlazePose topology).
NOSE = 0
L_SHOULDER, R_SHOULDER = 11, 12
L_ELBOW, R_ELBOW = 13, 14
L_WRIST, R_WRIST = 15, 16
L_PINKY, R_PINKY = 17, 18
L_INDEX, R_INDEX = 19, 20
L_THUMB, R_THUMB = 21, 22
L_HIP, R_HIP = 23, 24
SIDE = {
    "right": dict(shoulder=R_SHOULDER, elbow=R_ELBOW, wrist=R_WRIST, index=R_INDEX, pinky=R_PINKY, thumb=R_THUMB, hip=R_HIP),
    "left": dict(shoulder=L_SHOULDER, elbow=L_ELBOW, wrist=L_WRIST, index=L_INDEX, pinky=L_PINKY, thumb=L_THUMB, hip=L_HIP),
}
CONNECTIONS = [(11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24), (23, 24),
               (15, 19), (16, 20), (23, 25), (24, 26), (25, 27), (26, 28)]


@dataclass
class Body:
    world: np.ndarray        # (33, 3) metres, x right, y down, z away from camera
    image: np.ndarray        # (33, 2) normalised image coords 0..1
    visibility: np.ndarray   # (33,) 0..1
    t: float

    def vis(self, *idx: int) -> float:
        return float(min(self.visibility[i] for i in idx))


def model_path(variant: str = "lite") -> Path:
    p = CACHE / f"pose_landmarker_{variant}.task"
    if not p.exists():
        CACHE.mkdir(parents=True, exist_ok=True)
        print(f"downloading {MODEL_URLS[variant]} -> {p}")
        urllib.request.urlretrieve(MODEL_URLS[variant], p)
    return p


class PoseTracker:
    """MediaPipe Pose Landmarker in VIDEO mode (tracks between frames, faster than per-image)."""

    def __init__(self, variant: str = "lite", video: bool = True, min_conf: float = 0.5):
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision

        self._mp = mp
        mode = vision.RunningMode.VIDEO if video else vision.RunningMode.IMAGE
        opts = vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model_path(variant))),
            running_mode=mode, num_poses=1,
            min_pose_detection_confidence=min_conf, min_pose_presence_confidence=min_conf,
            min_tracking_confidence=min_conf)
        self.video = video
        self.lm = vision.PoseLandmarker.create_from_options(opts)
        self._last_ms = -1

    def detect(self, frame_bgr: np.ndarray, t: float) -> Body | None:
        rgb = np.ascontiguousarray(frame_bgr[:, :, ::-1])
        img = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        if self.video:
            ms = max(int(t * 1000), self._last_ms + 1)  # must strictly increase
            self._last_ms = ms
            res = self.lm.detect_for_video(img, ms)
        else:
            res = self.lm.detect(img)
        if not res.pose_world_landmarks:
            return None
        w = res.pose_world_landmarks[0]
        n = res.pose_landmarks[0]
        return Body(
            world=np.array([[p.x, p.y, p.z] for p in w], float),
            image=np.array([[p.x, p.y] for p in n], float),
            visibility=np.array([p.visibility if p.visibility is not None else 1.0 for p in n], float),
            t=t)

    def close(self) -> None:
        self.lm.close()


class SyntheticBody:
    """A stick person facing the camera who raises the right arm, waves, and points around.

    Landmarks are in MediaPipe's world convention. Use it to run the pipeline (and the sim)
    with no camera: ``--source synthetic``.
    """

    def __init__(self, period: float = 8.0):
        self.period = period

    def __call__(self, t: float) -> Body:
        P = np.zeros((33, 3))
        P[L_HIP] = (0.10, 0.0, 0.0)          # person's left = image right (+x) when facing the camera
        P[R_HIP] = (-0.10, 0.0, 0.0)
        P[L_SHOULDER] = (0.18, -0.50, 0.0)
        P[R_SHOULDER] = (-0.18, -0.50, 0.0)
        P[NOSE] = (0.0, -0.68, -0.08)
        # left arm hangs
        P[L_ELBOW] = P[L_SHOULDER] + (0.02, 0.28, 0.0)
        P[L_WRIST] = P[L_ELBOW] + (0.0, 0.26, -0.03)
        # right arm: elevation sweeps 20..160 deg, swings fore/aft, elbow bends and straightens
        s = 2 * math.pi * t / self.period
        elev = math.radians(90 + 70 * math.sin(s))
        az = math.radians(30 * math.sin(0.5 * s))           # 0 = straight out to the side
        bend = math.radians(45 + 40 * math.sin(2 * s))      # elbow flexion
        out = np.array([-1.0, 0.0, 0.0])                    # person's right, image left
        down, fwd = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, -1.0])  # forward = toward camera
        horiz = math.cos(az) * out + math.sin(az) * fwd
        u = math.sin(elev) * horiz + math.cos(elev) * down
        P[R_ELBOW] = P[R_SHOULDER] + 0.28 * u
        axis = np.cross(u, fwd if abs(np.dot(u, fwd)) < 0.9 else out)
        axis /= np.linalg.norm(axis)
        f = _rotate(u, axis, bend)
        P[R_WRIST] = P[R_ELBOW] + 0.26 * f
        P[R_INDEX] = P[R_WRIST] + 0.08 * f
        P[R_PINKY] = P[R_WRIST] + 0.07 * f + 0.02 * axis
        P[R_THUMB] = P[R_WRIST] + 0.04 * f - 0.02 * axis
        P[L_INDEX] = P[L_WRIST] + (0, 0.08, 0)
        img = np.c_[0.5 + P[:, 0] * 0.8, 0.75 + P[:, 1] * 0.8]
        return Body(world=P, image=img, visibility=np.ones(33), t=t)


def _rotate(v: np.ndarray, axis: np.ndarray, a: float) -> np.ndarray:
    return v * math.cos(a) + np.cross(axis, v) * math.sin(a) + axis * np.dot(axis, v) * (1 - math.cos(a))


def draw(frame: np.ndarray, body: Body | None, text: list[str] | None = None, side: str = "right") -> np.ndarray:
    """Overlay the skeleton (tracked arm highlighted) and a few lines of text."""
    import cv2

    out = frame.copy()
    h, w = out.shape[:2]
    if body is not None:
        pts = (body.image * [w, h]).astype(int)
        hot = {(SIDE[side]["shoulder"], SIDE[side]["elbow"]), (SIDE[side]["elbow"], SIDE[side]["wrist"])}
        for a, b in CONNECTIONS:
            c = (0, 200, 255) if (a, b) in hot else (200, 200, 200)
            cv2.line(out, tuple(pts[a]), tuple(pts[b]), c, 4 if (a, b) in hot else 2)
        for i in (SIDE[side]["shoulder"], SIDE[side]["elbow"], SIDE[side]["wrist"]):
            cv2.circle(out, tuple(pts[i]), 7, (0, 140, 255), -1)
    for k, line in enumerate(text or []):
        cv2.putText(out, line, (12, 28 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(out, line, (12, 28 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return out
