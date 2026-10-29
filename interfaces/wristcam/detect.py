"""interfaces.wristcam.detect: open-vocabulary 2D detection, one seam for every detector.

"Pick up the blue marker" needs a mask of *whatever* was named, not one hand-built finder per
object. Detection is a solved problem, so this module only adapts existing detectors to one
type (``Det2D``: box, mask, label, score, track id) and leaves the geometry (mask -> footprint,
height, grasp) to ``target.target_from_mask``.

    YoloeDetector      YOLO-E (open-vocabulary detect + segment, text prompts or prompt-free),
                       the model DimOS ships (``dimos.perception.detection.detectors.yoloe``);
                       its weights come through DimOS's ``get_data`` when DimOS is installed, else
                       straight from Ultralytics. GPU when there is one (the lab laptop), CPU otherwise.
    ClaudeBoxDetector  Claude vision (``target.ClaudeLocator``): prompt -> box, then a GrabCut mask
                       seeded by that box. Slow (seconds) and costs money: the fallback for prompts
                       YOLO-E can't ground ("the thing Naman drew on").
    SynthDetector      ground truth from the synthetic world (``synth.World``): tests only.

    detector = make_detector("yoloe")                  # or "claude"
    dets = detector.detect(frame, ["cup", "marker"])   # prompts=None: everything it can see
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import cv2
import numpy as np


@dataclass
class Det2D:
    box: tuple                   # (u0, v0, u1, v1) full-frame px
    mask: np.ndarray             # uint8 HxW, 255 = object
    label: str
    score: float
    track_id: int = -1
    extra: dict = field(default_factory=dict)

    @property
    def centre(self) -> np.ndarray:
        ys, xs = np.nonzero(self.mask)
        if len(xs):
            return np.array([xs.mean(), ys.mean()])
        u0, v0, u1, v1 = self.box
        return np.array([(u0 + u1) / 2, (v0 + v1) / 2])

    @property
    def area(self) -> int:
        return int(np.count_nonzero(self.mask))

    def to_dict(self) -> dict:
        return {"box": [round(float(v), 1) for v in self.box], "label": self.label, "score": round(float(self.score), 3),
                "track_id": self.track_id, "area_px": self.area, **self.extra}


@runtime_checkable
class Detector(Protocol):
    name: str

    def detect(self, frame: np.ndarray, prompts: list[str] | None = None) -> list[Det2D]: ...


def masks_from_polygons(polys, shape) -> list[np.ndarray]:
    """Ultralytics ``masks.xy`` (polygons in original-image px) -> full-frame uint8 masks. The
    polygons are already in frame coordinates; resizing ``masks.data`` instead ignores the
    letterbox padding (a 1280x720 frame runs at 640x384, not 640x360)."""
    out = []
    for p in polys:
        m = np.zeros(shape[:2], np.uint8)
        p = np.asarray(p, np.float32)
        if len(p) >= 3:
            cv2.fillPoly(m, [np.round(p).astype(np.int32)], 255)
        out.append(m)
    return out


def from_ultralytics(result, shape, keep_mask_px: int = 30) -> list[Det2D]:
    """One Ultralytics ``Results`` -> Det2D list (boxes, polygons -> masks, names, track ids)."""
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return []

    def arr(x):
        return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)

    xyxy, conf, cls = arr(boxes.xyxy), arr(boxes.conf), arr(boxes.cls).astype(int)
    ids = arr(boxes.id).astype(int) if getattr(boxes, "id", None) is not None else np.full(len(xyxy), -1)
    masks = getattr(result, "masks", None)
    polys = list(masks.xy) if masks is not None and getattr(masks, "xy", None) is not None else []
    mlist = masks_from_polygons(polys, shape) if polys else []
    names = getattr(result, "names", {}) or {}
    out = []
    for i in range(len(xyxy)):
        if i < len(mlist):
            m = mlist[i]
        else:                           # boxes only: the box as the mask (coarse)
            m = np.zeros(shape[:2], np.uint8)
            u0, v0, u1, v1 = np.round(xyxy[i]).astype(int)
            m[max(v0, 0):max(v1, 0), max(u0, 0):max(u1, 0)] = 255
        if np.count_nonzero(m) < keep_mask_px:
            continue
        name = names.get(int(cls[i]), f"class_{cls[i]}") if isinstance(names, dict) else \
            (names[cls[i]] if cls[i] < len(names) else f"class_{cls[i]}")
        out.append(Det2D(tuple(float(v) for v in xyxy[i]), m, str(name), float(conf[i]), int(ids[i])))
    return out


# --------------------------------------------------------------------------- YOLO-E
class YoloeDetector:
    """YOLO-E through DimOS when it is installed (same weights and prompt handling as DimOS's
    perception stack), else Ultralytics directly. ``prompts=None`` uses the prompt-free model,
    which names everything it sees (its own vocabulary)."""

    name = "yoloe"

    def __init__(self, conf: float = 0.15, device: str | None = None, model: str | None = None,
                 imgsz: int = 640):
        self.conf, self.device, self.model_name, self.imgsz = conf, device, model, imgsz
        self._text = None        # model for text prompts
        self._free = None        # prompt-free model
        self._prompts: tuple | None = None
        self.backend = None
        self.last_ms = 0.0

    def _load(self, prompt_free: bool):
        try:
            from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
            mode = YoloePromptMode.LRPC if prompt_free else YoloePromptMode.PROMPT
            d = Yoloe2DDetector(prompt_mode=mode, model_name=self.model_name, device=self.device, conf=self.conf)
            self.backend = "dimos"
            self.device = d.device
            return d.model
        except ImportError:
            from ultralytics import YOLOE
            import torch
            self.backend = "ultralytics"
            self.device = self.device or ("cuda" if torch.cuda.is_available() else "cpu")
            return YOLOE(self.model_name or ("yoloe-11s-seg-pf.pt" if prompt_free else "yoloe-11s-seg.pt"))

    def detect(self, frame, prompts=None) -> list[Det2D]:
        t0 = time.perf_counter()
        if prompts:
            if self._text is None:
                self._text = self._load(False)
            p = tuple(prompts)
            if p != self._prompts:
                self._text.set_classes(list(p), self._text.get_text_pe(list(p)))
                self._prompts = p
            model = self._text
        else:
            if self._free is None:
                self._free = self._load(True)
            model = self._free
        res = model.predict(source=frame, device=self.device, conf=self.conf, iou=0.6, imgsz=self.imgsz, verbose=False)
        out = from_ultralytics(res[0], frame.shape)
        self.last_ms = (time.perf_counter() - t0) * 1000
        return out


# --------------------------------------------------------------------------- Claude (fallback)
class ClaudeBoxDetector:
    """Claude vision finds the named object's box; GrabCut inside the box makes the mask (a box
    seeds GrabCut far better than a single tap). One API call per detect: use for the first look,
    not for frame-rate tracking."""

    name = "claude"

    def __init__(self, locator=None):
        from .target import ClaudeLocator
        self.locator = locator or ClaudeLocator()
        self.last_ms = 0.0
        self.last_answer = None

    def detect(self, frame, prompts=None) -> list[Det2D]:
        if not prompts:
            raise ValueError("the Claude detector needs a prompt (what to look for)")
        t0 = time.perf_counter()
        out = []
        for p in prompts:
            ans = self.locator.locate(frame, p)
            self.last_answer = ans
            if not ans.found or ans.box is None:
                continue
            m = grabcut_in_box(frame, ans.box)
            if m is not None:
                out.append(Det2D(tuple(float(v) for v in ans.box), m, ans.label or p, float(ans.confidence),
                                 extra={"reason": ans.reason}))
        self.last_ms = (time.perf_counter() - t0) * 1000
        return out


def grabcut_in_box(frame, box, margin: int = 6, iters: int = 4) -> np.ndarray | None:
    h, w = frame.shape[:2]
    u0, v0, u1, v1 = (int(round(x)) for x in box)
    u0, v0 = max(u0 - margin, 0), max(v0 - margin, 0)
    u1, v1 = min(u1 + margin, w - 1), min(v1 + margin, h - 1)
    if u1 - u0 < 8 or v1 - v0 < 8:
        return None
    m = np.zeros((h, w), np.uint8)
    bg, fg = np.zeros((1, 65), np.float64), np.zeros((1, 65), np.float64)
    try:
        cv2.setRNGSeed(0)   # reproducible: GrabCut uses OpenCV's global RNG
        cv2.grabCut(frame, m, (u0, v0, u1 - u0, v1 - v0), bg, fg, iters, cv2.GC_INIT_WITH_RECT)
    except cv2.error:
        return None
    out = np.where((m == cv2.GC_FGD) | (m == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(out)
    if n <= 1:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return np.where(lab == k, 255, 0).astype(np.uint8)


# --------------------------------------------------------------------------- synthetic (tests)
class SynthDetector:
    """Ground-truth masks from ``synth.World``: an object's mask is where the frame changes when
    the object is removed (rendered without noise). Labels: each object's ``label`` attribute if
    set, else "cup" / "box". Needs the camera pose of the frame: call ``set_view(cam, T_base_cam)``
    before ``detect`` (``SimArm.frame`` does not pass it on)."""

    name = "synth"

    def __init__(self, world, labels: dict | None = None, min_px: int = 40, miss: set | None = None):
        self.world, self.labels, self.min_px = world, labels or {}, min_px
        self.miss = miss or set()       # labels it "can't see" (to test refusals)
        self.cam = self.T = None
        self.last_ms = 0.0

    def set_view(self, cam, T_base_cam) -> None:
        self.cam, self.T = cam, np.asarray(T_base_cam, float)

    def _label(self, obj) -> str:
        return self.labels.get(id(obj)) or getattr(obj, "label", None) or type(obj).__name__.lower()

    def detect(self, frame, prompts=None) -> list[Det2D]:
        if self.cam is None:
            raise RuntimeError("SynthDetector.set_view() first")
        t0 = time.perf_counter()
        cam = self.cam.for_image(frame)
        kw = dict(ss=1, noise=0.0, blur=0.0, jaws=True, jpeg_q=0)
        full = self.world.render(cam, self.T, **kw).astype(np.int16)
        out = []
        for kind in ("cups", "boxes"):
            objs = list(getattr(self.world, kind))
            for o in objs:
                lab = self._label(o)
                if lab in self.miss or (prompts and not any(_matches(p, lab) for p in prompts)):
                    continue
                setattr(self.world, kind, [x for x in objs if x is not o])
                try:
                    without = self.world.render(cam, self.T, **kw).astype(np.int16)
                finally:
                    setattr(self.world, kind, objs)
                m = (np.abs(full - without).sum(2) > 12).astype(np.uint8) * 255
                m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                if np.count_nonzero(m) < self.min_px:
                    continue
                ys, xs = np.nonzero(m)
                out.append(Det2D((float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())), m, lab, 0.9))
        self.last_ms = (time.perf_counter() - t0) * 1000
        return out


def _matches(prompt: str, label: str) -> bool:
    p, l = prompt.lower().strip(), label.lower().strip()
    return p == l or p in l or l in p or set(p.split()) & set(l.split()) == set(l.split())


def make_detector(kind: str, **kw) -> Detector:
    if kind == "yoloe":
        return YoloeDetector(**kw)
    if kind == "claude":
        return ClaudeBoxDetector(**kw)
    raise ValueError(f"unknown detector {kind!r} (yoloe, claude)")
