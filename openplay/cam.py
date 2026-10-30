"""openplay.cam: relay the wrist camera to the active player's phone (arcade Robot Eyes, Claw Cam).

Read-only: it only GETs JPEGs, it never moves anything. One fetch thread pulls frames from the
lab console (``$VISION_DASHBOARD_URL``, default http://127.0.0.1:8090/camera.jpg; the console owns
the camera) at up to ``fps``, shrinks them to ``width`` px for phone Wi-Fi and keeps the latest.
It runs only while someone asked for a frame in the last ``idle_s`` seconds, plus a probe every
``probe_s`` seconds so open play knows whether to offer the camera games at all (``ok``).

``source="test"`` draws a synthetic scene (a moving test card that follows the arm's j1/j4), so
rehearsals and screenshots work without the console or a camera.
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time
import urllib.request
from typing import Callable

log = logging.getLogger("openplay.cam")

CONSOLE_URL = os.environ.get("VISION_DASHBOARD_URL", "http://127.0.0.1:8090/camera.jpg")
SOURCES = ("console", "test", "off")


def _console_fetch(url: str, timeout: float = 2.0) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        data = r.read()
    if not data.startswith(b"\xff\xd8"):
        raise ValueError("not a JPEG")
    return data


def _shrink(jpeg: bytes, width: int, quality: int = 70) -> bytes:
    """Re-encode wider frames at ``width`` px (phones on event Wi-Fi). Falls back to the input."""
    try:
        import cv2
        import numpy as np

        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None or img.shape[1] <= width:
            return jpeg
        h = round(img.shape[0] * width / img.shape[1])
        ok, buf = cv2.imencode(".jpg", cv2.resize(img, (width, h), interpolation=cv2.INTER_AREA),
                               [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes() if ok else jpeg
    except Exception:
        return jpeg


def synthetic_frame(q: list[float] | None, t: float, width: int = 640) -> bytes:
    """A synthetic 'camera' frame: a horizon and a few coloured blobs that slide as j1/j4 turn."""
    import cv2
    import numpy as np

    h = round(width * 9 / 16)
    j1 = (q or [0.0] * 6)[0]
    j4 = (q or [0.0] * 6)[3]
    img = np.zeros((h, width, 3), np.uint8)
    horizon = int(h * 0.55 + j4 * h * 0.8)
    img[:max(0, min(h, horizon))] = (70, 40, 30)           # back wall (BGR)
    img[max(0, min(h, horizon)):] = (60, 90, 120)          # table
    f = width / 1.2                                        # px per rad, ~70 deg across
    for i, (ang, col, r) in enumerate([(-0.5, (60, 60, 230), 34), (0.0, (80, 200, 80), 28),
                                       (0.45, (230, 120, 60), 40), (0.9, (40, 210, 240), 24)]):
        x = int(width / 2 + (j1 - ang) * f)
        y = horizon + 30 + 20 * i + int(6 * math.sin(t * 2 + i))
        cv2.circle(img, (x, y), r, col, -1)
    for i, ang in enumerate((-0.35, 0.1, 0.55)):          # an "audience" on the back wall
        cx, cy = int(width / 2 + (j1 - ang) * f), horizon - 70 - 25 * (i % 2)
        cv2.circle(img, (cx, cy), 34, (120, 200, 250), -1)
        for ex in (-12, 12):
            cv2.circle(img, (cx + ex, cy - 8), 5, (40, 30, 30), -1)
        cv2.ellipse(img, (cx, cy + 8), (14, 9), 0, 0, 180, (40, 30, 30), 3)
    cv2.putText(img, "TEST CAMERA", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(img, time.strftime("%H:%M:%S"), (width - 120, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return buf.tobytes()


class CamRelay:
    def __init__(self, source: str = "console", url: str = CONSOLE_URL, fps: float = 10.0, width: int = 640,
                 idle_s: float = 3.0, probe_s: float = 15.0, q: Callable[[], list | None] | None = None,
                 on_change: Callable[[bool], None] | None = None, fetch: Callable[[], bytes] | None = None):
        if source not in SOURCES:
            raise ValueError(f"camera source must be one of {SOURCES}")
        self.source, self.url, self.fps, self.width = source, url, fps, width
        self.idle_s, self.probe_s = idle_s, probe_s
        self.q = q or (lambda: None)
        self.on_change = on_change
        self._fetch = fetch
        self.cond = threading.Condition()
        self.jpeg: bytes | None = None
        self.seq = 0
        self.t = 0.0
        self.ok = source == "test"
        self.error: str | None = None if source != "off" else "camera off (--cam off)"
        self.wanted_t = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ---- lifecycle
    def start(self) -> None:
        if self.source == "off" or self._thread:
            return
        self._thread = threading.Thread(target=self._run, name="openplay-cam", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self.cond:
            self.cond.notify_all()

    # ---- frames
    def grab(self) -> bytes:
        if self._fetch:
            return self._fetch()
        if self.source == "test":
            return synthetic_frame(self.q(), time.time(), self.width)
        return _shrink(_console_fetch(self.url), self.width)

    def want(self) -> None:
        self.wanted_t = time.monotonic()
        with self.cond:
            self.cond.notify_all()

    def latest(self, after: int = 0, wait_s: float = 1.0) -> tuple[bytes | None, int]:
        """The newest frame with seq > ``after`` (waits up to ``wait_s``), else the newest one."""
        self.want()
        end = time.monotonic() + wait_s
        with self.cond:
            while self.seq <= after and not self._stop.is_set():
                left = end - time.monotonic()
                if left <= 0:
                    break
                self.cond.wait(left)
            return self.jpeg, self.seq

    def status(self) -> dict:
        return {"source": self.source, "ok": self.ok, "error": self.error,
                "age_s": round(time.time() - self.t, 1) if self.t else None}

    def _set_ok(self, ok: bool, error: str | None) -> None:
        changed = ok != self.ok
        self.ok, self.error = ok, error
        if changed:
            log.info("camera %s%s", "available" if ok else "unavailable", f": {error}" if error else "")
            if self.on_change:
                try:
                    self.on_change(ok)
                except Exception as e:  # never let a callback kill the relay
                    log.warning("camera on_change: %s", e)

    def _run(self) -> None:
        last_probe = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            active = now - self.wanted_t < self.idle_s
            if not active and now - last_probe < self.probe_s:
                with self.cond:
                    self.cond.wait(0.5)
                continue
            last_probe = now
            t0 = time.monotonic()
            try:
                jpeg = self.grab()
                with self.cond:
                    self.jpeg, self.seq, self.t = jpeg, self.seq + 1, time.time()
                    self.cond.notify_all()
                self._set_ok(True, None)
            except Exception as e:
                self._set_ok(False, f"camera unavailable ({type(e).__name__}: {e})"[:160])
                time.sleep(1.0)
            if active:
                time.sleep(max(0.0, 1.0 / self.fps - (time.monotonic() - t0)))
