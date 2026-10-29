"""interfaces.follow.track: frame-rate tracking of one object in the wrist camera.

Two rates, because the accurate finders (``interfaces.wristcam.locate``, ``target.SeededFinder``)
take 0.4-1.8 s a frame, and a camera that moves while it looks needs an answer every frame:

    fast   ``PatchTracker``: resample a small patch of each frame onto the plane at the object's
           top height (the top face is then the same size from any camera distance, whatever the
           fisheye does), NCC-match the top-face template from the last anchor, sub-pixel peak.
           A few ms. Gated: a weak or far match is a miss, never a jump to another object.
    slow   ``AnchorWorker``: the accurate finder, near the current estimate, in a thread. It gives
           the absolute position, refreshes the template and catches drift.

``KalmanCV`` (constant velocity, table xy) fuses both with time stamps (the frame's capture
time, not its arrival), so ``Tracker.estimate(t)`` can be asked for a time in the future: the
servo asks for "now + command latency" and leads a moving target instead of lagging it.

Every view must be built from the joint angles *at the frame's capture time* (``QHistory``):
the arm moves while it tracks, and a q 100 ms late puts the object a centimetre off.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

from interfaces.wristcam.locate import Detection, Hint, ObjectFinder, View


# --------------------------------------------------------------------------- filter
class KalmanCV:
    """Constant-velocity Kalman filter on a 2D point. State [x, y, vx, vy]; ``acc_sigma`` (m/s^2)
    is how hard the object may accelerate (process noise, white acceleration)."""

    def __init__(self, xy, t: float, sigma: float = 0.003, acc_sigma: float = 0.6, v_sigma: float = 0.05):
        self.x = np.array([xy[0], xy[1], 0.0, 0.0], float)
        self.P = np.diag([sigma**2, sigma**2, v_sigma**2, v_sigma**2])
        self.t = float(t)
        self.acc_sigma = acc_sigma

    def _FQ(self, dt: float):
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.acc_sigma**2
        a, b, c = dt**4 / 4, dt**3 / 2, dt**2
        Q = q * np.array([[a, 0, b, 0], [0, a, 0, b], [b, 0, c, 0], [0, b, 0, c]])
        return F, Q

    def predict(self, t: float) -> None:
        dt = float(t) - self.t
        if dt <= 0:
            return
        F, Q = self._FQ(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t = float(t)

    def at(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(xy, v, 2x2 position covariance) at time t, without changing the filter. A time
        before the filter's is extrapolated back along the current velocity (comparing a late
        anchor with the track needs where the track was *then*, not where it is now)."""
        dt = float(t) - self.t
        F, Q = self._FQ(abs(dt))
        F[0, 2] = F[1, 3] = dt
        x = F @ self.x
        P = F @ self.P @ F.T + Q
        return x[:2].copy(), x[2:].copy(), P[:2, :2].copy()

    def update(self, xy, sigma: float, t: float) -> float:
        """Measurement of the position at time t. A measurement older than the filter (a slow
        anchor delivered late) is moved forward by the current velocity, with its uncertainty
        grown to match. Returns the Mahalanobis distance of the innovation."""
        z = np.asarray(xy, float)
        R = np.eye(2) * sigma**2
        if t < self.t:
            lag = self.t - t
            z = z + self.x[2:] * lag
            R = R + self.P[2:, 2:] * lag**2
        else:
            self.predict(t)
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        S = H @ self.P @ H.T + R
        y = z - H @ self.x
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        return float(np.sqrt(y @ np.linalg.solve(S, y)))


# --------------------------------------------------------------------------- q at capture time
class QHistory:
    """Joint angles over time, so a frame can be paired with the q at its capture time."""

    def __init__(self, keep_s: float = 2.0):
        self.buf: deque = deque()
        self.keep_s = keep_s
        self._lock = threading.Lock()

    def add(self, t: float, q) -> None:
        q = np.asarray(q, float)
        if not np.isfinite(q).all():
            return
        with self._lock:
            if self.buf and t <= self.buf[-1][0]:
                return
            self.buf.append((float(t), q.copy()))
            while self.buf and self.buf[0][0] < t - self.keep_s:
                self.buf.popleft()

    def at(self, t: float) -> np.ndarray | None:
        """q interpolated at time t (clamped to the oldest / newest sample); None if empty."""
        with self._lock:
            if not self.buf:
                return None
            if t <= self.buf[0][0]:
                return self.buf[0][1].copy()
            if t >= self.buf[-1][0]:
                return self.buf[-1][1].copy()
            ts = [s[0] for s in self.buf]
            i = int(np.searchsorted(ts, t))
            (t0, q0), (t1, q1) = self.buf[i - 1], self.buf[i]
            s = (t - t0) / (t1 - t0)
            return q0 + s * (q1 - q0)

    @property
    def newest_t(self) -> float | None:
        with self._lock:
            return self.buf[-1][0] if self.buf else None


# --------------------------------------------------------------------------- fast tracker
@dataclass
class PatchMatch:
    xy: np.ndarray            # table xy of the top-face centre
    score: float              # NCC with the live template
    anchor_score: float       # NCC with the anchor's template (drift check)
    ms: float                 # time spent


class PatchTracker:
    """Template NCC on the plane at height ``z`` (table frame).

    ``set_template(frame, view, xy)`` cuts a square of side ``2 * half`` around the object's
    top-face centre at ``res`` m per pixel. ``match(frame, view, xy_pred, search)`` looks for it
    within ``search`` m of the prediction. The live template follows slow changes in look
    (lighting, the camera's angle onto the rim) by blending in good matches; the anchor template
    stays fixed until the next anchor, and a match must still resemble it (``min_anchor``)."""

    def __init__(self, z: float, half: float, res: float = 0.001, blend: float = 0.15,
                 min_score: float = 0.55, min_anchor: float = 0.35):
        self.z, self.half, self.res = float(z), float(half), float(res)
        self.blend, self.min_score, self.min_anchor = blend, min_score, min_anchor
        self.template: np.ndarray | None = None
        self.anchor: np.ndarray | None = None
        self._masks: dict = {}

    def _mask(self, view: View, shape) -> np.ndarray:
        key = (shape[1], shape[0])
        if key not in self._masks:
            self._masks[key] = view.cam.for_image(np.empty(shape, np.uint8)).jaw_mask()
        return self._masks[key]

    def _patch(self, frame, view: View, xy, half: float):
        """(gray float32, valid bool, grid) of the plane patch, or (None, None, None)."""
        o, valid, grid = view.ortho(frame, self.z, np.asarray(xy, float), half, res=self.res,
                                    mask=self._mask(view, frame.shape[:2]))
        if o is None:
            return None, None, None
        g = cv2.cvtColor(o, cv2.COLOR_BGR2GRAY).astype(np.float32) if o.ndim == 3 else o.astype(np.float32)
        return g, valid > 0, grid

    def set_template(self, frame, view: View, xy) -> bool:
        g, valid, _ = self._patch(frame, view, xy, self.half)
        if g is None or valid.mean() < 0.6:
            return False
        g = g.copy()
        g[~valid] = float(np.median(g[valid]))
        if g.std() < 3.0:            # a featureless patch can't be tracked
            return False
        self.template = g
        self.anchor = g.copy()
        return True

    @staticmethod
    def _subpixel(r, loc):
        x, y = loc
        dx = dy = 0.0
        if 0 < x < r.shape[1] - 1:
            l, c, rr = r[y, x - 1], r[y, x], r[y, x + 1]
            den = l - 2 * c + rr
            dx = 0.5 * (l - rr) / den if abs(den) > 1e-9 else 0.0
        if 0 < y < r.shape[0] - 1:
            u, c, d = r[y - 1, x], r[y, x], r[y + 1, x]
            den = u - 2 * c + d
            dy = 0.5 * (u - d) / den if abs(den) > 1e-9 else 0.0
        return x + float(np.clip(dx, -1, 1)), y + float(np.clip(dy, -1, 1))

    def match(self, frame, view: View, xy_pred, search: float) -> PatchMatch | None:
        if self.template is None:
            return None
        t0 = time.perf_counter()
        g, valid, grid = self._patch(frame, view, xy_pred, self.half + search)
        if g is None or valid.mean() < 0.3:
            return None
        T = self.template
        if g.shape[0] < T.shape[0] or g.shape[1] < T.shape[1]:
            return None
        g = g.copy()
        g[~valid] = float(np.median(g[valid])) if valid.any() else 0.0
        r = cv2.matchTemplate(g, T, cv2.TM_CCOEFF_NORMED)
        _, score, _, loc = cv2.minMaxLoc(r)
        u, v = self._subpixel(r, loc)
        cu, cv_ = u + T.shape[1] / 2, v + T.shape[0] / 2
        xy = np.array([grid["x0"] + cu * self.res, grid["y1"] - cv_ * self.res])
        # the anchor template, at the integer peak (cheap: one window)
        y0, x0 = loc[1], loc[0]
        win = g[y0:y0 + T.shape[0], x0:x0 + T.shape[1]]
        a_score = float(cv2.matchTemplate(win, self.anchor, cv2.TM_CCOEFF_NORMED)[0, 0]) \
            if self.anchor is not None and win.shape == self.anchor.shape else 0.0
        ms = (time.perf_counter() - t0) * 1000
        m = PatchMatch(xy, float(score), a_score, ms)
        if score >= max(self.min_score, 0.8) and a_score >= self.min_anchor:
            self.template = (1 - self.blend) * self.template + self.blend * win
        return m


# --------------------------------------------------------------------------- slow anchor
class AnchorWorker:
    """Runs an accurate (slow) finder near the estimate in a background thread.

    ``submit(frame, view, t, hint)`` starts one look if the worker is idle (returns False if busy);
    ``poll()`` returns ``(t, Detection | None, frame, view)`` once, when a look has finished. With
    ``threaded=False`` the look runs inside ``submit`` (simulation, tests)."""

    def __init__(self, finder: ObjectFinder, threaded: bool = True):
        self.finder, self.threaded = finder, threaded
        self._busy = False
        self._result = None
        self._lock = threading.Lock()
        self.n = 0
        self.last_ms = 0.0

    @property
    def busy(self) -> bool:
        return self._busy

    def _work(self, frame, view, t, hint):
        t0 = time.perf_counter()
        try:
            det = self.finder.find(frame, view, hint)
        except Exception:      # a finder bug must not kill the tracker: report "not found"
            det = None
        self.last_ms = (time.perf_counter() - t0) * 1000
        with self._lock:
            self._result = (t, det, frame, view)
            self._busy = False
            self.n += 1

    def submit(self, frame, view: View, t: float, hint: Hint | None) -> bool:
        with self._lock:
            if self._busy:
                return False
            self._busy = True
        if self.threaded:
            threading.Thread(target=self._work, args=(frame, view, t, hint), daemon=True).start()
        else:
            self._work(frame, view, t, hint)
        return True

    def poll(self):
        with self._lock:
            r, self._result = self._result, None
        return r

    def join(self, timeout: float = 5.0) -> None:
        """Wait for a running look to finish (call before exiting: a thread killed inside OpenCV
        at interpreter exit aborts the process)."""
        end = time.monotonic() + timeout
        while self._busy and time.monotonic() < end:
            time.sleep(0.01)


# --------------------------------------------------------------------------- the tracker
@dataclass
class TrackConfig:
    height: float = 0.08          # m, object top above the table (the tracking plane)
    size: float = 0.07            # m, object width across its top (template = size * template_scale)
    template_scale: float = 1.3
    res: float = 0.001            # m per template pixel
    search: float = 0.03          # m, search radius around the prediction (plus speed * age)
    min_score: float = 0.55
    sigma_px: float = 1.5         # template pixels: a good match's 1-sigma
    acc_sigma: float = 0.6        # m/s^2, how hard the object may accelerate
    coast_s: float = 0.3          # s without a match (and >= 2 missed frames) before the track counts as lost
    lost_s: float = 1.0           # s without a match at all: lost, frames or not
    anchor_gate: float = 0.03     # m: an anchor further than this from the estimate...
    anchor_disagree_n: int = 2    # ...this many times in a row resets the track to it
    anchor_period: float = 1.0    # s between anchor looks while tracking
    v_max: float = 0.25           # m/s: faster than this is a tracking error, not the object
    lost_radius: float = 0.12     # m, the widest re-search around the last position when lost
    grace_speed: float = 0.05     # m/s the object may have moved while a new track's anchor was computed


@dataclass
class TrackEstimate:
    xy: np.ndarray                # table xy (top-face centre) at the asked time
    v: np.ndarray                 # m/s, table frame
    sigma: float                  # m, 1-sigma (largest axis)
    age: float                    # s since the last accepted measurement
    state: str                    # tracking | coasting


class Tracker:
    """One object: anchored by a slow finder, tracked by ``PatchTracker``, fused by ``KalmanCV``.

    States: ``searching`` (no fix yet, or lost), ``tracking`` (a recent match), ``coasting`` (no
    match for less than ``coast_s``; the filter's prediction is still trusted), ``lost``.
    ``estimate(t)`` is None unless tracking or coasting."""

    def __init__(self, cfg: TrackConfig | None = None):
        self.cfg = cfg or TrackConfig()
        self.kf: KalmanCV | None = None
        self.patch = PatchTracker(self.cfg.height, self.cfg.size * self.cfg.template_scale / 2,
                                  self.cfg.res, min_score=self.cfg.min_score)
        self.state = "searching"
        self.t_last = -1e9            # capture time of the last accepted measurement
        self.t_frame = -1e9           # capture time of the newest frame seen (the tracker's clock)
        self.grace_t: float | None = None   # a new track: capture time of its first fast frame
        self.grace = False
        self.misses_since = 0          # frames looked at without a match since the last match
        self.t_anchor = -1e9
        self.disagree = 0
        self.stats = {"frames": 0, "matches": 0, "misses": 0, "anchors": 0, "anchor_misses": 0,
                      "resets": 0, "lost": 0, "match_ms": []}
        self.last: dict = {}

    # -------------------------------------------------------------- anchors
    def anchor(self, det: Detection | None, frame, view: View, t: float) -> str:
        """Feed one slow-finder result (taken from ``frame`` at capture time ``t``). Returns what
        it did: started | refreshed | reset | ignored | missed."""
        self.t_anchor = t
        if det is None:
            self.stats["anchor_misses"] += 1
            return "missed"
        self.stats["anchors"] += 1
        xy = np.asarray(det.xyz_table[:2], float)
        sigma = max(float(det.sigma_xy), 0.001)
        if self.kf is None or self.state in ("searching", "lost"):
            if not self.patch.set_template(frame, view, xy):
                return "ignored"
            self._start(xy, sigma, t)
            return "started"
        est, _, _ = self.kf.at(t)
        if np.linalg.norm(est - xy) > self.cfg.anchor_gate:
            self.disagree += 1
            if self.disagree < self.cfg.anchor_disagree_n:
                return "ignored"
            self.stats["resets"] += 1
            self.patch.set_template(frame, view, xy)
            self._start(xy, sigma, t)
            return "reset"
        self.disagree = 0
        self.kf.update(xy, sigma, t)
        # cut the new template at the anchor's own position, not the fused estimate: a template
        # cut off-centre would bias every later match by the same offset (it drifted 2-3 cm)
        self.patch.set_template(frame, view, xy)
        self.t_last = max(self.t_last, t)
        if self.state == "coasting":
            self.state = "tracking"
        return "refreshed"

    def _start(self, xy, sigma: float, t: float) -> None:
        """A new track from an anchor taken at capture time t. The slow look finished well after
        its frame (0.4-2.5 s), so the track gets a grace period: it is lost only after
        ``coast_s`` without a match counted from its first fast frame, and that frame searches a
        window grown by how far the object may have gone since t."""
        self.kf = KalmanCV(xy, t, sigma=sigma, acc_sigma=self.cfg.acc_sigma)
        self.state, self.t_last, self.disagree = "tracking", t, 0
        self.grace, self.grace_t, self.misses_since = True, None, 0

    def wants_anchor(self, t: float) -> bool:
        return self.state in ("searching", "lost") or t - self.t_anchor >= self.cfg.anchor_period

    def hint(self, t: float) -> Hint | None:
        """Where the slow finder should look. Tracking: tight around the estimate. Lost: around the
        last position, widening with the time lost (someone moved it), never the whole frame, so an
        identical object elsewhere isn't taken for it. No track yet: None (the caller's seed hint)."""
        if self.kf is None:
            return None
        if self.state in ("searching", "lost"):
            gone = max(0.0, t - self.t_last)
            return Hint(tuple(float(x) for x in self.kf.x[:2]), radius=min(0.04 + 0.05 * gone, self.cfg.lost_radius))
        xy, v, P = self.kf.at(t)
        # tight: a second, identical object a few cm away must not be what the anchor finds
        r = 0.02 + 3 * float(np.sqrt(np.linalg.eigvalsh(P).max())) + float(np.linalg.norm(v)) * 0.5
        return Hint(tuple(float(x) for x in xy), radius=min(r, 0.045))

    # -------------------------------------------------------------- fast updates
    def update(self, frame, view: View, t: float) -> PatchMatch | None:
        """One frame captured at time t (``view`` built from q at t). Returns the accepted match
        or None (a miss, or nothing to track yet)."""
        self.stats["frames"] += 1
        self.t_frame = max(self.t_frame, t)
        if self.grace and self.grace_t is None:
            self.grace_t = t
        self._age(t)
        if self.kf is None or self.state in ("searching", "lost"):
            return None
        xy_pred, v, P = self.kf.at(t)
        age = max(0.0, t - self.t_last)
        speed = self.cfg.grace_speed if self.grace else 0.05
        search = self.cfg.search + min(self.cfg.v_max, float(np.linalg.norm(v)) + speed) * min(age, 1.0)
        m = self.patch.match(frame, view, xy_pred, search)
        self.last = {"t": t, "pred": xy_pred, "match": None if m is None else m.xy, "score": None if m is None else m.score}
        if m is not None:
            self.stats["match_ms"].append(m.ms)
        ok = m is not None and m.score >= self.cfg.min_score and m.anchor_score >= self.patch.min_anchor
        if ok:
            # no Mahalanobis gate: a cup that starts moving would fail it for several frames. The
            # search window bounds the jump, and the anchor template guards the identity.
            self.kf.update(m.xy, self.cfg.sigma_px * self.cfg.res / max(m.score, 0.3), t)
        if not ok:
            self.stats["misses"] += 1
            self.misses_since += 1
            self._age(t)
            return None
        self.misses_since = 0
        self.stats["matches"] += 1
        self.t_last = t
        self.state = "tracking"
        self.grace = False
        self.misses_since = 0          # frames looked at without a match since the last match
        return m

    def _age(self, t: float) -> None:
        """Lost = no match for ``coast_s`` *and* at least two frames looked at in that time that
        didn't match (a gap in the frames themselves is not evidence that the object went; the
        caller holds the arm on stale frames), or no match at all for ``lost_s``."""
        if self.state in ("tracking", "coasting"):
            if self.grace and self.grace_t is None:
                return
            age = t - (self.grace_t if self.grace else self.t_last)
            if (age > self.cfg.coast_s and self.misses_since >= 2) or age > self.cfg.lost_s:
                self.state = "lost"
                self.stats["lost"] += 1
            elif age > 0.12:
                self.state = "coasting"

    def estimate(self, t: float) -> TrackEstimate | None:
        """Where the object is (or will be) at time t. None when there is no live track. The
        track's state ages on frame capture times only (``update``), so asking about the future
        (the servo's lead) doesn't change it; staleness of the frames themselves is the caller's
        check."""
        if self.kf is None or self.state not in ("tracking", "coasting"):
            return None
        xy, v, P = self.kf.at(t)
        sp = float(np.linalg.norm(v))
        if sp > self.cfg.v_max:
            v = v * self.cfg.v_max / sp
        return TrackEstimate(xy, v, float(np.sqrt(np.linalg.eigvalsh(P).max())), max(0.0, self.t_frame - self.t_last),
                             self.state)

    def summary(self) -> dict:
        s = dict(self.stats)
        ms = s.pop("match_ms")
        s["match_ms_median"] = round(float(np.median(ms)), 2) if ms else None
        s["match_ms_p95"] = round(float(np.percentile(ms, 95)), 2) if ms else None
        return s
