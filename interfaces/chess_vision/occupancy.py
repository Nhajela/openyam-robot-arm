"""Per-square occupancy and piece colour from one camera's top-down warp.

Everything is relative to that camera's **empty-board reference** (recorded at calibration), so
the board's own colours, wood grain and the camera's quirks cancel out:

1. **Lighting.** A gain field (per channel, linear across the board) is fitted from small patches
   at the 81 square corners, which pieces rarely cover, comparing the current warp with the
   reference (log ratio, trimmed least squares, so the few covered corners drop out). The current
   warp is divided by it.
2. **Sampling.** Each square is sampled in a disc. For an oblique camera the disc is shifted away
   from the camera (the "upper part" of the square in the image): a piece's body projects onto
   the board plane further from the camera, while the piece in front of the square leans into its
   near half. The shift is ``lean x z`` (``lean`` = horizontal distance / camera height, from the
   camera pose) capped at a quarter square, so the disc never reaches the square behind.
3. **Features** per square: mean colour difference to the reference, chromaticity difference
   (brightness-invariant) and the change in edge density of log-luminance (gradient of log
   intensity is invariant to a lighting gain).
4. **Calibrated probabilities.** A small L2-regularised logistic regression per camera, fitted on
   the start-position frame (32 known occupied, 32 known empty squares, including the empty ones
   that tall neighbours lean into). Colour: a second logistic on the robust luminance of the
   changed pixels, fitted on the 16 white and 16 black pieces of the start position.

The camera's fusion weight per square, ``view_w``, falls with ``lean`` (oblique views are less
trustworthy) and with the fraction of the disc that is outside the camera's view.
"""
from __future__ import annotations

from dataclasses import dataclass

import chess
import cv2
import numpy as np

from .board_geom import BoardSpec, Canon

LUMA = np.array([0.114, 0.587, 0.299], np.float32)   # BGR


# ---------------------------------------------------------------------- logistic
@dataclass
class Logistic:
    mu: np.ndarray
    sd: np.ndarray
    w: np.ndarray
    b: float

    @classmethod
    def fit(cls, X: np.ndarray, y: np.ndarray, l2: float = 1.0, iters: int = 60) -> "Logistic":
        X = np.asarray(X, float)
        y = np.asarray(y, float)
        mu, sd = X.mean(0), X.std(0) + 1e-6
        Z = np.c_[(X - mu) / sd, np.ones(len(X))]
        th = np.zeros(Z.shape[1])
        R = np.eye(Z.shape[1]) * l2
        R[-1, -1] = 1e-6
        for _ in range(iters):
            p = 1 / (1 + np.exp(-np.clip(Z @ th, -30, 30)))
            g = Z.T @ (p - y) + R @ th
            Hs = (Z * (p * (1 - p))[:, None]).T @ Z + R
            step = np.linalg.solve(Hs, g)
            th -= step
            if np.abs(step).max() < 1e-8:
                break
        return cls(mu, sd, th[:-1], float(th[-1]))

    def logit(self, X: np.ndarray) -> np.ndarray:
        return ((np.asarray(X, float) - self.mu) / self.sd) @ self.w + self.b

    def predict(self, X: np.ndarray) -> np.ndarray:
        return 1 / (1 + np.exp(-np.clip(self.logit(X), -30, 30)))

    def to_dict(self) -> dict:
        return {"mu": self.mu.tolist(), "sd": self.sd.tolist(), "w": self.w.tolist(), "b": self.b}

    @classmethod
    def from_dict(cls, d: dict) -> "Logistic":
        return cls(np.array(d["mu"]), np.array(d["sd"]), np.array(d["w"]), float(d["b"]))


# ---------------------------------------------------------------------- geometry of sampling
@dataclass
class Regions:
    centers: np.ndarray     # 64x2 warp px
    radius: float           # px
    lean: np.ndarray        # 64, horizontal distance / camera height at each square
    view_w: np.ndarray      # 64, 0..1


def sample_regions(canon: Canon, cam_pos: np.ndarray | None, z_sample_sq: float = 0.6,
                   max_shift_sq: float = 0.25, radius_sq: float = 0.24) -> Regions:
    s = canon.spec.square_mm
    centers_mm = np.array([canon.spec.square_center(i) for i in range(64)])
    if cam_pos is None or cam_pos[2] <= 1:
        lean_v = np.zeros((64, 2))
    else:
        lean_v = (centers_mm - cam_pos[:2]) / cam_pos[2]
    shift = lean_v * z_sample_sq * s
    n = np.linalg.norm(shift, axis=1, keepdims=True)
    shift = np.where(n > max_shift_sq * s, shift * (max_shift_sq * s) / np.maximum(n, 1e-9), shift)
    lean = np.linalg.norm(lean_v, axis=1)
    return Regions(canon.board_to_px(centers_mm + shift), radius_sq * canon.pps, lean,
                   1.0 / (1.0 + (lean / 0.6) ** 2))


def _disc(radius: float) -> np.ndarray:
    r = int(np.ceil(radius))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    keep = xx ** 2 + yy ** 2 <= radius ** 2
    return np.c_[yy[keep], xx[keep]]


def edge_map(img: np.ndarray) -> np.ndarray:
    L = np.log(img.astype(np.float32) @ LUMA + 6.0)
    L = cv2.GaussianBlur(L, (0, 0), 1.0)
    gx = cv2.Sobel(L, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(L, cv2.CV_32F, 0, 1, ksize=3)
    return np.sqrt(gx * gx + gy * gy)


def gain_field(cur: np.ndarray, ref: np.ndarray, valid: np.ndarray, canon: Canon) -> np.ndarray:
    """Multiplicative per-channel field G (size x size x 3) with cur ~= G * ref on uncovered board."""
    pts = canon.board_to_px(canon.spec.grid_points())
    off = _disc(0.12 * canon.pps)
    rows, X = [], []
    n = canon.size
    for u, v in pts:
        yy = np.clip(np.round(v).astype(int) + off[:, 0], 0, n - 1)
        xx = np.clip(np.round(u).astype(int) + off[:, 1], 0, n - 1)
        ok = valid[yy, xx] > 0
        if ok.mean() < 0.8:
            continue
        mc = cur[yy[ok], xx[ok]].astype(np.float32).mean(0)
        mr = ref[yy[ok], xx[ok]].astype(np.float32).mean(0)
        if mr.min() < 4 or mc.min() < 2:
            continue
        rows.append(np.log(mc / mr))
        X.append([1.0, u / n - 0.5, v / n - 0.5])
    if len(rows) < 8:
        return np.ones((n, n, 3), np.float32)
    Y, X = np.array(rows), np.array(X)
    y = Y @ LUMA.astype(float) / LUMA.sum()
    keep = np.ones(len(y), bool)
    for _ in range(4):
        c, *_ = np.linalg.lstsq(X[keep], y[keep], rcond=None)
        r = np.abs(y - X @ c)
        thr = max(0.04, 2.5 * 1.4826 * np.median(r[keep]))
        keep_new = r < thr
        if keep_new.sum() < 8:
            break
        keep = keep_new
    C, *_ = np.linalg.lstsq(X[keep], Y[keep], rcond=None)    # 3x3: per channel
    gx = (np.arange(n) / n - 0.5)
    G = C[0][None, None, :] + gx[None, :, None] * C[1][None, None, :] + gx[:, None, None] * C[2][None, None, :]
    return np.exp(G).astype(np.float32)


# ---------------------------------------------------------------------- per-camera model
OCC_FEATURES = ("log_diff", "log_chroma_diff", "signed_log_edge_change")
COL_FEATURES = ("lum_p90", "frac_brighter_than_square")


def start_labels() -> tuple[np.ndarray, np.ndarray]:
    b = chess.Board()
    occ = np.array([b.piece_at(i) is not None for i in range(64)], float)
    white = np.array([(b.piece_at(i) is not None and b.piece_at(i).color == chess.WHITE) for i in range(64)], float)
    return occ, white


class CameraModel:
    """Reference appearance + fitted classifiers for one camera."""

    def __init__(self, canon: Canon, ref: np.ndarray, ref_valid: np.ndarray,
                 occ: Logistic | None = None, col: Logistic | None = None):
        self.canon = canon
        self.ref = ref.astype(np.float32)
        self.ref_valid = ref_valid
        self.edge_ref = edge_map(ref)
        self.occ, self.col = occ, col

    def features(self, warp: np.ndarray, valid: np.ndarray, regions: Regions):
        v = (valid > 0) & (self.ref_valid > 0)
        G = gain_field(warp, self.ref, v.astype(np.uint8), self.canon)
        N = warp.astype(np.float32) / G
        D = np.abs(N - self.ref).mean(axis=2)
        chN = N / (N.sum(axis=2, keepdims=True) + 20.0)
        chR = self.ref / (self.ref.sum(axis=2, keepdims=True) + 20.0)
        DC = np.abs(chN - chR).sum(axis=2) * 100
        E = edge_map(np.clip(N, 0, 255))
        lumN = N @ LUMA
        lumR = self.ref @ LUMA
        off = _disc(regions.radius)
        n = self.canon.size
        occ_f = np.zeros((64, 3))
        col_f = np.zeros((64, 2))
        vis = np.zeros(64)
        for i, (u, vv) in enumerate(regions.centers):
            yy = np.clip(np.round(vv).astype(int) + off[:, 0], 0, n - 1)
            xx = np.clip(np.round(u).astype(int) + off[:, 1], 0, n - 1)
            ok = v[yy, xx]
            vis[i] = ok.mean()
            if ok.sum() < 10:
                continue
            yy, xx = yy[ok], xx[ok]
            d = D[yy, xx]
            de = E[yy, xx].mean() - self.edge_ref[yy, xx].mean()
            occ_f[i] = (np.log1p(d.mean()), np.log1p(DC[yy, xx].mean()), np.sign(de) * np.log1p(10 * abs(de)))
            # piece pixels: changed vs the empty square. A white piece's lit top is its brightest part
            # and a black piece has no bright part, so the upper percentile of the changed pixels'
            # brightness separates them on either square colour; the fraction of changed pixels
            # brighter than the empty square helps on dark squares.
            sel = d > 12.0
            if sel.sum() < 0.05 * len(d):
                sel = np.ones_like(d, bool)
            ln, lr = lumN[yy[sel], xx[sel]], lumR[yy[sel], xx[sel]]
            col_f[i] = (np.percentile(ln, 90) / 255.0, float(np.mean(ln > lr + 8)))
        return occ_f, col_f, vis

    def fit(self, start_warp: np.ndarray, valid: np.ndarray, regions: Regions, l2: float = 0.3) -> dict:
        occ_y, white_y = start_labels()
        occ_f, col_f, vis = self.features(start_warp, valid, regions)
        use = vis > 0.6
        self.occ = Logistic.fit(occ_f[use], occ_y[use], l2)
        o = use & (occ_y > 0)
        self.col = Logistic.fit(col_f[o], white_y[o], l2)
        po = self.occ.predict(occ_f[use])
        pc = self.col.predict(col_f[o])
        return {"squares_used": int(use.sum()),
                "occ_train_acc": float(((po > 0.5) == (occ_y[use] > 0)).mean()),
                "col_train_acc": float(((pc > 0.5) == (white_y[o] > 0)).mean())}

    def predict(self, warp: np.ndarray, valid: np.ndarray, regions: Regions):
        """Returns (p_occupied[64], p_white[64], visibility[64], occ_features[64x3])."""
        occ_f, col_f, vis = self.features(warp, valid, regions)
        return self.occ.predict(occ_f), self.col.predict(col_f), vis, occ_f
