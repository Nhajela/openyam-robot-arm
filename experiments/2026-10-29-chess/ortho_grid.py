"""Top-down (table-plane) view of a wrist-camera frame with a 5 cm base-frame grid. Read-only.
Usage: ortho_grid.py frame.jpg q1,..,q6 out.jpg [plane_h]"""
import sys
import cv2
import numpy as np
from interfaces.grasp.grasp import Workcell
from interfaces.wristcam.locate import View, load_view_inputs

f = cv2.imread(sys.argv[1]); q = [float(v) for v in sys.argv[2].split(",")]
h = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
cam, T, _ = load_view_inputs(); wc = Workcell.load()
v = View.from_q(q, cam.for_image(f), T, wc.T_base_table)
# grid in BASE frame: x 0.15..0.75, y -0.35..0.35, at table height + h
res = 0.001
xs = np.arange(0.15, 0.75, res); ys = np.arange(0.35, -0.35, -res)
X, Y = np.meshgrid(xs, ys)
zt = v.table_to_base([0, 0, h])[0][2]
P = np.stack([X, Y, np.full_like(X, zt)], -1).reshape(-1, 3)
Tcb = np.linalg.inv(v.T_base_cam)
Pc = P @ Tcb[:3, :3].T + Tcb[:3, 3]
uv = v.cam.project(Pc)
uv = np.asarray(uv, np.float32).reshape(X.shape + (2,))
ok = (Pc[:, 2] > 0).reshape(X.shape)
mx, my = uv[..., 0].copy(), uv[..., 1].copy(); mx[~ok] = -1; my[~ok] = -1
out = cv2.remap(f, mx, my, cv2.INTER_LINEAR, borderValue=(0, 0, 0))
# rows = y (top = +y = arm's left), cols = x (left = near the base)
out = cv2.transpose(out)          # now rows = x (top = near), cols = y
out = cv2.flip(out, 1)            # cols: left = -y (arm's right)?  keep a legend instead
H, W = out.shape[:2]
for k, xv in enumerate(xs):
    if abs((xv * 100) % 5) < 0.05 or abs((xv * 100) % 5 - 5) < 0.05:
        cv2.line(out, (0, k), (W, k), (0, 255, 255), 1); cv2.putText(out, f"x{xv:.2f}", (2, k - 2), 0, 0.35, (0, 255, 255), 1)
yl = ys[::-1]
for k, yv in enumerate(yl):
    if abs((yv * 100) % 5) < 0.05 or abs((yv * 100) % 5 - 5) < 0.05:
        cv2.line(out, (k, 0), (k, H), (255, 255, 0), 1); cv2.putText(out, f"y{yv:+.2f}", (k + 2, 12), 0, 0.35, (255, 255, 0), 1)
cv2.imwrite(sys.argv[3], out)
print("wrote", sys.argv[3], out.shape)
