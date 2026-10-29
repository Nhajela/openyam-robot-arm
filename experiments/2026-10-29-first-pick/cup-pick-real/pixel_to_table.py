"""Pixel(s) on a frame taken at q -> base and table xy on a plane at height h above the table.
Usage: pixel_to_table.py q1,..,q6 u,v,h [u,v,h ...]   (read-only, nothing moves)"""
import sys
import numpy as np
from interfaces.grasp.grasp import Workcell
from interfaces.wristcam.camera import ray_plane
from interfaces.wristcam.locate import View, load_view_inputs

q = [float(x) for x in sys.argv[1].split(",")]
cam, T_tc, notes = load_view_inputs()
wc = Workcell.load()
T_bt = np.array(wc.T_base_table, float)
v = View.from_q(q, cam, T_tc, T_bt)
o = v.T_base_cam[:3, 3]
for spec in sys.argv[2:]:
    u, vv, h = (float(x) for x in spec.split(","))
    d = v.T_base_cam[:3, :3] @ cam.rays(np.array([[u, vv]], float)).reshape(-1, 3)[0]
    p_plane = v.table_to_base([0, 0, h])[0]
    P = ray_plane(o, d, p_plane, T_bt[:3, 2])[0]
    Pt = np.linalg.inv(T_bt) @ np.r_[P, 1]
    print(f"px ({u:.0f},{vv:.0f}) h {h:.3f}: base ({P[0]:+.3f}, {P[1]:+.3f}, {P[2]:+.3f})  table ({Pt[0]:+.3f}, {Pt[1]:+.3f})  r {np.hypot(P[0], P[1]):.3f}")
