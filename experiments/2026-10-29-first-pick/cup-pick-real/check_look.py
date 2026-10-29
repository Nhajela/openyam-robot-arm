"""Which start poses can the engine plan a camera-down search view from? (read-only, nothing moves)"""
import sys
import numpy as np
from armctl import kin
from interfaces.grasp.grasp import Workcell
from interfaces.wristcam.engine import GraspEngine, GraspConfig
from interfaces.wristcam.locate import load_view_inputs, make_finder

xy = np.array([float(v) for v in sys.argv[1].split(",")])
cam, T_tc, _ = load_view_inputs()
wc = Workcell.load()
cfg = GraspConfig(obj="cup", width=0.074, height=0.08, grasp_frac=0.85, expected_table=tuple(xy), place_table=tuple(xy))
starts = {"home": [0.056, -0.001, 0.004, 0.023, 0.047, -0.047], "ready": [0, 0.5, 0.8, 0, 0, 0],
          "morning look": [0.371, 1.497, 1.303, -0.595, 0, 0]}
for name, q0 in starts.items():
    class A:
        def q(self):
            return np.array(q0, float)
    eng = GraspEngine(A(), make_finder("cup", obj="cup"), cam, T_tc, wc, cfg)
    for h in cfg.look_heights:
        q, hu = eng.camera_over(xy, h)
        why = None
        if q is None:
            # retry without the path check to tell IK failure from path failure
            eng2_q = eng.tip_over(xy, eng.top_z + h - eng._cam_above_tip(np.array(q0)), np.array(q0), look=True)
            why = "IK fails" if eng2_q is None else f"IK ok, path check: {eng._check(np.array(q0), eng2_q, kin.MIN_TIP_Z, quiet=True)}"
        print(f"{name:13s} h {h:.2f}: {'q ' + str(np.round(q, 3).tolist()) + f' (h {hu:.2f})' if q is not None else 'NONE  ' + why}")
