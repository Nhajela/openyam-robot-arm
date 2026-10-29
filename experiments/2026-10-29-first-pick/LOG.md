# 2026-10-29 first pick (gravity fix, Route A pick, camera pick)

**Experimenters:** Naman (+ Claude)
**Goal:** j4 cool and healthy; test the gravity-scale fix; first real pick of the white paper cup without the camera (ruler-measured positions, "Route A"); then finish the wrist-camera calibration for camera-based picking. Continues [2026-10-28-laptop-systems-check](../2026-10-28-laptop-systems-check/LOG.md).
**Status:** active

## Environment
Lab laptop `pheonix-drone`, Ubuntu 24.04, DimOS 0.0.14, repo `main` @ `9b941a2` (= fork/main). The base is rotated (forward = down the table), `armctl/site_limits.json` j1 soft −0.6..+1.2.

## Log

### 2026-10-29 morning: power-on probe
The arm's power was off overnight (j4 had overheated and gone silent on CAN, 28 Oct 17:40). Naman powered it on; `.venv/bin/python -I tools/probe_motors.py can0`:
```
joint1 0.036  joint2 -0.001  joint3 -0.001  joint4 0.021  joint5 -0.136  joint6 -0.068  gripper -2.899
all 27–28 °C rotor, enabled False, fault None; CAN sent 35 received 70, 0 errors
```
- **Observation:** **j4 answers again** and is cold (28 °C), so last night was its thermal protection, cleared by the power-cycle. j5 at −0.136 (turned by hand overnight), inside its range.

### Gravity-scale: sim plumbing check
`ZENOH_SCOUT_ADDR=224.0.0.124:7450 .venv/bin/python -m armctl.dimos_coordinator --simulation --kp j4=22 --kd j4=2 --gravity-scale j4=0.83`
→ `gains[OVERRIDE j4]: kp=[80,80,80,22,10,10] kd=[5,5,5,2,1.5,1.5]`, `gravity[OVERRIDE j4x0.83]`, `gravity feedforward at ready: j4 2.378->1.974 N·m (adapter mock_whole_body, mock: not applied)`, started. The wiring works; the mock has no gravity, so the real effect needs the arm.

### 08:15–08:21 Gravity-scale on the real arm: j4 hold test (stiffer + gravity 0.83)
Real coordinator `--kp j4=22 --kd j4=2 --gravity-scale j4=0.83`, adapter **`armctl_openyam_damiao`** (first real use; activation fine), startup `gravity feedforward at ready: j4 2.378->1.974 N·m`; the daemon's `armctl_info` confirms `scale j4 0.83, applied: True`. Lab console restarted (read-only) for torque/temperature. `j4_hold_test.py --console http://127.0.0.1:8090 --hold 60 --kp4 22` ([logs/j4-grav083.log](logs/j4-grav083.log), CSV [logs/j4-hold-20261029-081627.csv](logs/j4-hold-20261029-081627.csv)).

| j4 target | model τ4 (stock) | err Tue run 1 (kp 10) | err Tue run 2 (kp 22) | **err today (kp 22 + scale 0.83)** | measured τ4 | rotor °C |
|---|---|---|---|---|---|---|
| 0.0 | 2.378 | +0.0937 | +0.0484 | **+0.0147** | 1.858 | 29→33 |
| −0.3 | 2.560 | +0.1091 | +0.0503 | **+0.0226** | 2.013 | 33→36 |
| +0.3 | 1.984 | +0.0412 | +0.0252 | **−0.0007** | 1.620 | 37→37 |
| +0.7 | 1.191 | +0.0151 | +0.0081 | **−0.0076** | 1.014 | 37→35 |

`home` done, max error **0.0231** (Tue 0.0616 / 0.0319). No refusals or faults.
- **Result:** the worst-case j4 error is ~0.02 rad (≈2–3 mm at the tip), 4–5× better than the default; **j4 peaked at 37 °C** (Tue 62 °C).
- **Insight:** light poses now go slightly negative, matching the agent's affine fit (τ_true ≈ s·τ_model + c; friction/offset). A lower scale would trade heavy-pose error for more light-pose overshoot. **0.83 is a good single value.**
- **Decision:** the real-arm start line is now `--kp j4=22 --kd j4=2 --gravity-scale j4=0.83` (HANDOVER updated).

### 2026-10-29 (sim only) Tap to pick / say what to pick
Built "pick up THAT": tap the thing (or say "the cup on the table", Claude finds it) on a still wrist-camera frame, and the closed-loop engine homes in on it and picks it up. It's open play's new **Pick it up** mode. Simulation and tests only: nothing ran on the arm. Write-up, results and what it needs from the real arm: [TAP-TO-PICK.md](TAP-TO-PICK.md).
- **Insight:** the silhouette alone can't give an object's height. A prism fit (top face = footprint) can: synthetic tap → box 1.4 mm, cup 0.4 mm.
- **Fix:** the engine refused short objects (approach and lift-clear below the 0.08 m tip guard). Fixed in `engine.py`; the cup tests are unchanged.
