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
