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

### Re-plan for today's layout; arm powered off for hand reach-checking; camera exposure
- The combined lens (`interfaces/wristcam/data/combo1`, 44 views) was re-solved into `interfaces/wristcam/calib/wristcam.json` (fx 537.1, fy 532.9, RMS 0.79; the arm-only solve is kept as `wristcam.intr1-only.json`).
- **Hand-eye re-solve of `he1` at 22.9 mm** ([handeye-he1-229.rejected-*.json](.)): RWHE-SHAH+refined, reprojection 4.37 px, spread 6.68 mm / 1.44°, camera t = (−85.8, 13.3, 61.3) mm in the tip frame, **optical axis 44.7° from the fingers**, 10.6 cm from the tip; board scale from kinematics ×0.979 → **square ≈ 22.42 mm**; table z −0.044 (old layout, probably the base board's thickness). Gates: reprojection, spread and table z fail narrowly; axis, camera distance and scale pass. Kept as a planning guess, not used for grasping.
- `handeye plan/collect` gained `--calib-guess` / `--handeye-guess` (plan visibility with the measured camera instead of the nominal). With them, 15 rotation-diverse views around the board (median 37.5°, axis spread 0.55) all see 4/4 corners.
- **Decision (Naman):** "we can print more sheets": tape one checkerboard **permanently**, flat, off to the side of the work area (arm's left, ~30–40 cm out) as the reference for today's hand-eye and future re-checks; the table centre stays free for objects. Reprints must use the same printer settings (this print came out at ~22.4 mm/square, not 30).
- Naman asked to power the arm off to move it by hand and check its reach: daemon + coordinator stopped, `disable_all` (all disabled, no faults; j4 rotor 43 °C after the gravity test). Back near home after (j1 +0.075).
- **Problem:** at home the wrist camera's auto-exposure (`auto_exposure=3`, aperture priority) blew the sunlit table out to white. **Fix:** manual exposure with `v4l2-ctl -d /dev/video0 -c auto_exposure=1 -c exposure_time_absolute=110 -c gain=60` (exposure 60 → mean 53/255, auto → nearly all clipped). Set again after a camera reconnect or reboot; tune per lighting.
