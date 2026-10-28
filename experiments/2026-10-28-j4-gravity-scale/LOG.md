# 2026-10-28 j4 gravity feedforward: why ~1 N·m too much, and a per-joint scale

**Goal:** explain why j4 (wrist pitch, DM4310, kp 10) holds 0.015–0.109 rad off target, with the error growing with gravity load ([laptop-systems-check LOG 13:07 / 13:19](../2026-10-28-laptop-systems-check/LOG.md)), and give armctl a way to correct DimOS's gravity feedforward without editing the venv.

**Status:** analysis and code done (offline, mock/introspection tests only). Real-arm test not run yet: procedure below.

Environment: lab laptop (Linux 6.8.0-48, Ubuntu), `.venv` Python 3.12.3, DimOS 0.0.14, pinocchio 4.1.0, branch from `4c175d4`. No hardware, no zenoh, no CAN: the real arm was in a live session on 224.0.0.123:7449 and was not touched.

## 16:30 Models fetched and diffed
- Fetched from `github.com/i2rt-robotics/i2rt` (main, 2026-10-28) into [ref/i2rt/](ref/i2rt/): `yam.urdf`, `yam.xml`, `crank_4310.xml`, `linear_4310.xml`, `yam_v1.yml`, `crank_4310.yml`, `linear_4310.yml`, `yam_station_crank_4310_d405.urdf`.
- DimOS's gravity model `~/.local/share/dimos/repo/data/yam_description/urdf/yam_gripper_gravity.urdf` comes from `TomCC7/OpenYAM_description` commit `c24ef66` (per `SOURCE_COMMIT.txt`). That repo's public `main` today has only a README (one commit `db94155`), so there is no newer upstream to compare.
- Script: [gravity_models.py](gravity_models.py) → [logs/gravity_models-output.txt](logs/gravity_models-output.txt). It uses the measured q (last 5 s of each hold), not the target.

**Observation: mass carried by j4** (link4 + everything outboard):

| model | link4 | link5 | gripper body (+ fingers) | total |
|---|---|---|---|---|
| DimOS gravity URDF (fed forward on the real arm) | 0.464 | 0.351 | **1.191** (`yam_link_6` 0.950 + 2 × 0.121 fingers) | **2.006** |
| DimOS `i2rt/yam.urdf` = i2rt `yam.urdf` | 0.462 | 0.403 | 0.693 (gripper 0.553 + 2 × 0.070 tips) | 1.558 |
| i2rt `yam.xml` + `crank_4310.xml` | 0.462 | 0.403 | 0.600 (0.45 + 2 × 0.075) | 1.465 |

link4 is the same in all three. COMs of link4/5 are the same to within mm (different frames). The difference is the gripper: the DimOS gravity model's gripper is about **0.5–0.6 kg heavier** than i2rt's.

**Per-body share of τ4** at the heaviest pose (run 1, j4 −0.3): gripper body 2.034 N·m (DimOS) vs 1.005 (i2rt URDF) vs 0.906 (crank). link5 0.252 vs 0.293, link4 0.229 vs 0.229.
- **Insight:** the gripper mass alone accounts for **+1.03 to +1.13 N·m** of DimOS's j4 feedforward, the same size as the ~1 N·m surplus the spring carried in both runs (kp·err ≈ 0.94–1.11). That is the cause.

## 16:35 Fits
Truth two ways (8 holds = 4 poses × 2 runs):
- **A** = τ_ff − kp·err (MIT balance: the torque the motor was told to produce; friction ignored)
- **B** = the motor's own torque reply on CAN (console `c_tau`)

| pose | DimOS τ_ff (run 1 / 2) | i2rt URDF | i2rt crank | A run 1 / 2 | B run 1 / 2 |
|---|---|---|---|---|---|
| j4 0.0 | 2.267 / 2.319 | 1.342 / 1.379 | 1.250 / 1.285 | 1.330 / 1.255 | 1.937 / 1.971 |
| j4 −0.3 | 2.514 / 2.541 | 1.527 / 1.550 | 1.427 / 1.451 | 1.423 / 1.435 | 2.137 / 2.187 |
| j4 +0.3 | 1.901 / 1.929 | 1.092 / 1.110 | 1.012 / 1.030 | 1.489 / 1.374 | 1.687 / 1.622 |
| j4 +0.7 | 1.150 / 1.158 | 0.602 / 0.607 | 0.550 / 0.554 | 0.999 / 0.981 | 1.007 / 0.995 |

j4, against the DimOS feedforward:
- A: (a) **s = 0.628** (rms 0.21 N·m); (b) **τ = 0.289·model + 0.716** (rms 0.10).
- B: (a) s = 0.857 (rms 0.026); (b) τ = 0.842·model + 0.033 (rms 0.025).
- Gripper-mass fit (truth A): the gripper body would be 0.67 kg (k = 0.56 × stock), about i2rt's 0.69. rms 0.24.
- **Insight:** A at the two heavy poses matches i2rt's gripper model within 0.05 N·m. At the light poses A is above every model by 0.3–0.4 N·m, and run 1 and run 2 agree there. That looks like friction holding the wrist (i2rt models 0.06 N·m Coulomb on j4, but the gearbox likely has more), or wrist-camera mass with a different lever. The large intercept in A(b) is that friction, not a real offset.
- **Insight:** B (the CAN torque) is a tight 0.84–0.86 × the feedforward, but it does not equal A: it is 0.6–0.75 N·m higher at the heavy poses and equal at +0.7. The motor's reported torque is not a faithful copy of the commanded MIT torque. The scale that zeroes the error is set by A (what the firmware actually balances), so the target is about **0.63**; B's 0.83–0.86 is a conservative first step. Expected error at the heavy poses (kp 10): s 0.83 → ~0.066 rad, s 0.70 → ~0.033, s 0.63 → ~0.015 rad. At +0.7, s 0.63 undershoots by ~0.27 N·m (−0.027 rad at kp 10, inside the friction band).
- **j2/j3:** no torque logged, only position error. j2 sits −0.032 to −0.040 and j3 exactly −0.0292 in all 8 holds. The sign (sag) says under-compensation, matching i2rt's `gravity_comp_factor` 1.1 on j2/j3, but j3's error doesn't change while its model load varies by 1.4 N·m (which kp 80 would show as 0.017 rad). Stiction or a target offset dominates, so **no reliable j2/j3 scale from this data**. A = ff − 80·err would give s ≈ 1.33 for j3 and a meaningless 3.3 for j2. **Idea:** log the j2/j3 CAN torque and error at 3–4 j2/j3 poses.
- i2rt's own `yam_v1.yml`: `gravity_comp_factor [1.0, 1.1, 1.1, 1.2, 1.0, 1.0]`, `coulomb_friction [0.3,0.3,0.3,0.06,0.06,0.06]`. Their 1.2 on j4 applies to their lighter gripper model: 1.2 × i2rt ≈ 0.73 × DimOS's, consistent with our A-based 0.63–0.73.

## 16:40 Decision: armctl-side gravity scale (no venv edit)
DimOS: `dimos/hardware/whole_body/damiao/adapter.py` `write_motor_commands` (346–364) sends `tau + gravity[i]` in the MIT rows. `_gravity_torques` (506–516) is pinocchio `computeGeneralizedGravity(self._pin_model, …, measured q)`. `_load_kinematic_model` builds `_pin_model` from `OpenYamDamiaoAdapter.kinematic_model` (`urdf/yam_gripper_gravity.urdf`), which is also where the fault-latching limits come from.
- **Decision:** subclass, not monkeypatch. [`armctl/gravity_adapter.py`](../../armctl/gravity_adapter.py) `ArmctlOpenYamDamiaoAdapter(OpenYamDamiaoAdapter)` overrides `_gravity_torques` (DimOS's vector × per-joint scale) and `kinematic_model` (optional validated URDF swap). [`armctl/gravity.py`](../../armctl/gravity.py) parses and validates, and changes the real hardware component to `adapter_type="armctl_openyam_damiao"` with `adapter_kwargs["armctl_gravity"]`. `ArmctlTwistCoordinator._create_whole_body_adapter` builds it inside the DimOS worker. That worker already imports `armctl.dimos_coordinator` to unpickle the class, so no registry patching is needed. Any other adapter gets the kwarg stripped and goes to DimOS's factory.
- CLI: `armctl.dimos_coordinator --gravity-scale j4=0.83` (repeatable or comma list, each in **[0.5, 1.5]**, else refused) and `--gravity-urdf PATH`. The URDF must match the stock file's joints, origins, axes and **limits** to 1e-6, and its total mass must be within 0.5–1.5×. The adapter re-checks both in the worker. No flag: stock adapter, component untouched.
- Startup line: `gravity[OVERRIDE j4x0.83]` plus `gravity feedforward at ready [...]: j4 2.378->1.974 N·m (adapter …)`. This is computed offline through the real subclass with no CAN. `armctl_info()["gravity"]` shows the adapter, scale, urdf and whether it is applied.
- In `--simulation` the mock adapter has no gravity, so the request is carried and shown but not applied. That makes sim a plumbing check only.
- Tests: [`armctl/tests/test_gravity.py`](../../armctl/tests/test_gravity.py) (33). Parsing and range limits; defaults unchanged; real/sim component routing; unit scale equals DimOS's adapter exactly; per-joint scaling at 4 poses; ready value 2.378 matches the 13:07 table; DimOS's own `write_motor_commands` with fake motors sends `tau + scale·gravity` with kp and q untouched; the URDF validator (limits, origin, mass, missing joint, junk); CLI refusals happen before pgrep/build; `_create_whole_body_adapter` routing. `pytest -q armctl/tests`: **219 passed**. (`test_daemon.py::test_stream_deadman_stops_after_ttl` is timing-flaky: 1 of 3 isolated runs failed `0.1 > 0.1`. It is unrelated, and the full suite passed on rerun.)

## Proposed real-arm procedure (not run; needs the person at the arm)
1. **Sim plumbing**, bus 224.0.0.124:7450: `ZENOH_SCOUT_ADDR=224.0.0.124:7450 .venv/bin/python -m armctl.dimos_coordinator --simulation --kp j4=22 --kd j4=2 --gravity-scale j4=0.83`. Check the startup line (`j4 2.378->1.974`, mock: not applied). Start the sim daemon and check that its `armctl_info` log has `gravity.openyam.scale.j4 = 0.83`. Run `j4_hold_test.py --url ws://127.0.0.1:8791/armctl --hold 5`.
2. **Real, step 1 (conservative):** pre-flight per CLAUDE.md (arm folded, `tools/probe_motors.py` ≈ 0, no faults, wide `pgrep`). Then `ZENOH_SCOUT_ADDR=224.0.0.123:7449 … --real --can-port can0 --kp j4=22 --kd j4=2 --gravity-scale j4=0.83`. Confirm `adapter=armctl_openyam_damiao` and the gravity line. **Support the wrist at enable**: less feedforward means it can droop more if the scale is wrong. Then `j4_hold_test.py --console http://127.0.0.1:8090 --hold 60 --kp4 22`. Expect err ≈ 0.03 at 0/−0.3, versus 0.048/0.050 in run 2.
3. **Step 2:** same with `j4=0.70`, then `j4=0.63`. Stop at the scale where the heavy-pose error is ≲0.01 and the +0.7 error doesn't go negative beyond ~−0.01. Then try dropping back to kp 10 (the j4 temperature should fall).
4. After every stop: support the arm, `tools/disable_all.py`, LEDs steady red. Log the CSVs here.
5. **Idea (later):** a corrected gravity URDF (gripper body ≈0.67 kg plus the measured camera + bracket mass and COM) via `--gravity-urdf`, which fixes j5/j6 too. Weigh the gripper + camera first.
