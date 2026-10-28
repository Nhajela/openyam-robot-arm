# 2026-10-28 laptop systems check

**Experimenters:** Naman (+ Claude)
**Goal:** Pull the fork's `main` (interfaces, open play, DimosBackend built on serenity) onto the lab laptop and check everything short of enabling torque: CAN link, motors (read-only), Python deps, test suites, and the full DimOS-sim path. This is the go/no-go before the first hardware session in [HANDOVER.md §5](../../HANDOVER.md).
**Status:** software checks green; first real-arm session (HANDOVER §5 steps 1–3) done at 12:43. Two findings: j4 holds 0.06–0.12 rad off target, and the gripper direction still needs Naman's eyes.

## Environment

- Laptop `pheonix-drone`, Ubuntu 24.04 LTS, kernel 6.8.0-48-generic
- Python 3.12.3 (repo `.venv`), DimOS 0.0.14
- Repo `main` @ `2bb9b6d`, tracking `fork/main` (Nhajela/openyam-robot-arm)
- CAN: gs_usb adapter `1d50:606f` (USB 1-1), `can0` UP, 1 Mbit/s, sample point 0.747, restart-ms 100
- Arm folded on its stops, cables connected, motors powered

## Log

### 12:27 Pull
- **Observation:** `fork/experiments` had nothing new. `fork/main` = `d4a5c10` (Edge-City PR #2, whose tree is identical to our old `experiments` branch) + 24 commits from serenity (10:13–12:27).
- **Decision:** local `main` now tracks `fork/main` (`git branch -u fork/main`), fast-forwarded to `2bb9b6d`. This matches HANDOVER §1: the code lives on the fork.

### 12:29 CAN link and processes
- `pgrep -af "bin/dimos|dimos_coordinator|armctl_blueprints|armctl.daemon"`: nothing running.
- `ip -details -statistics link show can0`: `state ERROR-ACTIVE`, 0 bus errors, 0 frames before the probe.

### 12:30 Motor probe (read-only, no torque)
`.venv/bin/python -I tools/probe_motors.py can0`
```
CAN frames sent=35 received=70 dropped=0 errors=0
motor      id  pos(rad)  mos°C  rotor°C  enabled fault
joint1   0x01     0.041     33       31    False None
joint2   0x02    -0.005     33       31    False None
joint3   0x03    -0.001     33       31    False None
joint4   0x04     0.022     35       33    False None
joint5   0x05    -0.041     35       32    False None
joint6   0x06    -0.033     34       32    False None
gripper  0x08    -1.201     33       32    False None
```
- **Observation:** all 7 motors answer. j1–j6 are within 0.05 rad of 0 (j1 did not come up 180° off this time). No faults, 31–35 °C, all disabled.
- **Idea:** gripper −1.201 rad is the raw motor angle (on 16 Oct `--watch` saw g=−2.91). Record the raw closed/open angles in hardware step 3, when we check the gripper direction.

### 12:31 Python deps
- **Problem:** `import mediapipe` failed. The interfaces deps were not in the laptop venv yet.
- **Fix:** `uv pip install --python .venv/bin/python -r interfaces/requirements.txt qrcode anthropic` installed mediapipe 1.2.0, chess 1.11.2, anthropic 2.3.0. `import cv2; cv2.aruco; import mediapipe, chess, dimos, qrcode, anthropic` now works, with OpenCV 4.14.0 (aruco not broken).

### 12:32 Unit tests
`.venv/bin/python -m pytest -q armctl/tests interfaces/tests openplay/tests`: **291 passed in 81.5 s.** HANDOVER expected about 253; more tests have been added since.

### 12:33 Full DimOS-sim path (sim bus only)
Two tmux sessions. Coordinator in pane `%1`, daemon in pane `%0`, both with `ZENOH_SCOUT_ADDR=224.0.0.124:7450`:
```sh
.venv/bin/python -m armctl.dimos_coordinator --simulation --traj-vel-limit 0.7
.venv/bin/python -m armctl.daemon --backend dimos --auto-approve --port 8791 --log ''
.venv/bin/python armctl/tests/e2e_dimos_sim.py --url ws://127.0.0.1:8791/armctl
```
- The coordinator says `mode=SIMULATION adapter=mock_whole_body bus=224.0.0.124:7450`, with tasks `eef_twist_arm`, `joint_trajectory`, `openyam_gripper`, zenoh on `lo` only. The daemon log says `DimosBackend connected: DimOS SIM (mock_whole_body) ... can None`.
- **Result: 27/27 passed.** Key numbers: final errors 0.0, first motion 30 ms, twist deadman stop 0.30 s, joint stream deadman 0.38 s, stop-to-still 0.09–0.15 s, E-STOP to still 5.7 ms, home 8.2 s. These match the serenity run. Logs: [logs/sim-e2e.log](logs/sim-e2e.log), [logs/sim-daemon.log](logs/sim-daemon.log), [logs/sim-coordinator.log](logs/sim-coordinator.log).
- **Observation:** while the arm is folded, the coordinator logs `Pink streaming command envelope is empty for 'yam_joint2'` about 100 times a second. This is known and harmless (same on serenity): Pink IK has no feasible step from the folded pose.
- **Problem (tmux):** `tmux send-keys -t 0` went to session **1**, not session 0. The daemon command was typed into the running coordinator's stdin. **Fix:** target panes by ID (`-t %0`, `-t %1`). On shutdown, the buffered line was cleared with extra Ctrl-C.
- After the stop: no DimOS processes left. The `can0` counters read 70 sent / 140 received, exactly the probe's two passes (refresh + disable on exit). **Insight:** the sim run never touched the real bus, as designed.

### 12:35 Crash dialog: sim coordinator segfault on shutdown
- **Problem:** Ubuntu showed "python stopped unexpectedly". The kernel log has `python[16349]: segfault at 10 ... in python3.12` at 12:35:43, about 10 s after Ctrl-C to the **sim** coordinator (`/var/crash/_usr_bin_python3.12.1000.crash`). This is a DimOS worker dying on teardown, with no hardware involved. **Idea:** if it repeats on the real coordinator, motors could be left in an unknown state, which is one more reason `disable_all` always follows.

### 12:39 Real arm, HANDOVER §5 steps 1–3 (Naman supporting the arm)
Pre-flight: no DimOS processes. Probe: j1–j6 within 0.05 rad of 0, no faults, gripper raw −1.201.

**Step 1.** Coordinator in pane `%1`, daemon in pane `%0` ([logs/real-coordinator.log](logs/real-coordinator.log), [logs/real-daemon.log](logs/real-daemon.log)):
```sh
ZENOH_SCOUT_ADDR=224.0.0.123:7449 .venv/bin/python -m armctl.dimos_coordinator --real --can-port can0
ZENOH_SCOUT_ADDR=224.0.0.123:7449 ARMCTL_DIMOS_EXPECT=real .venv/bin/python -m armctl.daemon --backend dimos --auto-approve
```
- The coordinator says `mode=REAL can=can0 adapter=openyam_damiao bus=224.0.0.123:7449 ... traj_vel_limit=0.5`. The daemon says `DimOS REAL (openyam_damiao) ... can can0`, with caps of 0.3 rad/s and 1.0 rad/s².
- **Observation:** the coordinator warns `Clamping Damiao joint feedback at position limit ... yam_joint2 limit=0.0 value=-0.0048` (and j3 −0.0010). This is harmless: folded on the stops, the encoders read a hair below 0.
- **Decision:** the console (`dashboard/server.py`, which opens `can0` itself in `auto` mode) stayed off, so only one process was on the bus. Commands went through [arm_do.py](arm_do.py), an `interfaces.client` wrapper. To make that work, the daemon ran with `--auto-approve`, bound to 127.0.0.1.

**Step 2: ready, then home** ([logs/real-steps.log](logs/real-steps.log))
- `ready` played as a 5.00 s trajectory, then was **refused** on verify: `ready did not reach target: j4 error 0.094 rad > 0.09; stopped`. The held pose:

  | | j1 | j2 | j3 | j4 | j5 | j6 |
  |---|---|---|---|---|---|---|
  | target | 0 | 0.5 | 0.8 | 0 | 0 | 0 |
  | actual | +0.007 | 0.463 | 0.771 | **+0.094** | −0.023 | −0.014 |
- **Fix:** `armctl/daemon.py` `REACH_TOL_BY_BACKEND["dimos"]` wrist tolerance raised from 0.09 to 0.12 (j4–j6). Only the daemon was restarted; the coordinator held the arm meanwhile. `ready` again: done, but j4 now **+0.116** (it grew while holding).
- `home`: 7.95 s trajectory, `done home (max error 0.0620 rad)`, j4 still +0.060 when folded.
- After `disable_all`, j4 reads +0.022 with torque off, and its rotor went from 33 °C to **43 °C** (the others stayed at 32–38 °C).
- **Insight:** j4 is not sagging like j2/j3. It carries a +0.06 to +0.12 rad holding error in **both** poses (even folded, where gravity on the wrist is small), and it runs the hottest. The likely causes are a low kp on j4 against friction or cable drag, or a load on the wrist. The 0.12 tolerance gives no headroom. **Idea:** before the next session, log j4 command vs feedback while holding `ready` for 60 s. Check the cable routing around j4, and the kp/kd that DimOS uses for the DM4310 wrist.

**Step 3: gripper**
- `gripper 0.5`, then 2 s later `gripper 1`. Both were acked. The reported grip afterwards was 0.9997, and the raw motor angle went from −1.201 (start) to **−2.910** (after; the same −2.91 seen on 16 Oct).
- **Problem (script):** the daemon only acks `gripper` and never sends `done`, so `arm_do.py` waited and timed out at 60 s. **Fix:** it now waits only for the ack for gripper.
- Under armctl's mapping (and the probe's `opening_direction="decreasing_position"`), the move to −2.91 is an OPEN.

**Shutdown 12:43:** Ctrl-C daemon, Ctrl-C coordinator, `.venv/bin/python -I tools/disable_all.py can0`: all 7 `enabled=False fault=None`. Naman: **all LEDs steady red.**

### 12:45 Lab console started (read-only)
`.venv/bin/python dashboard/server.py --log experiments/2026-10-28-laptop-systems-check/LOG.md` in a new tmux window (pane `%2`), http://127.0.0.1:8090. It polls at 10 Hz in `auto` mode and stops transmitting while DimOS drives the bus (`external_controller: true`). It stays up between runs.
- **Insight (which joint is j4):** per the URDF, j4 (`link3 -> link4`, axis y, DM4310, CAN 0x04) is the **wrist pitch**: the first small motor after the elbow, tipping the gripper up and down about an axis parallel to j2/j3.

### 12:47 Gripper re-run (Naman watching)
Same coordinator + daemon commands as at 12:39. `gripper 0`, 3 s, `gripper 1`:

| command | reported grip | raw motor angle (console) |
|---|---|---|
| (before) | – | −2.916 |
| 0 | 0.0075 | +0.012 |
| 1 | 0.9996 | −2.915 |

- **Naman: "it did close and open".** ✅ **Gripper direction confirmed: 0 = closed (raw ≈ 0), 1 = open (raw ≈ −2.91).** `DimosBackend.gripper` needs no change.
- **Observation:** at 12:39 the daemon reported `grip=1.0` while the raw angle was −1.201 (partly open). So before the first command, the reported grip is probably the commanded default, not a measurement. Don't trust `grip` until a gripper command has been sent.
- Naman then said "twice": the gripper closed and opened twice. Explained at 12:58 (below).
- **Observation (j4 again):** folded and holding, j4 = +0.064 (+0.022 torque-off), so about 0.04 rad of holding error even with no wrist load. After about 1 min of torque, j4 is again the warmest (mos 40 °C).
- Shutdown 12:48: daemon, coordinator, `disable_all`: all 7 `enabled=False fault=None`.

## Verdict
Every check short of torque passed: CAN link, all 7 motors, deps, 291 unit tests, 27/27 DimOS-sim e2e. Steps 1–3 on the real arm worked end to end through the armctl → DimOS path. Before steps 4–5 (joystick, library moves): understand j4's holding error, and confirm the gripper direction.

### 12:55 Why j4 holds off target (code research by a read-only subagent; key lines checked by hand)
- **Insight:** DimOS **does** apply gravity feedforward on the real arm. The 16 Oct notes say it doesn't (`2026-10-16-linux-real-arm/SUMMARY.md:37`, `LOG.md:145`); that is wrong.
  - `dimos/robot/manipulators/openyam/config.py:59-74`: `DamiaoRuntimeConfig(gravity_comp=True)`, **kp = (80, 80, 80, 10, 10, 10, 0)**, kd = (5, 5, 5, 1.5, 1.5, 1.5, 0).
  - `dimos/hardware/whole_body/damiao/adapter.py:346-364`: MIT mode, `tau + gravity`, where gravity is pinocchio `computeGeneralizedGravity` at the measured q.
  - The gravity model is `yam_description/urdf/yam_gripper_gravity.urdf` (`openyam_damiao/adapter.py:51`), not `i2rt/yam.urdf`.
- **Arithmetic:** the gravity model gives a j4 torque of 2.56 N·m (home) and 2.38 N·m (ready); `i2rt/yam.urdf` gives 1.57 and 1.42. About 1.56 kg hangs below j4. The P term is carrying 10 × 0.062 ≈ 0.6 N·m (home) and 10 × (0.094 → 0.116) ≈ 0.9 → 1.2 N·m (ready). With no feedforward, the sag would be 0.14–0.26 rad, so the feedforward works but misses by 25–50%. The likely causes: a mass or centre-of-mass mismatch (the two URDFs differ by 1 N·m on this joint), the +0.022 encoder zero offset on j4, and gearbox friction. j2/j3 have the same kind of mismatch, but kp 80 shrinks it 8×.
- 16 Oct had j4 sag of 0.062–0.084 (LOG.md:142-174); today's 0.094 → 0.116 at ready is larger. The drift while holding plus the warming (33 → 43 °C) fit a torque constant that drops as the rotor heats.
- **Idea / next tests** (sim first, real only with Naman's OK):
  1. Hold `ready` for 5 min and log j4 q and rotor temperature at 10 Hz; repeat with j4 at −0.3 / 0 / +0.3. If the sag follows the model's j4 torque, the cause is the model or gain; if it stays constant, it's the zero or friction.
  2. Settle the +0.022 torque-off offset (re-zero check) before any tuning.
  3. Raise j4 kp from 10 to 20–25 (kd 1.5 → 2) through an armctl-side override (not by editing the venv). Watch temperature and buzz.

### 12:58 Gripper: the calibration sweep on enable (code research by a read-only subagent; docstring checked by hand)
- **Insight:** `can_motor_control.Robot.enable` "also runs the feedback-based opening calibration" for grippers in normalized-opening mode. The DimOS adapter calls it from `activate()` (`dimos/hardware/whole_body/damiao/adapter.py:247`). **Every coordinator start moves the gripper through a close/open sweep and leaves it fully open.** That was Naman's first cycle; our `0`/`1` was the second.
- That also explains `grip=1.0` at raw −1.201: the −1.201 came from the probe *before* the coordinator started. After calibration the gripper sat open at −2.91, so 1.0 was a correct measurement. The earlier "commanded default" guess was wrong. armctl's `grip` is the measured DimOS `arm/gripper` value (`armctl/backend.py:1011`). The gripper task has no default target, and nothing moves at shutdown.
- **Decision (safety):** before every coordinator start, keep hands and objects out of the gripper jaws, because the start-up sweep moves them. Also, a grasped object would be released at start-up. The `DimosBackend` docstring now records the gripper mapping as measured.

### 13:05 Commit, j4 hold test (sim), open play on the sim, parallel agents
- Committed and pushed `f89c234` to `fork/main`.
- Corrected the "no gravity feedforward" lines in the 16 Oct SUMMARY.md and LOG.md (left struck through, with a pointer here).
- **[j4_hold_test.py](j4_hold_test.py):** for j4 at 0, −0.3, +0.3 and +0.7 (j2 0.5, j3 0.8), it does `move_joints`, holds, logs at 10 Hz (daemon q, plus j4 tau and temperatures from the read-only console), then `home`. It prints the model's j4 gravity torque beside the measured error. The model torque is 2.38 / 2.56 / 1.98 / 1.19 N·m for those four poses, so the spread can tell model-proportional sag from a constant offset.
  - Sim run (`--hold 5`, DimOS sim on 224.0.0.124:7450, daemon :8791): all 4 poses and home OK, errors 0 (the mock arm doesn't sag). [logs/j4-hold-20261028-125124.csv](logs/j4-hold-20261028-125124.csv).
  - ⏳ Real run: `--console http://127.0.0.1:8090 --hold 120`, about 10 min of torque, only with Naman at the arm.
- **Open play on the DimOS sim** (window `openplay`, pane `%3`): `.venv/bin/python -m openplay.server --daemon ws://127.0.0.1:8791/armctl --host-ip 192.168.100.191 --turn 300`. It says `Connected to the arm (dimos backend); auto-play off`. Display: http://127.0.0.1:8451. Phones: the QR code, `https://192.168.100.191:8450/?k=...` (self-signed certificate). Log: [logs/openplay-sim.log](logs/openplay-sim.log).
- **Decision:** three coding agents in separate git worktrees, mock/pytest only, no CAN, no zenoh: (a) a j4/wrist kp/kd override for `armctl.dimos_coordinator`, (b) a flowing spline ease for pose sequences, (c) chess mode in open play.


### 13:20 Direction: phone UX and chess on a real board
- **Decision (Naman):** the phone should feel like a game. You choose a mode from cards, get a "get set" screen (for tilt, a **start pose** to hold the phone in, which becomes "still" for the whole turn), then play, with STOP, a **🏠 Home** button (new server `home` op) and a Modes button. An agent is building it in a worktree.
- **Decision (Naman):** chess is played on a **real board**, not a tappable one. The human moves, hits a **chess-clock** "I've moved" button (phone, laptop, later a physical button), **cameras around the board** see the change, the server matches it to a legal move, and the robot moves the real pieces. Tap-to-move stays only as the fallback when vision is unsure. This is CHESS.md §1 layer 2 promoted to the main flow.
  - Shared interface `interfaces/chess_vision/sensor.py`: `Snapshot` (per-square occupancy + colour probabilities, camera thumbnails), the `BoardSensor` protocol, and `rank_moves(board, before, after)`.
  - Game agent: clock UI, game flow, `rank_moves`, `FakeBoardSensor`.
  - Vision agent: `CameraBoardSensor`, i.e. ArUco corner markers → homography per camera, per-square occupancy and colour, multi-camera fusion, calibration CLI, synthetic-image tests.
- Merged `9f9ec21` (`--kp/--kd` per-joint gain overrides) as `630226e`. One conflict, because the agent's worktree started from `d4a5c10`; resolved by keeping both sides; armctl 171 passed. Pushed `a312001`.

### 13:07–13:17 j4 hold test, run 1 (DimOS default gains, j4 kp 10 / kd 1.5); Naman at the arm
Pre-flight: no DimOS processes. Probe: j1–j6 within 0.04 of 0, no faults, j4 rotor 36 °C. (A leftover agent screenshot `openplay.server` pointed at a dead mock port; nothing on 8091 or `can0`.) Coordinator `gains[dimos-default]: kp=[80,80,80,10,10,10] kd=[5,5,5,1.5,1.5,1.5]`; daemon `DimOS REAL`. Then:
`.venv/bin/python experiments/2026-10-28-laptop-systems-check/j4_hold_test.py --console http://127.0.0.1:8090 --hold 120`
([logs/j4test-run1.log](logs/j4test-run1.log), CSV [logs/j4-hold-20261028-130801.csv](logs/j4-hold-20261028-130801.csv), 4663 rows; coordinator/daemon logs `logs/j4test-*.log`)

| j4 target (j2 0.5, j3 0.8) | model gravity τ4 (DimOS feedforward) | j4 error, held | kp·err | measured τ4 (CAN reply) | measured / model | rotor °C |
|---|---|---|---|---|---|---|
| 0.0 | 2.378 | +0.0937 | 0.94 | 1.937 | 0.81 | 38→46 |
| −0.3 | 2.560 | +0.1091 | 1.09 | 2.137 | 0.83 | 46→48 |
| +0.3 | 1.984 | +0.0412 | 0.41 | 1.687 | 0.85 | 49→49 |
| +0.7 | 1.191 | +0.0151 | 0.15 | 1.007 | 0.85 | 48→44 |

`home`: done, max error 0.0616. Shutdown 13:17: daemon, coordinator, `disable_all`: all 7 disabled, no faults, **LEDs all steady red (Naman)**; j4 rotor 46 °C (hottest; j3 42).
- **Observation:** each error is rock-steady within a pose (first 5 s = last 5 s to 4 decimals), and the same at 0.0 as at 12:39 (0.094). It doesn't drift while held; the earlier 0.094 → 0.116 came from re-running `ready` from a slightly different start, not from creep.
- **Insight:** the error **tracks the gravity load**: 0.015 at 1.19 N·m up to 0.109 at 2.56 N·m. It is not a constant offset, so j4's zero is fine (only the +0.022 torque-off reading, which is small).
- **Insight (sign):** the torque j4 actually delivers is a steady **0.81–0.85 × the model's gravity torque**, and j4 sits on the **+ side** of its target in every pose. So the DimOS gravity model (`yam_gripper_gravity.urdf`) most likely **over-estimates** the wrist load by about 15–20%. The feedforward pushes the wrist past its target, and kp 10 only partly pulls it back. This corrects the 12:55 subagent guess of "under-estimates". kp·err doesn't equal model − measured exactly (0.94 vs 0.44 at 0.0), so friction and the CAN torque scaling are also in play. Treat the measured τ as indicative, not exact.
- **Idea / next:**
  1. **Run 2 with `--kp j4=22 --kd j4=2`** (merged today): the error should drop roughly in proportion (~0.04 at 0.0). This needs no DimOS change.
  2. Better long-term: scale the j4 gravity feedforward by about 0.83, or fix the gripper mass in the gravity URDF (the gripper + camera actually fitted may weigh less than modelled). That needs a DimOS-side hook.
  3. j4 runs warm (46–49 °C rotor during holds) but within a normal range. Watch it after the kp increase.

### 13:19–13:29 j4 hold test, run 2: `--kp j4=22 --kd j4=2`; Naman at the arm
Same procedure as run 1. Coordinator `gains[OVERRIDE j4]: kp=[80,80,80,22,10,10] kd=[5,5,5,2,1.5,1.5]`; the daemon's `armctl_info` confirms j4 kp 22 / kd 2. A leftover agent **mock** daemon was on :8591 (no CAN, no DimOS): ignored. ([logs/j4test-run2.log](logs/j4test-run2.log), CSV [logs/j4-hold-20261028-132005.csv](logs/j4-hold-20261028-132005.csv), `logs/j4test2-*.log`)

| j4 target | model τ4 | err run 1 (kp 10) | **err run 2 (kp 22)** | kp·err run 1 | kp·err run 2 | measured τ4 run 2 | rotor °C run 2 |
|---|---|---|---|---|---|---|---|
| 0.0 | 2.378 | +0.0937 | **+0.0484** | 0.94 | 1.06 | 1.971 | 42→51 |
| −0.3 | 2.560 | +0.1091 | **+0.0503** | 1.09 | 1.11 | 2.187 | 51→55 |
| +0.3 | 1.984 | +0.0412 | **+0.0252** | 0.41 | 0.55 | 1.622 | 54→50 |
| +0.7 | 1.191 | +0.0151 | **+0.0081** | 0.15 | 0.18 | 0.995 | 50→47 |

`home`: done, max error **0.0319** (run 1: 0.0616). Shutdown 13:29: daemon, coordinator, `disable_all`: all 7 disabled, no faults; j4 rotor 48 °C.
- **Result:** j4's holding error **halved in every pose** (worst case 0.109 → 0.050 rad). `ready` now verifies with 0.07 rad of margin under the 0.12 wrist tolerance.
- **Insight:** kp·err (the spring's share of the torque) is about the same in both runs (≈1.0–1.1 N·m at the heavy poses). So the error is a **fixed torque surplus from the gravity feedforward**, and the error scales as 1/kp. This confirms run 1's reading: the model's j4 gravity is about 1 N·m (≈ 40%) too strong at the heavy poses. (The script's printed `kp*err` column assumed kp 10; it now takes `--kp4`.)
- **Cost:** j4 runs about 5 °C hotter while holding (peak 55 °C rotor vs 49 °C). That's acceptable for a DM4310, but long holds at heavy poses warm it, so watch it during open play.
- **Decision:** real-arm runs use `--kp j4=22 --kd j4=2` from now on (HANDOVER §5 start line updated). The root fix (scale the j4 gravity feedforward ≈0.6–0.8, or correct the gripper/camera mass in `yam_gripper_gravity.urdf`) is the next DimOS-side step.
- No hum or chatter was reported by Naman during the run.

### 13:38–13:56 Phone on the real arm (HANDOVER §5 step 4), then dance caps; Naman on the phone, a second person supporting the arm
Real coordinator (`--kp j4=22 --kd j4=2`), daemon (0.3 rad/s), `openplay.server --daemon ws://127.0.0.1:8091/armctl --auto-play off --turn 600`, all on the real bus ([logs/phone-real-*.log](logs/), [logs/openplay-real.log](logs/openplay-real.log)).
- **Observation:** Sticks and tilt drove the real arm. From `ready` (j2 0.46), driving down or toward the player sinks j2 below 0.3 within about a second. After that **every** Cartesian stream is refused (`Cartesian streaming needs the arm unfolded (j2 0.20 < 0.3)`, 252 refusals), even ones that would raise it, so the phone felt "unresponsive". No position limit was crossed: closest j2 0.203 against soft 0.10, other joints ≥0.5 rad from theirs. Recovery: Ready.
- **Fix (phone, `af04279`):** a "too low to steer" bar with a Ready button on that refusal; live controls moved to the bottom by the thumb; the hold buttons are now touch `div`s (iOS long-press swallowed `<button>` holds: Naman reported the dock hold "doesn't work").
- **Idea:** a higher drive-start pose than `ready` (more j2 headroom), and/or let the daemon accept Cartesian streams that *raise* j2 below the threshold, so the player can recover without Ready.
- **Problem:** library moves were refused at Play ("playable path leaves the envelope: j1 velocity 0.563 > cap 0.300"). The dance moves declare `caps: "dance"` (0.6 rad/s) and pass validation against that preset, but the daemon on the real arm enforces 0.3. **Fix (`0cdef77`):** `openplay.server.fit_to_caps` re-tags such moves to the daemon's caps and `playable.retime`s them (robot-dance 17.6 s → 27.5 s, disco-point 18.9 → 34.2 s); the phone says "slowed to fit". Wave hello and the shimmy played on the real arm at 0.3.
- **Observation:** auto-play was switched on from the display rail at 13:49:56; the open play restarts put it back off.
- **Decision (Naman):** run at dance caps. Home, everything stopped, `disable_all` (all disabled, no faults; j4 rotor 54 °C, j3 49 °C), then the coordinator with `--traj-vel-limit 0.7` (its 0.5 default would clip 0.6 moves) and `--kp j4=22 --kd j4=2`, the daemon with `--vel-cap 0.6 --acc-cap 2.0`, open play with auto-play off. Cartesian phone driving stays at the coordinator's 0.3 joint-speed cap. Watch j4 temperature (pause above ~65 °C).

### 14:00–14:35 Wrist-camera exploration, j1 FAULT (arm went limp), limits fix, phone demo
Daemon at dance caps (0.6/2.0), coordinator `--traj-vel-limit 0.7 --kp j4=22 --kd j4=2`. Tool: [scan_record.py](scan_record.py) records wrist-camera frames (deduplicated, 10 Hz from the console's `/camera.jpg`) with the joint state and FK tip at each frame. Frames are local only (people in view, `scan*-local/`, git-ignored).
- **Scan 1** (7 "fingers-down" IK poses, 575 frames): every frame showed the **arm's own base** between the jaws, at the same image spot regardless of j1. **Insight:** the wrist camera's optical axis sits ~55° "back" from the fingers (in the arm's bending plane): fingers-down → camera looks down-and-back at the base. Also: fingers-straight-down is reachable only up to tip z ≈ 0.20–0.22 m (j4 soft limit).
- **Scan 2** (joint-space panorama, j1 −1.39..+1.39, two wrist tilts, 1065 frames): **the arm sits at the edge of the platform facing away from it.** j1 ≈ 0 looks over the edge at the pavilion floor (people on cushions). The white-cloth platform lies to the arm's right (−40°..−80°) and left (+40°..+80°). **The white paper cup is on the right at j1 ≈ −60°**, near a glass jar. A person working on a laptop sat at that edge of the table, inside reach.
- **Scan 3a** (6 close looks over the cup, camera ~straight down using the 55° offset, tip ≥17 cm above the cloth, 437 frames): the cup is clearly in view in all six, standing upright on the cloth, with the jar and a wooden board nearby.
- **Scan 3b** (behind the arm, high, j1 to −2.45 and +1.40..+2.95): **FAULT.** The j1 −2.10/−2.45 stops fell 0.18/0.53 rad short (cause unknown: cable or contact? not seen). Then going to +1.40 → +1.75: `internal error: Damiao feedback fault is latched: yam_joint1 reported 1.621 outside [-3.92699, 1.5708] by more than 0.05 rad`, and **the arm went limp** (Naman: "you killed the power"; the supporter caught it; it ended folded with j1 at +1.61). Shutdown + `disable_all`: all disabled, no motor fault codes.
- **Problem / cause:** armctl's joint limits came from `i2rt/yam.urdf` (j1 up to +3.142, soft +3.042). The DimOS Damiao adapter on the real arm takes its limits from `urdf/yam_gripper_gravity.urdf` (j1 [−3.927, **+1.571**], j6 [−2.356, **+1.833**]) and latches a feedback fault, dropping torque, when a reading is >0.05 rad past one. The scan planner trusted the armctl soft box. Exactly the CLAUDE.md trap ("DimOS does not check joint limits on raw trajectories… the adapter faults and the arm goes limp").
- **Fix (`75b632d`):** `armctl/envelope.py` hard limits = **i2rt ∩ DimOS-adapter** limits (`I2RT_URDF_LIMITS`, `DIMOS_ADAPTER_LIMITS`, `URDF_LIMITS` = intersection): j1 max 1.571 (soft 1.471), j4 min −1.658 (soft −1.558), j6 max 1.833 (soft 1.733). All 17 phone moves still validate. 3 tests that hardcoded old values were updated; full suite green.
- **Fix (`d5df1a8`):** `OPENPLAY_NO_CARTESIAN=1` refuses Tilt/Sticks streams, because Pink IK uses the looser model and could still drive j1 past +1.571. Joints/Moves/Say stay on.
- **Decision (Naman: "let's not do that ever again… a harness that makes sure it never just goes limp… with a fault if it could maintain old position"):** background agent building layer 2 (last-line gate in DimosBackend against the adapter limits), layer 3 (live watchdog stopping Cartesian streams near a limit), layer 4 (latched fault → E-STOP + "ARM FAULTED, support it" banner on phone and display), and researching whether DimOS can hold position instead of going limp on a feedback fault.
- Naman turned the base back to ~0 by hand (motors off). Probe clean. Restarted at 0.3 caps with the new limits + no-Cartesian. **Phone demo** (Moves/Joints/Say it) on the real arm: done, no incidents.
- 14:35 `home` (done, no faults), shutdown, `disable_all`: all 7 disabled, no faults. Gripper left closed (raw −0.011).
- **Next:** merge the harness, re-enable Tilt/Sticks; pick up the white cup at j1 ≈ −60° with the wrist-camera pipeline (agent running) using the scan-3a frames + poses as the first data.

### 14:35 DimOS adapter-limit harness after the j1 +1.62 fault (code only, no hardware)
Environment: Linux laptop, `.venv` Python 3.12, DimOS 0.0.14, branch based on `5dba1fc`; unit tests with fake links only (no DimOS process, no zenoh, no CAN). The real arm was in a live session on 224.0.0.123:7449 and was not touched.
- **Problem (13:5x):** a scan sent j1 to +1.62. armctl allowed it (its i2rt limit +3.142, soft +3.04), but the Damiao adapter takes its limits from `yam_description/urdf/yam_gripper_gravity.urdf` (j1 [-3.92699, 1.5708]) and latched `Damiao feedback fault is latched: yam_joint1 reported 1.62 outside [-3.927, 1.5708]`. The arm went limp; the daemon only showed `internal error: ...`.
- **Fix, five layers:**
  1. `envelope.URDF_LIMITS` = i2rt ∩ `DIMOS_ADAPTER_LIMITS` (soft j1 upper now 1.471, j4 lower -1.558, j6 upper 1.733).
  2. `armctl/dimos_limits.py`: re-reads the installed model file (`~/.local/share/dimos/repo/data/yam_description/urdf/yam_gripper_gravity.urdf`, resolved the way `dimos.utils.data` does it). `DimosBackend` refuses to start on a real adapter when that file is tighter than the pinned values. `python -m armctl.dimos_limits` → `OK` today.
  3. Last-line gate in `DimosBackend` (`dimos_gate`): every trajectory, braking path and joint-stream step must stay 0.03 inside the DimOS limits (folded stops j2/j3: down to -0.02). Otherwise RuntimeError naming joint, value and limit, and nothing is sent.
  4. Edge-of-reach watchdog: while a stream is live, a measured joint within 0.10 rad of its limit and moving toward it stops the stream. For twists the limit is the tighter of DimOS and the soft box; joint streams are already braked at the soft box, so they use the DimOS limit. That direction stays latched until the joint is 0.12 back; moving away is allowed. The client gets `edge of reach: j1 near its limit (1.38 / 1.47): steer back` (stream refusal, once a second).
  5. Adapter faults (text `feedback fault`) from any RPC → `state.fault` (joint, reason, action), torque false. The daemon E-STOPs, logs ERROR, broadcasts `{"op":"fault"}`, and refuses acquire, motion, E-STOP reset and enable with `ARM FAULTED ...: support the arm, run tools/disable_all.py ...`. There is no auto-recovery. The openplay display and phone show an **ARM FAULTED** banner.
- **Check:** `scan_record.py`'s soft-box check now refuses j1 1.472 / 1.5 / 1.62 and accepts 1.40 / 1.47 (its panorama default j1_max is 1.4).
- **Insight (DimOS hold-on-fault, read from `dimos/hardware/whole_body/damiao/adapter.py`):** `_latch_feedback_fault` calls `deactivate()` (`robot.disable()`, which makes the arm limp). Every `read_motor_states` then raises, and `write_motor_commands` returns False (`_active` False). The coordinator keeps running and logs a read error every tick. `activate()` refuses while the fault is latched, and only `connect()` (a coordinator restart) clears it. `feedback_clamp_margin_rad` (0.05) is a class attribute; `DamiaoRuntimeConfig` has no option for it. Within the margin the adapter already clamps the feedback and keeps going.
- **Idea:** add a DimOS-side `feedback_fault_policy="hold"`: on a latch, keep the motors enabled and override every arm command with the positions measured at the latch (dq 0, same kp/kd, gravity feedforward), while still surfacing the fault and refusing new targets. This needs care for a glitched encoder reading. Not patched.
- Tests: `armctl/tests/test_dimos_harness.py` (12).

### 15:00 Wrist-camera pipeline merged; camera mounting measured from scan 1
- Merged `interfaces/wristcam/` (agent branch `f670ec5`): ChArUco lens + hand-eye + table calibration from one taped A4 sheet, a tag-free `CupFinder` (rim ellipse), and a closed-loop `GraspEngine` (search → centre and move closer → descend → close → verify via grip stall + post-lift look → retry once), with a JSON-lines decision log. Dry run by default; `--execute` asks before every move.
- **Problem:** the agent assumed the camera looks along the fingers and the board sits 30 cm in front of the arm. Both are wrong here (scan 1/2).
- **Insight / measurement:** from the 7 scan-1 poses (base mount always at pixel ~(660, 470)), the tip→base direction in the tip frame is a constant ≈ (0.9, 0, −0.4). Fitting a camera rotation about the tip's y axis plus image roll (the jaws must sit at the image bottom, opening left/right) gives **camera axis ≈ 60° off the finger axis toward tip +x, roll 90°, ~6 cm behind the tip**, with a median reprojection error of 54 px using the uncalibrated fisheye guess. Now `camera.T_TIP_CAM_NOMINAL`.
- **Fix:** `camera.finger_dir_for_cam_axis()` (camera straight down = fingers ~30° below horizontal, outward; j6 = 0 keeps the camera in the arm's plane). Used by `handeye.plan_views`, the engine's look poses (grasp poses stay fingers-down), and the synthetic tests (true mounting = measured nominal + 3° / few mm error). The camera-down pose over the cup solves to q ≈ [−1.06, 0.80, 0.64, −0.37, 0, 0], the same pose that saw the cup well in scan 3a. Also fixed a dtype bug in `handeye._in_jaws`.
- **Known limitation (xfail test):** with the calibration sheet still on the table, a camera-down search view sees a ~6 cm circle on the sheet as a cup. **Remove the sheet before grasping**; a proper fix is to reject rims that overlap detected ChArUco markers.
- README real-arm procedure updated: sheet on the arm's RIGHT next to the cup (`--board-center 0.15,-0.25,0`), camera-down park pose over the cup.
- Tests: full suite 411 passed + the wristcam set (22 passed, 1 xfailed) after the roll-variety threshold went from 0.4 to 0.3 (big j6 rolls swing the board out of the tilted camera's view).

### 15:40–16:00 Wrist-camera calibration, first attempt (Naman taped the printed sheet on the arm's right, cup beside it)
Coordinator `--kp j4=22 --kd j4=2` (0.3 caps, new limits + harness), daemon `--auto-approve`; `WRISTCAM_BOARD=interfaces/wristcam/print/printed_vision_board.json`.
- **Observation:** the lab printed `vision/targets/charuco_board_A4.pdf` (7×5 squares, 29.97 mm, DICT_5X5_100), not the 9×6 wristcam sheet: the 7×5 layout fits the detected markers to 1.3 px vs 32 px for 9×6. The 50 mm bar read "close to 50", not yet measured exactly. → `print/printed_vision_board.json` plus the `WRISTCAM_BOARD` override in `board.py`.
- First look at j1 −1.06 showed the sheet obliquely. **Naman: "turn like 90 degs… in the direction of the cup"** → at j1 −1.50 (q [−1.50, 0.80, 0.64, −0.37, 0, 0]) the camera looks straight down at the sheet with the cup at its top edge ([calib-look86-local/look.jpg], local). In that pose image-up = world +y (toward the base).
- **Problem → fix:** OpenCV's default ArUco params decoded 6/17 markers (~15 px each at 1280×720). Tuned params (adaptive window 3–53/4, minMarkerPerimeterRate 0.01, polygonalApprox 0.05, errorCorrection 0.8, tryRefineMarkers) decode **17/17, 24/24 corners** → now in `BoardDetector`.
- Board centre ≈ (0.02, −0.15, 0) base (from the geometry under the camera). Planned + collected **20 intrinsics views** and **12 hand-eye views** (`yes y |` piped to the per-move prompts; 0.25 rad/s; 0 refusals/faults; arm returned to ready). Data: `interfaces/wristcam/data/` (git-ignored).
- **Intrinsics:** fisheye, RMS 0.63 px, fx 492.6, fy 489.3, cx 693, cy 372, "FOV 130°", but **coverage only 48% of image cells, the board never in the lower 2/5** (jaws) or the edges.
- **Hand-eye: REJECTED.** HORAUD/PARK: camera t = (−95, 30, 48) mm, optical axis **179° from the fingers**, table origin (0.12, −0.56, **−0.125**) m, reprojection 13 px, spread 16.5 mm. Physically impossible (the board is on the table ~0.2 m away; the camera is ~60° off the fingers). It had already written `workcell.json` as `calibrated: true`: **reverted immediately** (backup `workcell.before-handeye.json`; bad result kept as `calib/handeye.rejected-155702.json`).
- **Insight (diagnosis):** with the calibrated intrinsics, PnP puts the board **0.56–0.71 m** from the camera in the 12 views; the real distance is ~0.3–0.4 m. With the measured 60° nominal mounting the per-view board centres still scatter by 10–16 cm. The board's apparent size (210 mm ≈ 190 px at ~0.35 m) implies f ≈ 300–330 px, not 492. **The intrinsics are wrong** (focal length/distortion trade-off from poor edge coverage), and hand-eye inherits it.
- **Decision:** re-collect intrinsics **hand-held**: the arm parked camera-down and still, a person moving the sheet across the whole view (edges, corners, near/far, tilted). Then re-solve hand-eye on the existing 12 views, and only accept it if it passes physical sanity checks (axis ~60° from the fingers, board z ≈ 0 at ~0.2 m, spread < 5 mm).
- **Idea:** `handeye solve --write-workcell` should refuse to mark `calibrated: true` when the reprojection is > 2 px, the spread > 5 mm, or the table is not near z = 0.

### 16:10 Setup photos from Naman; the camera is a bracket camera; site limit for the glass
- **Observation (photos, `media/setup-*-local.jpg`):** the arm is mounted on a wooden board at the back of the table, **facing the room**. Directly **behind it is a glass wall with wooden posts**. The sheet + cup are on the table's left as seen from the room (= the arm's right); the ArUco tag sheet is on the other side.
- **Correction:** the wrist camera is **not** between the fingers. It's a small white USB camera (4K module, M12-style lens) on an **angled aluminium bracket on top of the wrist**, about 10 cm back from the fingertips and a few cm above the gripper, looking diagonally past the fingers. That's consistent with the measured ~60° offset (the jaws are seen from behind, at the image bottom). The ~6 cm nominal translation is a guess; hand-eye will measure it.
- **Insight:** the two "behind" scan poses at 13:5x (j1 −2.10/−2.45, tip 15–23 cm behind the base) that stopped 0.18/0.53 rad short were most likely **pushing against the window frame / post**.
- **Fix:** `armctl/site_limits.json` (hard j1 ≥ −1.80, soft −1.70) intersected into `armctl.envelope` (only ever tightens; `ARMCTL_SITE_LIMITS=''` disables; a root `conftest.py` disables it for tests; new `test_site_limits.py`). Daemon restarted, coordinator kept holding.
- Naman: "this is just today's setup, we can change it". **Idea:** move the base to the table centre so the arm can work all round, then drop the site limit.

### 16:10–16:20 Hand-held lens capture, started (collaborating with Naman at the table)
- Arm parked camera-down over the sheet: `move_joints` q [−1.50, 0.80, 0.64, −0.37, 0, 0] (measured [−1.49, 0.77, 0.66, −0.32]); camera ≈ (0.02, −0.22, 0.27) base. Image ↔ table: image right = +x (front edge of the table, the room side), image down = −y (far end of the table, away from the arm), image up = toward the base, image left = toward the glass.
- Tool: [hand_capture.py](hand_capture.py) grabs a console frame every 0.5 s, detects the board (tuned detector, printed 7×5 spec), saves a frame when the board is somewhere new (position/size/aspect), and keeps a 6×4 coverage map + `status.json` with the cells still missing. Output `interfaces/wristcam/data/intr2/` (git-ignored). It never commands the arm.
- **Naman: "I can only place it at different positions on the table while it is flat."** → **Decision:** flat placements across the table give image-edge/corner coverage; the 20 arm-driven tilted views (`intr1`) give tilt/distance variety; solve the lens from both together (flat views at one distance alone can't separate focal length from distance).
- Naman then started removing the tablecloth and moving the arm's base back. Recorder paused (12 frames saved during the rearranging; to be checked). Asked to fold home + power down before moving the base. Moving the base invalidates the table frame (re-measure) but not the lens or hand-eye data.

### 16:25–16:40 Table re-arranged (Naman): cloth off, base moved back, then rotated
- 16:25 Folded home, shutdown, `disable_all` (all disabled, no faults; j4 rotor 57 °C after the calibration runs) **before** Naman moved anything.
- 16:30 Photo (`media/setup-bare-table-local.jpg`): bare wooden table, sheet taped on its left part, base on its board at the right end against the glass, wooden post on the right. **Check:** folded, the fingers point along base +x (FK at q=0: tip (0.211, 0, 0.174), approach (1, 0, 0)), and the folded gripper pointed at the **post**, so the table was *behind* the arm, mostly outside j1's range (DimOS j1 ∈ [−3.93, 1.57], armctl soft [−2.52, 1.47]).
- **Decision (Naman: "so that the gripper points towards the sheet?" Yes):** rotate the whole base so the folded gripper points down the table at the sheet. Forward = the table; glass on the arm's right (−y); post behind.
- **Site limit replaced (temporary, conservative):** `armctl/site_limits.json` j1 hard [−0.7, 1.3] → soft [−0.6, 1.2] until a look-around maps the glass. Old limits saved as `site_limits.before-rotation.json`. Moving the base invalidates the table frame, but not the lens (`intr1`) or hand-eye (`he1`) data.
- Probe after the rotation: all joints ≈ 0 (j1 0.074), no faults, j4 rotor 47 °C.

### 16:45–17:10 Wrist-camera re-analysis, offline (subagent; no arm, camera or DimOS touched)
Laptop, `.venv` Python 3.12, OpenCV 4.14.0, worktree from `4c175d4`; data copied read-only from `interfaces/wristcam/data/{intr1,intr2,intr3,he1}` (intr3 = 15 hand-held flat frames at the time). `WRISTCAM_BOARD=print/printed_vision_board.json`. Script: [wristcam_reanalysis.py](wristcam_reanalysis.py).
- **Observation (intrinsics per subset, ≥ 8 corners/view):**

  | subset | views | cells / radius | fisheye fx, RMS | rational fx | pinhole fx |
  |---|---|---|---|---|---|
  | intr1 (arm) | 18 | 52% / 84% | 485, 0.71 px (hfov 131°) | 488 | 486 |
  | intr3 (flat, hand) | 14 | 60% / 59% | 630, 1.03 px | 955 | 957 |
  | intr2 (flat) | 12 | 25% / 72% | fails | 1484 | 1407 |
  | he1 | 12 | 18% / 52% | 567, 0.21 px | 571 | 560 |
  | intr1+intr3 | 31 | 72% / 84% | 541, 0.79 px (hfov 122°) | 554 | 550 |

  Fewer terms (fisheye k1k2 / k1, pinhole fixed aspect, rational fixed aspect): intr1 483–517, intr1+intr3 545–562. Flat-only sets are degenerate (one distance, no tilt) and should never be solved alone.
- **Insight: the focal length was not the main error.** fx ≈ 490–560 is self-consistent across every model and subset that has tilted views, and an equidistant lens with the stated ~130° horizontal FOV has f ≈ 640/1.13 ≈ 565 px; f ≈ 300 would mean > 200°. The "≈190 px for 210 mm at 0.35 m" check assumed the board is 210 mm wide. **It isn't:** in `he1/view_02.png` the 7×5 board covers only ~2/3 of the sheet's width (homography from the detected corners onto the sheet outline: square ≈ 19–21 mm). Independently, a joint refinement of camera pose + board pose + **board scale** against the arm's FK (FK has metric scale) gives **square ≈ 22.4–22.9 mm** (±0.2, same for every lens candidate), not 29.97. Wrong scale → PnP distances 0.55–0.70 m instead of ~0.4 m → hand-eye translation/table garbage. The board was probably printed scaled (fit-to-page / 2-up).
- **Observation (hand-eye on he1, lens = intr1 fisheye):** with 29.97 mm every method (TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS, calibrateRobotWorldHandEye SHAH/LI) gives 13–20 px, 16–24 mm spread, table z −0.10…−0.19 m. All satisfy the bracket prior on the *axis* (11–17° from nominal); the "179°" in the 15:40 entry was the full rotation vs nominal, i.e. the **image roll is 180° from the nominal guess** (the solved pose puts the tip at pixel ~(715, 540), between the jaws, which matches the frames). With square 22.9 mm: RWHE-SHAH+refined 4.4 px, spread 5.2 mm / 1.19°, camera t = (−59, 12, 41) mm (7.3 cm from the tip), optical axis 46° from the fingers, table origin (0.108, −0.468, −0.038). Full 12-parameter + scale refinement: 3.98 px, view 11 an outlier at 8.9 px; q_planned instead of q_measured: 14 px (measured q matters, j4 sags ~0.04 rad).
- **Gate verdict: still REJECTED** (reprojection 4.4 > 3 px, rotation spread 1.19 > 1°, table z −0.038; −0.038 may be real if the base sits on its wooden plate: measure it and pass `--table-z`).
- **Fix (code, worktree branch):** `handeye solve --write-workcell` refuses unless every sanity gate passes (prints each gate; failed results go to `calib/handeye.rejected-<time>.json`, never `handeye.json`); `calibrateRobotWorldHandEye` (SHAH, LI) added as candidates; bracket prior (axis ≤ 25° from nominal, camera ≤ 15 cm from the tip) filters candidates; `solve` always estimates the board scale from kinematics and gates on |x−1| < 5% (PnP-vs-FK disagreement = wrong square size); `WRISTCAM_SQUARE_MM` env sets the measured square for every tool; `plan_views` hand-eye default 15 views, tilt 10–35° toward golden-angle headings, alternating j6 roll 0.15–0.6 rad, `rotation_diversity` score. Synthetic tests for all of it.
- **Decision / next data needed:** (1) **measure one printed square with a ruler** (and the base plate height) before any further solve; (2) re-collect hand-eye with the new planner (≥ 15 rotation-diverse views, sheet 0.25–0.35 m out in the new table layout); (3) more tilted lens views with the board in the image corners and the lower band (arm-driven `--purpose intrinsics` plus hand-held *tilted* placements, not only flat); (4) a look at the jaws to confirm the image roll and `JAW_POLYGONS`. `camera.T_TIP_CAM_NOMINAL` roll should be flipped 180° once hand-eye passes.
- **17:15 Coordinator's hypothesis checked (square ≈ 15.8 mm, from PnP 0.58 m vs "FK ~0.31 m" with the fx 537 combined lens):** he1 with intr1+intr3+intr4 fisheye (46 views, RMS 0.89, fx 549 fy 539) and with the coordinator's fx 537.1 lens:

  | lens | square | best method | reproj | spread | table z | kinematic scale fit |
  |---|---|---|---|---|---|---|
  | combo (mine) | 29.97 | ANDREFF+ref | 10.4 px | 20.2 mm / 1.70° | −0.094 | 22.49 mm |
  | combo (mine) | 22.9 | RWHE-SHAH+ref | 4.8 px | 7.2 mm / 1.70° | −0.052 | 22.49 mm |
  | combo (mine) | 15.8 | RWHE-SHAH | 21.7 px | 12.6 mm / 1.70° | +0.007 | 22.49 mm |
  | fx 537 | 29.97 | HORAUD | 14.5 px | 19.2 mm / 1.43° | −0.137 | 22.42 mm |
  | fx 537 | 22.9 | RWHE-SHAH+ref | 4.4 px | 6.7 mm / 1.43° | −0.044 | 22.42 mm |
  | fx 537 | 15.8 | RWHE-SHAH | 21.1 px | 12.5 mm / 1.43° | +0.014 | 22.42 mm |

  **Insight:** the scale fit converges to 22.4–22.5 mm from every start and lens; 15.8 mm fits 5× worse. The "FK ~0.31 m" distance assumed the nominal camera position (unmeasured offset, roll 180° off), so a ratio of 1.9 is not a measurement; 29.97/22.45 = 1.33. Still only a fit: **a ruler measurement decides.** Every row fails the gates (all > 3 px, rotation spread > 1°).
