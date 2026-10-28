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
