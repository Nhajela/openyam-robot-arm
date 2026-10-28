# Experiment: more ways to make the arm move

**Experimenters:** Naman (with Claude Code)
**Started:** 2026-10-28
**Status:** active. Everything here ran in the mock simulation and (from 10:30) DimOS `--simulation`. Nothing has touched the real arm yet. Start the hardware session from [HANDOVER.md](../../HANDOVER.md).

## Goal

Treat every way of moving the arm as an *interface* on top of one safety layer (armctl), and prototype the next ones in the sim, ready to try once the arm is connected again:

1. **Pose mirror:** a camera watches a person and the arm copies their arm (elbows and posture) in real time.
2. **Record + timeline:** record what the arm does, edit it as a choreography timeline, and have the computer say what the arm can't do ("j3 can't move this far this fast").
3. **Grasping:** a top-down pick and place we can trust, then chess on top of it.
4. **Open play** (added at 10:15): a public station. People scan a QR on the laptop, queue, and drive the arm from their phones. The laptop screen is both the show and Naman's operator dashboard.

Music sync (`armctl/choreo.py`) and English → motion (`armctl/llm_author.py`) from 16 Oct are the same kind of thing, a timeline interface. Code: [`interfaces/`](../../interfaces/README.md).

## Environment

- Machine: serenity (Ubuntu 24.04.5 LTS, Linux 6.8.0-147, x86_64, no GPU, no camera, no CAN adapter). This is a server, not the lab laptop.
- Repo `Edge-City/openyam-robot-arm` at `d4a5c10` (PR #2 merged), cloned to `~/src/openyam/openyam-robot-arm`.
- `.venv`: Python 3.12.3 via `uv venv`. **No DimOS** (not needed: armctl's mock backend is a kinematic sim). Packages: numpy 2.5.4, mediapipe 1.2.0, opencv-contrib-python-headless 5.0.0.93, chess 1.11.2, fastapi, uvicorn, websockets, pytest.
- Mock daemon: `.venv/bin/python -m armctl.daemon --backend mock --auto-approve --port 8191 --vel-cap 0.6 --acc-cap 2.0` (log: [logs/mock-daemon.log](logs/mock-daemon.log)).
- From 10:20: DimOS 0.0.14 in a separate venv `~/src/openyam/dimos-venv` (`uv pip install --torch-backend cpu "dimos[manipulation]==0.0.14"`, 6.8 GB). It needed `sudo apt install libgl1 git-lfs` on the server; the DimOS data (yam_description URDF) downloaded to `~/.local/share/dimos/repo/data`. `qrcode` and `anthropic` (2.3.0) added to `.venv`.

## Log

### 2026-10-28 09:45 IST: setup

- **Problem:** `mediapipe` installs `opencv-contrib-python` (GUI build) next to the headless one, and `import cv2` then failed with `ImportError: libGL.so.1`. **Fix:** uninstall `opencv-contrib-python` and `opencv-python`, reinstall `opencv-contrib-python-headless`. `cv2.aruco` still works (5.0.0).
- **Problem:** the MediaPipe Tasks pose landmarker failed with `OSError: libEGL.so.1: cannot open shared object file`. **Fix:** `sudo apt install libegl1 libgles2` (server only; desktop Ubuntu has them).
- **Observation:** mediapipe 1.2.0 has no `mp.solutions` any more, only the Tasks API (`mediapipe.tasks.python.vision.PoseLandmarker`) plus a model file (`pose_landmarker_lite.task`, 5.8 MB, auto-downloaded to `~/.cache/openyam/`).
- Existing tests: `pytest armctl/tests` gave 68 passed, 1 skipped.

### 09:50: kinematics without pinocchio (`armctl/kin.py`)

- **Decision:** FK/IK in plain numpy, using the URDF joint table already copied into the playground (`src/lib/arm/kinematics.ts`). No pinocchio and no DimOS data dir needed, so it runs anywhere.
- **Observation:** FK home tip (0.211, 0, 0.174), ready tip (0.205, 0, 0.432): exactly the pinocchio values.
- **Insight (joint meanings, from FK):**
  - j1: +j1 swings the tip to +y (the arm's left).
  - j2: the upper arm's angle from "flat, pointing back" (π/2 vertical, ~π flat forward).
  - j3: the elbow opening (0 folded, ~π straight).
  - j4: +j4 tilts the fingers up.
  - j6: turns the fingers about the approach axis.
- **Observation, top-down reach** (fingers straight down, IK with soft limits):

  | tip height | reachable x |
  |---|---|
  | ≤ 10 cm | 0.10–0.52 m |
  | 15 cm | 0.10–0.46 m |
  | 20 cm | 0.14–0.40 m |
  | 30 cm | none (j4 hits its −1.59 soft limit) |

  So grasping happens low and within about half a metre.

### 09:55: pose mirror (`interfaces/mirror/`)

- `PoseTracker` (MediaPipe, VIDEO mode) gives 33 landmarks in metres. `SyntheticBody` makes a fake person waving, so everything runs with no camera.
- **Decision:** mirror convention. The arm faces the person like a reflection: a hand on the image's left sends the tip to the image's left.
- **Joint mapping** ("joints" mode):
  - j1 = −0.8 × upper-arm yaw (held while the arm points nearly straight up or down)
  - j2 = 3.0 → 1.2 as the arm goes from hanging to overhead
  - j3 = 0.95 × elbow opening
  - j4 = wrist bend (optional)
  - "hand" mode instead maps the wrist position into a box in front of the arm and solves IK.
- Test photo (MediaPipe's `pose.jpg`, a person in a warrior pose, arms level):
  - Both arms were found, visibility ≥ 0.98.
  - Right arm: elev 1.61 rad, yaw −1.71, opening 2.68, giving target q = [1.36, 2.08, 2.55, 0, 0, 0].
  - **Observation:** the arm swings 78° to its left with the upper arm about 60° up and the elbow nearly straight, which reads as that pose. Not committed (third-party photo).
- First live run on the mock: `interfaces.mirror.run --source synthetic --url ws://127.0.0.1:8191/armctl --duration 14 --record ...`. **Observation:** the arm follows with about 0.5–1 s lag at 0.6 rad/s caps. The elbow (j3 swings of about 1 rad) lags most.
- **Problem:** every *target* was checked for table clearance, but the recording had **7 of 163 samples** too close to the table (worst: tip 2.6 cm under the 8 cm rule at q = [1.40, 2.41, 1.74, 0, 0, 0]). Each joint servos on its own, so the path *between* two safe targets can dip. **Fix:** `servo.table_guard` predicts 0.3 s ahead and zeroes the joint velocities that take the arm down. Rerun: **0 of 139 samples under**, minimum margin 5 mm.
- **Idea:** `--clutch` (follow only while the other hand is raised) as a deadman gesture for the first real-arm tries.

### 10:00: timeline + coach (`interfaces/timeline/`)

- **Coach:** per-segment reasons in words (cubic ease: peak velocity 1.5·d/T, peak acceleration 6·d/T²), plus a per-keyframe "reachable band", an FK table check, and fits that either *stretch* the time or *shrink* the moves.
- Test: `wave` sped up 2.5×. **Observation:** 9 of 9 segments are flagged, e.g. "0 → 0.74 s is too fast for j2, j3, j4: needs at least 1.50 s (has 0.74); in 0.74 s j3 can move at most 0.09 rad". After Stretch it's valid at 10.9 s; after Shrink it's valid and keeps 4.68 s. Screenshot: [media/timeline-wave-too-fast.png](media/timeline-wave-too-fast.png).
- **Recording → playable:** keep the pauses, refine until within 0.04 rad, then fit. A 10 s mirror recording became 32 keyframes and a 22 s playable (dance caps), max deviation 0.038 rad.
- **Insight:** the playable format only has `cubic` (stop at every key), `linear` and `hold`, so a flowing recorded move becomes stop-and-go and about 2× longer. **Idea:** a spline ease (Catmull-Rom through the keys, zero velocity only at the ends) in spec 1.1. It has to go into `armctl/playable.py` and the playground's spec, simulator and validator together.
- End to end on the mock, from the editor's API:
  1. Record start.
  2. The mirror drives the arm for 10 s.
  3. Record stop gives a valid playable.
  4. "Send to arm" gives `done playable:rec-test (max error 0.0000 rad)`.

### 10:05: grasp + chess (`interfaces/grasp/`)

- `pick_place` plans: ready → above (open) → down → close → lift → above target → down → open → lift → ready. IK has the fingers pointing down and j6 found by a numeric search so the fingers close across the object (error < 0.05 rad).
- **Decision:** a playable may carry `"min_tip_z"` (an unknown field is allowed by spec 1.x) for moves that touch the table on purpose. The coach then uses that floor for the tip only.
- `chess_arm check` with the default board (3.5 cm squares, centred 0.30 m in front of the base): every square is reachable. The nearest is a8 at x = 0.177 m, the furthest h1 at 0.422 m.
- Played `... f3e5 c6e5` (black knight takes on e5) on the mock: the white knight goes to the graveyard (41.8 s), then the black knight c6 → e5 (39.7 s). Both `done` with 0.0000 rad error.
- **Observation:** at the default caps (0.3 rad/s) one pick and place is about 40 s, mostly the ready → above-board moves. At dance caps it would be about 20 s. Skipping "ready" between the two halves of a capture would save about a third.

### 10:10: tests

`pytest interfaces/tests armctl/tests` gave **95 passed, 1 skipped** (27 new tests: FK reference values, IK round trips, mirror signs and safety on every synthetic frame, clutch, servo caps, table guard, coach formulas, both fits, recording → playable, pick/place validity and finger yaw, chess captures/castling/en passant, board geometry).

## Next (needs the lab laptop and the arm)

1. **Mirror with a real camera, still in the sim:** USB camera or phone, `--source phone:front`. Tune the mapping and smoothing on real people; measure the end-to-end lag.
2. ~~**`DimosBackend`**~~ done in sim (see 10:30 below). Next is a supervised real run from `ready` (joystick, then the mirror with `--clutch`), with the daemon at 0.3 rad/s and the person outside the arm's reach.
3. **Calibrate the table frame:**
   - Touch the tip to tag 0's centre and two marks 10 cm along its axes.
   - Get their FK, then `fit_table_frame` and set `calibrated: true` in `interfaces/grasp/workcell.json`.
4. **Measure the gripper:** grip 0..1 → width, where the pads sit relative to `gripper_tip`, whether it stalls on an object (`holding`), and the sag at full reach.
5. **First real grasp:** a foam block from a taped spot, then one chess piece, then a capture.
6. Camera-based board reading (which square changed) so the human doesn't have to type their move.

## 2026-10-28 10:30-11:00 IST: DimosBackend against DimOS --simulation

Step 2 of "Next": `armctl.backend.DimosBackend` now drives a running `armctl.dimos_coordinator`, so the daemon (and every phone, camera and open-play interface on it) can drive DimOS instead of only the mock. **Simulation only** (mock_whole_body adapter). Nothing ran on hardware, and no CAN interface was named anywhere except the fake `nonexistent_can9`.

**Environment:** serenity (Ubuntu 24.04.5, Linux 6.8.0-147, 6 CPUs, no arm, no CAN adapter). Repo at `08690fb` plus uncommitted `armctl/backend.py`, `armctl/dimos_coordinator.py`, `armctl/tests/{test_dimos_backend,e2e_dimos_sim}.py`. DimOS 0.0.14 in its own venv `../dimos-venv` (Python 3.12.3, zenoh 1.10.1, numpy 2.5.4, fastapi 0.162.0, websockets 16.1.1; nothing installed). Everything ran on the sim bus `224.0.0.124:7450`.

**Commands** (repo root; three terminals):

```bash
ZENOH_SCOUT_ADDR=224.0.0.124:7450 ../dimos-venv/bin/python -m armctl.dimos_coordinator --simulation
ZENOH_SCOUT_ADDR=224.0.0.124:7450 ../dimos-venv/bin/python -m armctl.daemon --backend dimos --auto-approve --port 8791 --log ''
ZENOH_SCOUT_ADDR=224.0.0.124:7450 ../dimos-venv/bin/python armctl/tests/e2e_dimos_sim.py      # 27 checks + numbers
.venv/bin/python -m pytest -q armctl/tests interfaces/tests                                    # no DimOS needed
```

Logs: [logs/dimos-backend-e2e.log](logs/dimos-backend-e2e.log) (final run), [logs/dimos-backend-daemon.log](logs/dimos-backend-daemon.log), [logs/dimos-sim-coordinator.log](logs/dimos-sim-coordinator.log) (with the "Pink streaming command envelope is empty" lines filtered out: about 100 per second while the arm sits at folded home).

- **Problem / Fix (bus):** the docstrings and the default in `dimos_coordinator.py` used `224.0.0.123:7449` for sims. CLAUDE.md says that is the REAL-arm bus. Now an unset `ZENOH_SCOUT_ADDR` defaults to the sim bus 224.0.0.124:7450, `--simulation` refuses the real bus, and `--real` needs `ZENOH_SCOUT_ADDR=224.0.0.123:7449` exported explicitly. Checked: `--real --can-port nonexistent_can9` on the sim bus or with the bus unset → "--real needs the REAL-arm bus explicitly exported"; `--simulation` on 7449 → refused. `experiments/2026-10-16-linux-real-arm/armctl_sim.py` and `armctl_blueprints.py` still default to 7449 for sims. They are historical scripts and were left as they are: pass the sim bus explicitly if you rerun them.
- **Decision (connect + verify):** the backend picks its bus before any dimos import. Unset means the sim bus, the LAN default is refused, and the real bus is refused unless `ARMCTL_DIMOS_EXPECT=real`. It then calls `armctl_info()` and refuses to start unless all of these match: launcher, exactly the three tasks (name, type, priority, arm joints), IK `command_limit_margin` ≥ the daemon's soft margin, IK joint speed ≤ the daemon's caps, the bus, and sim vs real. The result is logged. `state()["adapter"]` is `"sim"` | `"real"` (and `adapter_type`), and the daemon's 20 Hz state carries it. Checked live: `--margin 0.15` → "eef twist command_limit_margin 0.1 < armctl soft margin 0.150"; no coordinator → "no DimOS coordinator answered on bus 224.0.0.124:7450"; real bus with expect=sim → refused before dimos is imported.
- **Observation (DimOS 0.0.14 internals that shaped the design):**
  - eef twists are integrated in the base frame (`matrix[:3,3] += v*dt`, `exp3(w*dt) @ R`), which is armctl's convention.
  - The trajectory task has no `set_estop`. The coordinator's `set_estop` latches only the twist and gripper tasks, so E-STOP = `set_estop(True)` + `cancel_trajectory()`.
  - `execute_trajectory` while one runs continues from the commanded position (anchoring), but restarts the clock at `elapsed=0` on its first tick, so that tick holds.
  - `JointTrajectory.sample` is a linear scan, run once per joint per tick: 100 Hz points for a 60 s playable would be 6000 points × 6 joints × 100 Hz.
  - `RpcCall.__call__` keeps one closure per call forever, with a 120 s default timeout. The backend calls the transport's `call_sync` itself, with 1 s timeouts.
  - `/coordinator_joint_state` arrives at ~100 Hz in another process. The sim reports zero velocity, so `qd` is a 50 ms finite difference.
  - The `arm/gripper` joint is already a calibrated 0..1 opening, in sim and on the real Damiao adapter (`can_motor_control.Gripper(opening_direction="decreasing_position")`).
  - DimOS's velocity task emits VELOCITY commands, which the whole-body adapter drops, so it can't serve joint streaming.
- **Decision (mapping):**
  - `execute` decimates the daemon's gated samples to a piecewise-linear path within 0.5 mrad, with gaps ≤ 0.1 s and always ≥ 2 points (ready: 51 points from 501 samples, 0.23 mrad worst DimOS-sampled error). Completion is `get_status` at 20 Hz, then a settle of ≤ 1.5 s until within 0.01 rad, then `trajectory_done`.
  - `stop` brakes along the trajectory's own gated path at the accel cap.
  - Twists go out at 50 Hz from a 100 Hz deadman thread, which sends 3 zero twists at ttl expiry. Each twist is capped (0.1 m/s, 0.6 rad/s) and scaled with armctl's Jacobian so the joints stay under the caps, so the IK target can't run ahead. Twists are refused below j2 0.3 / j3 0.5.
  - Joint streams are rolling trajectories: follow the setpoint for min(ttl, 0.3 s), then brake to rest. Every plan ends at rest. A new one goes out when the setpoint changes, or when < 0.1 s of cruise is left.
  - `relax` is refused (no torque-off RPC). `torque` is True in sim, with a TODO for the real CAN status nibble.
- **Observation (numbers, final run 10:56, 27/27 PASS):**

  | what | measured |
  |---|---|
  | ready / move_joints / wave / home final error | 0.0000 rad (sim tracks commands exactly) |
  | move_joints tracking vs our 100 Hz plan | 0.7 mrad max, at a fitted start lag of 34 ms from sending `do` |
  | Cartesian +x 0.02 m/s for 2 s from ready | tip +43.2 mm in x, 0.0 in y/z, orientation change 5e-7 rad (+z: +33.6 mm z only) |
  | stream → first motion | twist 28-35 ms, joint stream 27 ms |
  | deadman (ttl 200 ms), last setpoint → still | twist 0.30 s (ttl + ~70 ms exponential IK decay), joint 0.38 s (ttl + braking 0.2 rad/s at 1 rad/s²) |
  | `do stop` → still | twist 0.09 s; trajectory 0.14 s, j1 overran 0.02 rad past the pose at stop (braking on the path) |
  | E-STOP while moving → still | 8 ms; latched, then after reset `ready` and gripper work again |
  | joint stream j1 0.2 rad/s | 0.19 rad/s mean over the cruise; 12.5 % of ticks held (one per re-issue), 3.9 mrad worst deviation from a straight line |
  | j5 streamed at its soft limit | stopped at 1.462-1.465 (soft 1.471), guard refusal sent to the client |
  | gripper 1.0 / 0.3 / 0.0 / 0.5 | reached each within 0.02 |

- **Problem / Fix (joint stream ripple):** the first version re-issued at 25 Hz from our own clock. Each re-issue held one DimOS tick, and our reference ran ~5 ms ahead, so the steps were 0 / 0.23 rad/s instead of 0.2 and the mean was 8 % slow. **Fix:** re-issue only on a setpoint change or when cruise runs low (about every 80 ms for a steady stream), take the reference from DimOS's own tick-quantised `time_elapsed`, and drop the plan's first step so the tick after the hold catches up. Per-tick pattern now `0, 3.9, 2.02, 2.02, ...` mrad (the mean holds; the hold itself is inherent to DimOS 0.0.14).
- **Insight:** in sim, every path works: discrete moves, playables, Cartesian and joint streaming, deadman, stop, E-STOP, gripper. The numbers that matter on hardware are not measured yet: PD sag (the settle step and the daemon's 0.02 rad verify), the held-tick ripple at kp 80, gripper direction, and torque state.
- **Problem (daemon, not changed here):** `do relax` comes back as "internal error: relax is not available…" with a traceback in the log, because the daemon maps only `Refused` cleanly. The DimOS E-STOP latch clears on the next allowed motion, because the daemon's `/estop/reset` does not call `backend.enable()`. The daemon's 0.02 rad execute-then-verify is one tolerance for every joint (SUMMARY: about 0.03 sag on j2/j3 and 0.06 on the wrist).
- **Tests:** `armctl/tests/test_dimos_backend.py` covers bus rules, coordinator verification, decimation, braking, rolling plans, the twist deadman, E-STOP latch/clear, gripper samples and the daemon on top of the backend. It uses a fake link and needs no DimOS: 19 passed, 1 skipped (the DimOS-message test, which was run by hand under dimos-venv). Whole suite: **157 passed, 2 skipped**.
- **Next (laptop, sim first):** the same three commands with `.venv` (which has DimOS there), then the phone joystick/tilt through phonecam against `--port 8791`. Then, only with the person at the arm, arm folded and probed:

  ```bash
  ZENOH_SCOUT_ADDR=224.0.0.123:7449 .venv/bin/python -m armctl.dimos_coordinator --real --can-port can0
  ZENOH_SCOUT_ADDR=224.0.0.123:7449 ARMCTL_DIMOS_EXPECT=real .venv/bin/python -m armctl.daemon --backend dimos
  ```

  Then `ready` from the console, then a short joystick test. Afterwards support the arm, Ctrl-C both, `tools/disable_all.py can0`, and check the LEDs.

## 2026-10-28 10:15-11:10 IST: open play (phones drive the arm, the laptop is the show and the admin)

**Goal (Naman):** people scan a QR on the laptop screen, get a turn (about 10 min) that locks the arm to them, and control it from their phone:
- tilt (gyro) or joysticks
- tap moves
- type what they want (Claude)
- record poses and play them back within the speed limits
- grab things

The laptop screen is always visible, so it has to be fun for the audience and a good admin dashboard for Naman. Local network first, for latency.

**Decision (architecture):** `openplay.server` is the only armctl program (`openplay`), holding the daemon lease and arbitrating players. Phones never talk to the arm. The contract was written first ([openplay/PROTOCOL.md](../../openplay/PROTOCOL.md)), then five parallel agents built against it, each owning disjoint files:
- core server
- phone page
- laptop display
- moves library
- pose recorder, grab spots and the real-Jacobian mock twist

The coordinator integrated and committed each piece separately.

- **Observation (moves):** 13 new playables via `openplay/tools/make_moves.py`, all cleared by validate + coach + FK (tip ≥ 10 cm above the table). The phone catalogue has 17 moves. `graceful-bow` never lowers the tip (FK: 0.43 → 0.67 m), so it doesn't read as a bow and a new `bow` replaces it (tip down to 0.16 m). `conductor`'s left-right stroke is only about 5 cm, because one 0.67 s beat at the dance caps limits j1.
- **Observation (pose recorder, `interfaces/timeline/poses.py`):** segment time = the cubic minimum × 1.25 (lively) or 1.9 (gentle), plus holds. It refuses, naming the pose, poses outside the box, near the table, or with a dipping path between them. "Route via ready" almost never rescues a dipping path, so those are refused with "add a pose in between, higher up".
- **Observation (mock twist):** `MockBackend.set_twist` now does DLS on the real Jacobian (`kin.jacobian_geometric`). +x 0.02 m/s for 0.5 s moves the tip about 2 cm in x with < 25 % cross-talk. Sideways (+y) at ready is slow: keeping the orientation needs j1 and j5 together, and the uniform cap scaling limits the tip.
- **Decision (approve = Play):** with auto-play off, the operator's Approve on the loopback display presses the daemon's Play, so there is one approval, not two. Auto-play defaults to on only for the mock backend.
- **Decision (laptop screen):** the show takes about 76 % of the width: arm in 3D, QR, now playing, queue, captions, and a Claude "say" card with a ghost preview. An always-on operator rail covers the rest:
  - health, E-STOP / reset, the approval card (Enter/Esc), auto-play, pause
  - skip, +1 min, turn length, kick
  - errors, joints with limit bars

  `h` hides the rail. A slot is reserved for a chess panel.
- **Integration (mock):** `armctl.daemon --backend mock` + `openplay.server` + display (1920×1080, URDF three.js view) + phone (390×844, HTTPS) through join, turn, Disco point, stop, and Say it "wave at the kids" (no key, so the library fallback plays Wave hello with the ghost preview). No page errors. Screenshots: [say + ghost](media/openplay-integration-say-urdf.png), [tall move](media/openplay-integration-tall-move.png), [phone](media/openplay-integration-phone-say.png).
- **Problem / Fix:** at full stretch the gripper left the top of the 3D view; the framing was widened.
- **Problem / Fix (server rule):** on serenity the first run bound the phone listener to `0.0.0.0:8450`, the server's *public* IP, for a few seconds, behind the room key. The house rule is no new public ports, so it was restarted at once with `--phone-host 127.0.0.1`. On the laptop `0.0.0.0` is right (phones on the LAN). Never run it that way on a server.
- **Integration (DimOS sim):** coordinator `--simulation --traj-vel-limit 0.7` + `armctl.daemon --backend dimos --vel-cap 0.6 --acc-cap 2.0` + openplay + phone. Auto-play came up **off** (non-mock), Celebrate waited for the operator, Approve played it as a 227-point DimOS trajectory, and leaving the turn returned the arm to ready.
- **Problem / Fix (safety UX):** the badge said "REAL ARM · DIMOS" on the DimOS *simulator*. hello/arm now carry the daemon's `adapter` (mock | sim | real). The badge says "simulated" only when the daemon reports sim; anything unknown on a non-mock backend is shown as real.
- **Problem / Fix (daemon, from the DimosBackend work):**
  - Execute-then-verify now uses per-backend, per-joint tolerances (DimOS: j2/j3 0.05, wrist 0.09, for the real arm's sag).
  - `/estop/reset` calls `backend.clear_estop()`.
  - `do relax` on a backend that can't relax is a clean refusal, not "internal error".
- **Tests:** `pytest armctl/tests interfaces/tests openplay/tests` gives **253 passed, 2 skipped**. openplay alone: 93 passed, including a 43 s e2e with two phones and a display against the mock daemon.
- **Insight (chess with a real set):** design in [interfaces/grasp/CHESS.md](../../interfaces/grasp/CHESS.md).
  - The arm has no camera; use an overhead phone or webcam.
  - We track the game from the start, so vision only has to tell which legal move happened (per-square change + legal-move matching).
  - Tap-to-move is the always-available fallback.
  - A 4-4.5 cm-square board fits the top-down reach.

### Server changes (serenity, for its CHANGELOG)
- apt: `libegl1`, `libgles2` (MediaPipe), `libgl1` (DimOS/open3d), `git-lfs` (DimOS data).
- `~/src/openyam/` (two clones + `dimos-venv`, about 7.5 GB), `~/.cache/openyam/` (pose model), `~/.local/share/dimos/` (DimOS data).
- No services, ports or config. Every test process was stopped.


## 2026-10-28 12:30-13:20 IST: chess board vision v1 (`interfaces/chess_vision/`), synthetic only

Environment: pheonix-drone, Linux 6.8, repo `.venv` (Python 3.12.3, OpenCV-contrib 4.14.0, numpy 2.5.4, python-chess 1.11.2), worktree based on a312001. No camera was opened and nothing talked to the arm: everything ran against `interfaces/chess_vision/synth.py` renders.

- **Decision (markers):** DICT_4X4_50 **IDs 10 (a1), 11 (h1), 12 (h8), 13 (a8)**, 40 mm, one diagonally outside each corner. IDs 0-9 stay with `vision/targets`. Printable sheet: `interfaces/chess_vision/markers/board_markers_A4.pdf`. The marker IDs fix the orientation; each marker's printed rotation is solved for.
- **Decision (lean-aware sampling):** an oblique camera sees tall pieces lean away from it, so each square's sample disc is shifted away from the camera by lean × 0.6 square (capped at ¼ square). Lean comes from the marker PnP. Fusion weights each camera per square by 1/(1+(lean/0.6)²) × marker quality × visibility, and fuses with a weighted mean of logits.
- **Problem / Fix:** the first features (raw colour difference) left the logistic badly scaled. A white pawn on e2 came out at P = 0.43 with a difference 20× the empty level. Log features fixed it. The colour feature "median of changed pixels" picked the shaded sides of white pieces on light squares, so it read b1/d1 as black. The 90th percentile of changed pixels, plus the fraction brighter than the empty square, fixed it.
- **Problem / Fix:** from the side camera, the a8 rook hides the a8 marker in the start position. Calibration now reuses the empty board's homography for the start frames when the visible markers agree with it.
- **Observation (results, `pytest -s interfaces/tests/test_chess_vision.py`):** markers placed 2-3 mm off and rotated, three cameras (overhead 72 cm, sides about 47-50° elevation), 10 random positions with lighting gain 0.65-1.25, gradients and tint:
  - fused: 100% occupancy, 100% colour
  - overhead alone: 100% / 100%
  - side alone: 89.5% / 97.4%; left alone: 89.5% / 100%
  - side + left fused: 94.1% / 99.0%
  - grid error: 1.3 mm mean from the nominal marker geometry, **0.17 mm** after the empty-board 7×7 corner refinement
  - e2e4, exd5, O-O and en passant change exactly the right squares
- **Insight:** an overhead view is what makes it reliable. Side views alone lose the squares behind tall pieces, as CHESS.md predicted.
- **Decision (arm scan seam, from the coordinator):** the arm *does* have a camera (the wide-angle USB CAMERA 4K between the gripper fingers, served as `dashboard`). CHESS.md said otherwise; that is now fixed. `PosedFrame` (a frame plus a camera pose from FK × hand-eye) is accepted by the sensor. The planned `ArmScanBoardSensor` and an optional Claude vision second opinion are described in the chess_vision README and are not built.
- **Tests:** `pytest armctl/tests interfaces/tests openplay/tests` gives **320 passed**. `/dev/video0` and `/dev/video1` exist (checked with `ls`, not opened; the console holds the camera).
- **Next:** print the markers and tape them to the real board. Calibrate with an overhead phone (`calibrate --source top=phone:top ...`), then run `watch` while playing a few moves.
