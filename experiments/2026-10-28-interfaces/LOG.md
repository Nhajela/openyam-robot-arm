# Experiment: more ways to make the arm move

**Experimenters:** Naman (with Claude Code)
**Started:** 2026-10-28
**Status:** active. Everything here ran in the mock simulation only; nothing has touched the real arm yet.

## Goal

Treat every way of moving the arm as an *interface* on top of one safety layer (armctl), and prototype the next ones in the sim, ready to try once the arm is connected again:

1. **Pose mirror:** a camera watches a person and the arm copies their arm (elbows and posture) in real time.
2. **Record + timeline:** record what the arm does, edit it as a choreography timeline, and have the computer say what the arm can't do ("j3 can't move this far this fast").
3. **Grasping:** a top-down pick and place we can trust, then chess on top of it.

Music sync (`armctl/choreo.py`) and English → motion (`armctl/llm_author.py`) from 16 Oct are the same kind of thing, a timeline interface. Code: [`interfaces/`](../../interfaces/README.md).

## Environment

- Machine: serenity (Ubuntu 24.04.5 LTS, Linux 6.8.0-147, x86_64, no GPU, no camera, no CAN adapter). This is a server, not the lab laptop.
- Repo `Edge-City/openyam-robot-arm` at `d4a5c10` (PR #2 merged), cloned to `~/src/openyam/openyam-robot-arm`.
- `.venv`: Python 3.12.3 via `uv venv`. **No DimOS** (not needed: armctl's mock backend is a kinematic sim). Packages: numpy 2.5.4, mediapipe 1.2.0, opencv-contrib-python-headless 5.0.0.93, chess 1.11.2, fastapi, uvicorn, websockets, pytest.
- Mock daemon: `.venv/bin/python -m armctl.daemon --backend mock --auto-approve --port 8191 --vel-cap 0.6 --acc-cap 2.0` (log: [logs/mock-daemon.log](logs/mock-daemon.log)).

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
2. **`DimosBackend`** (step 1 of the 16 Oct next steps): without it, live interfaces only drive the mock. Then a supervised real run from `ready` with `--clutch`, the daemon at 0.3 rad/s, the person outside the arm's reach.
3. **Calibrate the table frame:**
   - Touch the tip to tag 0's centre and two marks 10 cm along its axes.
   - Get their FK, then `fit_table_frame` and set `calibrated: true` in `interfaces/grasp/workcell.json`.
4. **Measure the gripper:** grip 0..1 → width, where the pads sit relative to `gripper_tip`, whether it stalls on an object (`holding`), and the sag at full reach.
5. **First real grasp:** a foam block from a taped spot, then one chess piece, then a capture.
6. Camera-based board reading (which square changed) so the human doesn't have to type their move.
