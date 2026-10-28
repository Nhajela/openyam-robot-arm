# interfaces: ways to make the arm move

Every interface is just another **armctl program**. It talks to the armctl daemon over the same WebSocket as the phone tilt and joystick pages, so the lease, the soft-limit envelope, the deadman, operator approval and E-STOP apply to all of them. Nothing here talks to DimOS or the CAN bus directly. Experiment log: [`experiments/2026-10-28-interfaces/LOG.md`](../experiments/2026-10-28-interfaces/LOG.md).

There are two kinds:

| Kind | What it sends | Examples |
|---|---|---|
| **Live** | a joint *target* many times a second; `servo.py` turns it into `stream qd` with a 200 ms deadman | pose mirror, hand tracking, phone tilt and joystick |
| **Timeline** | a *playable* (keyframes): validated, previewed, the operator presses Play | recorded moves, timeline editor, music choreography, English to motion (LLM), pick and place, chess moves |

You can turn a live session into a timeline: record it, then edit and replay it.

| Interface | Where | Status |
|---|---|---|
| Pose mirror (camera → arm copies your arm) | `mirror/` | new: works in the mock sim with a synthetic person and on still photos; not yet tried with a live camera or the real arm |
| Record → timeline editor with "the arm can't do this" bounds | `timeline/` | new: works in the mock sim (record, edit, check, fit, send) |
| Grasp: top-down pick and place | `grasp/grasp.py` | new: planned with IK, plays in the mock sim; **table frame uncalibrated**, gripper mapping unmeasured |
| Chess | `grasp/chess_arm.py` | new: legal moves, captures, castling, en passant; plays in the mock sim |
| Music → dance | `armctl/music.py`, `armctl/choreo.py` | from 16 Oct, played on the real arm |
| English → motion | `armctl/llm_author.py`, [openyam-playground](https://github.com/Nhajela/openyam-playground) | from 16 Oct, played on the real arm |
| Phone tilt and joystick | `dashboard/static/tilt.html`, `joystick.html` | mock + DimOS sim (`DimosBackend` built 28 Oct, not on hardware yet) |
| **Open play**: QR → queue → turns, phone controls, laptop show + operator rail | [`openplay/`](../openplay/README.md) | mock + DimOS sim |
| Pose recorder → playable | `timeline/poses.py` | mock (used by open play's Poses tab) |
| Grab between named spots | `grasp/spots.py` | mock (spots are placeholders) |

## Setup

Use the repo `.venv` (Python 3.12). The interfaces need a few extra packages:

```sh
uv pip install --python .venv/bin/python -r interfaces/requirements.txt
.venv/bin/python -c "import cv2; cv2.aruco; import mediapipe, chess"   # must not fail
sudo apt install libegl1 libgles2      # MediaPipe needs these; desktop Ubuntu usually has them
```

`mediapipe` pulls in `opencv-contrib-python`. If `cv2.aruco` breaks after installing it, see `vision/README.md` (use a separate venv for vision). The MediaPipe pose model (5.8 MB) downloads to `~/.cache/openyam/` on first use.

## Try it without hardware

```sh
# 1. a mock arm (kinematic sim) that grants the lease at once
.venv/bin/python -m armctl.daemon --backend mock --auto-approve --vel-cap 0.6 --acc-cap 2.0
# 2a. the arm copies a synthetic waving person, and the run is recorded
.venv/bin/python -m interfaces.mirror.run --source synthetic --duration 15 --record /tmp/rec.json
# 2b. recording -> playable, checked against the caps
.venv/bin/python -m interfaces.timeline.record /tmp/rec.json --name my-move --caps dance
# 3. timeline editor: http://127.0.0.1:8097
.venv/bin/python -m interfaces.timeline.server
# 4. chess: reach map for the board, then a capture played on the mock arm
.venv/bin/python -m interfaces.grasp.chess_arm check
.venv/bin/python -m interfaces.grasp.chess_arm move e2e4 e7e5 g1f3 b8c6 f3e5 c6e5 --play --auto-confirm
```

Watch the mock arm in the console (`dashboard/start-all.sh`, http://127.0.0.1:8090, SIM view). `--auto-confirm` only works on the mock backend: on the real arm a person presses Play.

## Pose mirror (`mirror/`)

`frame → MediaPipe pose (33 landmarks, metric 3D) → retarget → make_safe → One Euro filter → servo → stream qd`

- **`--mode joints`** (default, "puppet"): your arm's angles drive the joints. Where your upper arm points horizontally sets j1 (base yaw), how high it is raised sets j2, your elbow opening sets j3, and with `--wrist` your wrist bend sets j4. This copies elbows and posture, not hand position.
- **`--mode hand`**: your wrist position relative to your shoulder is scaled into a box in front of the arm and solved with IK, so the tip goes where your hand goes.
- **Mirror convention:** the arm faces you like a reflection. Move your hand to the left of the camera image and the tip goes left in the image too.
- **Safety:**
  - Every target is clipped to the soft box. If a target would bring the arm near the table, the planner raises the upper arm until it's clear, or drops the target.
  - `table_guard` also predicts 0.3 s ahead and holds a joint that would make the arm dip between two safe targets. This was a real problem in the sim: 7 of 163 samples went too low before it was added.
  - If nobody is seen, the arm isn't visible, or targets go stale (300 ms), the program stops sending and the daemon's deadman stops the arm.
- **`--clutch`:** the arm follows only while your *other* hand is raised above your shoulder. Lower it and the arm stops. Use this for the first real-arm tries.
- **Sources:** `synthetic`, `/dev/video0`, `phone:<name>`, `dashboard` (the console's USB camera), any JPEG URL, a video file or a photo (`vision/sources.py` specs).

## Timeline (`timeline/`)

- **`coach.py`** explains in words what a timeline asks of the arm, per segment. For example: *"0 → 0.74 s is too fast for j2, j3, j4: needs at least 1.50 s (has 0.74); in 0.74 s j3 can move at most 0.09 rad."* For each keyframe it also gives the band each joint can reach from the previous one in the time allowed. It checks table clearance with forward kinematics, and has two fixes:
  - `fit(p, "stretch")` slows only the offending segments down.
  - `fit(p, "shrink")` keeps the timing (the beat) and shrinks the offending moves.
  - Motion model: cubic ease peaks at 1.5·d/T velocity and 6·d/T² acceleration.
- **`record.py`** turns a recording (measured joints over time, from any interface) into keyframes. It keeps the pauses, then refines until the cubic playback is within 0.04 rad of the recording, then fits the caps.
- **`server.py`** + `static/timeline.html` is the editor, at http://127.0.0.1:8097. It has one lane per joint plus the gripper. You drag keyframes, double-click to add one and right-click to delete one. Green bars show what each joint can reach, red marks what the arm can't do. The buttons are Stretch/Shrink to fit, Record (observes the daemon while another interface drives the arm), Send to arm (the operator approves and presses Play) and Save.

## Grasp and chess (`grasp/`)

- **`grasp.py`** plans a top-down pick and place as a playable, in this order: ready, above the object, down, close, lift, above the target, down, open, lift, ready.
  - Each pose is solved with `armctl.kin.ik` with the fingers pointing straight down, and j6 turned so the fingers close across the object.
  - Segment times come from the caps, so the playable is feasible by construction.
  - Positions are in the **table frame** (tag 0 from `vision/detect_tags.py`). `workcell.json` maps them to the arm's base frame; `fit_table_frame` computes that mapping from 3 or more points touched with the tip.
- **`chess_arm.py`:**
  - Board geometry in the table frame. With white's rank 1 on the far side, the arm plays black.
  - `check` prints an 8×8 reach map. Moves are checked with python-chess, and the arm's reply comes from Stockfish if it's installed, else from a greedy one-ply picker.
  - Captures go to a row of slots beside the board, and castling moves both pieces.

**Top-down reach** (fingers straight down, from `armctl.kin`): x from 0.10 to 0.52 m up to 10 cm above the table, 0.10 to 0.46 m at 15 cm, 0.14 to 0.40 m at 20 cm, none at 30 cm (j4 hits its limit). A 28 cm board (3.5 cm squares) centred 0.30 m in front of the base fits with margin.

**Unknowns that need the real arm:**
- the table frame (calibrate)
- where `gripper_tip` sits relative to the finger pads (`tip_to_pad`)
- the playable grip 0..1 → finger opening mapping (assumed linear up to 9.4 cm)
- whether the gripper stalls measurably on an object (`holding()`)
- how much the arm sags at full reach without gravity compensation (about 0.03 rad on j2/j3 on 16 Oct: that's about 1.5 cm at the tip, so expect to add a z offset)

## `armctl/kin.py`

FK and IK in plain numpy: no pinocchio, no URDF file. The joint table is copied from `yam.urdf`. FK matches pinocchio (home tip (0.211, 0, 0.174), ready tip (0.205, 0, 0.432)). IK is damped least squares, clamped to the soft box, with restarts. It never returns a target outside the box and says why when it fails.

## Tests

```sh
.venv/bin/python -m pytest -q interfaces/tests armctl/tests
```
