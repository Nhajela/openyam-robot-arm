# interfaces/follow: track an object and keep the wrist camera over it

Instead of look, then plan, then move (`interfaces/wristcam/engine.py`), a feedback loop: the wrist camera tracks the object every frame and the arm is steered toward it continuously, so it **follows an object that moves**. It is the base for correcting a grasp on the way in (next step, see below). Notebook: [experiments/2026-10-29-follow/LOG.md](../../experiments/2026-10-29-follow/LOG.md).

**Status (29 Oct):** built on serenity (no arm, no camera). Tested with the synthetic wrist camera on a virtual clock and end to end through the armctl daemon on the mock backend. **Not yet run on the real arm.**

| File | What |
|---|---|
| `track.py` | `Tracker`: two-rate tracking. **Fast** (`PatchTracker`, every frame, ~7 ms at 640×360): the frame is resampled onto the plane at the object's top height around the prediction, and the top-face template is matched by NCC with a sub-pixel peak. **Slow** (`AnchorWorker`, in a thread, ~1 Hz): the accurate finder (`CupFinder`, `TagFinder`, …) near the estimate re-anchors it, refreshes the template and catches drift. `KalmanCV` (constant velocity, table xy) fuses both with **capture-time** stamps. `QHistory` gives the q at each frame's capture time. |
| `control.py` | `FollowController`: position-based visual servoing. The camera goes `follow_h` above the object, optical axis down, and looks at the object (look-at, at most `max_tilt`). 5-D task (camera point + axis direction; roll is free, and j6 is held in the null space). DLS on the geometric Jacobian plus velocity feedforward. **Translation gets the joint-speed budget first**, the axis turn gets what's left. Then: speed caps, acceleration caps, `envelope.clamp_step` with lookahead, `servo.table_guard`. |
| `look.py` | `look_pose`: camera straight down over a table point (same construction as `engine.camera_over`). |
| `sim.py` | Virtual-clock simulation: `VelocityArm` (the mock backend's streaming rules: caps, acceleration, deadman, soft box), the ray-cast camera at a set fps and latency, and cup paths `static line circle stopgo jump vanish`, plus an optional identical distractor cup. Metrics + `plot.png`. |
| `run.py` | The program: ArmClient → daemon (mock or real), `--source dashboard\|/dev/videoN\|synthetic`, look pose, seed, follow, log. |

## Try it (no hardware)

```sh
# one simulated run on a virtual clock (deterministic): metrics, log.jsonl, plot.png
.venv/bin/python -m interfaces.follow.sim --path circle --speed 0.04 --duration 10 --out /tmp/follow-sim

# the whole program through the daemon: mock backend + synthetic camera rendered from the mock's q
.venv/bin/python -m armctl.daemon --backend mock --auto-approve
.venv/bin/python -m interfaces.follow.run --source synthetic --sim-path circle --expect 0.0,-0.12 --duration 20 --v-max 0.06

.venv/bin/python -m pytest -q interfaces/tests/test_follow.py        # 14 tests, ~1 min
```

## How it decides to move (and not to)

- Every 1/30 s the controller asks the filter for the object's position at *now + `--lead`* and streams one joint-velocity setpoint with a 200 ms ttl.
- **Nothing is sent** (so the daemon's deadman brings the arm to rest within its acceleration cap) when any of these holds:
  - the track is lost: no match for 0.3 s with at least 2 frames that didn't match, or no match at all for 1 s;
  - the newest frame is older than 0.35 s;
  - the joint state is older than 0.3 s.
- After 5 s lost (`--give-up`), the program stops.
- While lost, the slow finder searches around the last position. Its window widens with the time lost, up to 12 cm, but never covers the whole frame, so an identical object elsewhere is not taken for the target. The first look searches near `--expect`.
- The object position is clamped to a reach annulus (0.16–0.44 m from the base axis). Outside it, the camera stops at the edge and only turns to look. Joints stop at the soft box (lookahead), and the table guard holds any joint that would take the arm down.
- Speeds, real-arm defaults: camera ≤ 0.03 m/s, joints ≤ 50% of the daemon's caps (0.15 rad/s at the default 0.3). The sim uses 0.06 m/s and 60%.

## Simulated results (29 Oct, virtual clock, 640×360 fisheye, 15 fps, 80 ms camera latency)

Centring error = where the optical axis hits the cup's top plane vs the cup's true centre, measured after the first 3 s. Full table and discussion: [the notebook](../../experiments/2026-10-29-follow/LOG.md).

| Case | Centring median / p95 | Estimate error median |
|---|---|---|
| still cup | ~1 / 1.5 mm | 0.6 mm |
| circle 4 cm/s | 3.7 / 4.4 mm | 0.7 mm |
| cup removed | arm stops (< 0.003 rad after) | lost, held |
| cup teleported 4.5 cm | followed, no loss | — |
| identical cup 8 cm away | no switch | 0.95 mm p95 |
| end to end (mock daemon, synthetic camera rendered from the daemon's q, 2.5 cm/s circle) | — | 0.5 mm (match vs truth) |

The renderer is the model, so these are best case. On the real arm expect a few mm more, from the provisional hand-eye (4.2 mm spread), camera latency we haven't measured, and the real cup's look. A cup faster than the joint caps allow (8 cm/s with j1 capped at 0.18 rad/s) lags by 4–6 cm: the tracker keeps it, the arm can't.

## Real-arm procedure (person at the arm, every step agreed)

Follow `CLAUDE.md`: probe first, real bus `ZENOH_SCOUT_ADDR=224.0.0.123:7449`, start line from `HANDOVER.md` §0, and after every stop support the arm, run `tools/disable_all.py` and check the LEDs. **j4 heats up while it holds a look pose** (58–60 °C on 29 Oct), so keep runs short (default `--duration 45`) and fold home between runs.

1. **Track only, nothing moves.** Park the arm at a camera-down look pose over the cup (for example from `look_and_grasp --dry-run`'s look pose), then:
   `.venv/bin/python -m interfaces.follow.run --source dashboard --finder cup --dry-run --duration 30 --frames-every 5`
   Slide the cup by hand. The log's `frame` events show the matches. Check that the track stays on the cup, and measure the camera latency: tap the cup and compare the frame in which it moves against the log time. Set `--latency` to that.
2. **Follow a still cup:** `--expect <x,y>` (it moves to the look pose after a y/N, then asks y/N again before following), `--v-max 0.02`. The camera should settle over the cup and stay still.
3. **Slide the cup slowly** (1–2 cm/s). Then raise `--v-max` to 0.03–0.04 and `--speed-scale` up to 0.6.
4. Tune: `--lead` (0 = no prediction; the sim's best is 0 with no actuation delay, the real arm has some), `--follow-h`, `--max-tilt`.

## Next

- **Servoed descent for grasping:** while following, lower `follow_h` step by step down to the engine's approach height, keep correcting, then hand over to the engine's CLOSE/LIFT/VERIFY (`GraspEngine` with the tracker's estimate). Also look at `target.SeededFinder` as the anchor for tapped objects.
- **Extra cameras:** a fixed overhead or phone camera is a second measurement for the same `KalmanCV` (another table-frame xy with its own sigma and latency).
- Image-based servoing (pixel error straight to camera velocity) when the hand-eye is uncertain. It is less sensitive to calibration but needs the interaction matrix and the depth.
