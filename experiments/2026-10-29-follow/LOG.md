# 2026-10-29 Follow: continuous tracking + visual-servo following

**Experimenters:** Naman (idea) + Claude (cloud build on serenity)
**Goal:** replace "look, then plan, then move" with a feedback loop. The wrist camera tracks the target at frame rate, and the arm is steered toward it continuously, so it follows an object that moves (and, later, corrects a grasp on the way in). Backlog entry: [experiments/README.md](../README.md) "Continuous object tracking + visual-servo following".
**Status:** active. Built and tested in simulation on serenity (no arm, no camera). The real-arm test is next.

## Environment (cloud build)
- serenity (Linux 6.8.0, Ubuntu), repo `.venv` Python 3.12.3, OpenCV 5.0.0, numpy 2.5.4. No DimOS in this venv (DimOS 0.0.14 is in `~/src/openyam/dimos-venv`).
- Start commit `e5a64ec` (fork `Nhajela/openyam-robot-arm` `main`).

## 2026-10-29 10:58 IST: starting point and design

**Observation (timings on serenity, `/tmp` scratch script):**

| Step | Time per frame |
|---|---|
| synthetic render 1280×720, supersample 2 | 1176 ms |
| synthetic render 1280×720, ss 1 | 275 ms |
| synthetic render 640×360, ss 1 | 59 ms |
| `CupFinder.find`, whole frame | 1000–1800 ms |
| `CupFinder.find` with a hint (4 cm) | 380–410 ms |

**Insight:** the existing finders are built for accuracy on still frames (1 mm), not speed. At 0.4 s per hinted look they can't steer a moving arm (a cup moving at 5 cm/s goes 2 cm per look, and the arm moves the camera during the look). So the tracker has to be two-rate.

**Decision (design):**
1. **Fast track (every frame, a few ms):** resample a small patch of the frame onto the plane at the object's top height (as `SeededFinder` does: the top face looks the same size from any distance), and NCC-match a template of the top face taken from the last anchor. Sub-pixel peak. The q used for the view is the q *at the frame's capture time* (q history, interpolated, minus the camera latency), because the camera is moving.
2. **Slow anchor (background, ~1 Hz):** the existing finders (`CupFinder`, `TagFinder`, `SeededFinder` from a tap) re-find the object near the estimate. They give the absolute position, refresh the template and catch drift.
3. **Filter:** a constant-velocity Kalman filter in table xy with time-stamped measurements. The control loop asks for the position *now + command latency*, so a moving target is led, not lagged.
4. **Servo (30 Hz, joint-velocity streaming through the daemon's deadman):** position-based. Desired camera: `follow_h` above the object's top, optical axis straight down. The axis aims at the target (look-at), tilted at most `max_tilt` from vertical, so the target is centred quickly by the wrist while the position catches up. The camera twist is moved to the tip, then damped least squares on the geometric Jacobian gives qd, then: Cartesian speed cap, joint caps, acceleration limit, `envelope.clamp_step` with lookahead, and `servo.table_guard`. The target is clamped to a reach annulus around the base; outside it the arm only looks.
5. **No target, no motion:** a lost track, a stale frame or a stale q means nothing is sent, and the daemon's deadman stops the arm (ttl 200 ms). A lost track coasts on the filter for at most 0.3 s, then holds and re-searches with the slow finder.
6. **Simulation on a virtual clock:** a velocity-streaming fake arm (caps, acceleration, deadman, envelope), the ray-cast camera at a set fps and latency, and a cup that moves along a path. Deterministic, so tests can measure the tracking error and lag.

## 11:00–11:25 IST: built `interfaces/follow/` and what the simulation found

Code: [`interfaces/follow/`](../../interfaces/follow/README.md) (`track.py`, `control.py`, `look.py`, `sim.py`, `run.py`), tests `interfaces/tests/test_follow.py`, sweep [`sweep.py`](sweep.py). Simulation: 640×360 fisheye (`synth.TRUE_CAMERA` scaled), 15 fps, 80 ms camera latency, control 30 Hz, slow anchor `CupFinder` taking 0.4 s of virtual time, joint noise 0.5 mrad, red cup 6 × 8 cm, no calibration sheet. The camera is 0.15 m above the cup top.

Bugs the simulation caught, in order (each one is a **Problem** → **Fix**):
1. **Problem:** a track was lost at birth. The anchor arrives 0.4 s (in the program, up to 2.5 s) after its frame, so the new track was already older than `coast_s`. **Fix:** a new track gets a grace period. It is lost only after `coast_s` counted from its first fast frame, and that frame searches a window grown by how far the object may have moved since (0.05 m/s, at most 1 s).
2. **Problem:** the controller asks for the estimate at now + lead. That call aged the track into "coasting" every tick. **Fix:** the track ages on frame capture times only (`update`). Stale frames are the caller's check (`run.py` holds when the newest frame is > 0.35 s old).
3. **Problem:** a 2–3 cm drift on a moving cup while "tracking". An anchor refresh re-cut the template at the *fused* estimate, so its bias was baked into every later match. **Fix:** cut it at the anchor's own position. Estimate error: 25 mm → 0.7 mm.
4. **Problem (line path, 4 cm/s toward the base):** the camera lagged 8 cm with j4 pinned at its cap. Cause: the lag asks for more look-at tilt, j4 saturates, the uniform cap scaling shrinks the translation too, so the lag grows. **Fix:** priority. The translation (with the axis held) gets the joint-speed budget first, and the axis turn gets what still fits. Line: 25.5 → 9 mm median.
5. **Problem (end to end):** after frame gaps the track was "lost" with no missed match at all. The synthetic renderer, the slow finder and the fast match share the CPU, so frames came at 4–8 fps with gaps. **Fix:** lost = no match for `coast_s` **and** ≥ 2 frames looked at that didn't match, or no match at all for 1 s. A gap in the frames is not evidence that the object went (the arm holds anyway).
6. **Problem:** two identical cups. The first look and the re-search after a loss scanned the whole frame and could take the other cup (estimate error p95 183 mm). **Fix:** the seed searches near `--expect` (`--seed-radius` 6 cm). A lost track is re-searched around its last position, widening 5 cm/s up to 12 cm. Anchor looks while tracking stay within ~3 cm.
7. **Problem (8 cm/s):** 3 track resets with an accurate estimate. `KalmanCV.at(t)` for a past t returned the current position, so a correct late anchor of a fast cup "disagreed" by more than the 3 cm gate, and the track was reset to a stale position. **Fix:** `at()` extrapolates backwards too.
8. **Problem:** the program aborted at exit ("terminate called without an active exception", core dump), with the anchor thread still inside OpenCV. **Fix:** `AnchorWorker.join()` and the frame threads joined before exit, and the tasks gathered.
9. **Problem:** the daemon's own guard refused j1 at the lab's site limit (−0.6) once: harmless, it slid along the edge. **Fix:** the controller's `clamp_step` lookahead gets +0.1 s, because our q is up to a state period older than the daemon's.

- **Insight:** the tracker is not the limit. Its estimate stays within 0.5–1.7 mm (median) in every case. The centring error comes from the arm: joint caps (an 8 cm/s cup needs j1 ≈ 0.27 rad/s, and we cap at 0.18), the reach edge near the base, and the lead.
- **Insight:** velocity feedforward halves the centring error on a moving cup (circle 4 cm/s: 6.4 → 3.5 mm). The lead (aiming ahead) helps only with actuation delay, which the sim doesn't model: here 0 is best (2.4 mm), 0.05 s costs 1 mm. The real arm has some delay (daemon → DimOS → motors), so `run.py` defaults to 0.05 s, to be tuned on the arm.
- **Insight:** mismatched latency (150 ms real vs 80 ms assumed) only costs ~1 mm here. The q is taken at the wrong time, but the arm moves slowly. A fast-moving arm would be more sensitive, so still measure it.
- **Decision:** real-arm defaults are conservative: camera ≤ 0.03 m/s, joints ≤ 50% of the daemon's caps, 45 s runs (j4 heating at look poses), a y/N before the look move and before following, refused on the real arm unless `workcell.json` is calibrated. The `--dry-run` (track only, no lease) is the first real step.

## 11:40 IST: final sweep (all fixes in)

`PYTHONPATH=. .venv/bin/python experiments/2026-10-29-follow/sweep.py` → [logs/sweep.jsonl](logs/sweep.jsonl) (full metrics per case), [logs/sweep.txt](logs/sweep.txt). 10 s per case; metrics after the first 3 s. This ran with the lab's **site limits** (j1 ≥ −0.6) and the **kinematic corrections** on, so it is closer to the lab than the tests are (the tests turn both off, see `conftest.py`). Defaults: lead 0.05 s, v_max 0.06 m/s, joints 60% of 0.3 rad/s, acc_sigma 0.6.

| Case | Centring median / p95 (mm) | Estimate median / p95 (mm) | Lost / resets |
|---|---|---|---|
| still cup | 0.8 / 1.3 | 0.6 / 1.05 | 0 / 0 |
| line 4 cm/s (toward the base) | 9.2 / 21.3 | 0.5 / 3.9 | 0 / 0 |
| circle 4 cm/s | 3.5 / 4.7 | 0.7 / 1.3 | 0 / 0 |
| circle 8 cm/s (faster than the joint caps) | 34.8 / 53.4 | 1.7 / 2.7 | 0 / 0 |
| stop-and-go 5 cm/s | 5.0 / 11.8 | 0.6 / 4.25 | 0 / 0 |
| teleport 4.5 cm | 1.2 / 31.0 (the transient) | 0.6 / 1.55 | 0 / 0 |
| cup removed at 4 s | arm moved 0.0014 rad after, then still | — | 1 / 0 |
| circle 4 cm/s, lead 0 | 2.4 / 3.4 | 0.7 / 1.3 | 0 / 0 |
| circle 4 cm/s, no feedforward | 6.4 / 11.4 | 0.7 / 1.3 | 0 / 0 |
| circle 4 cm/s, latency 150 ms (known) | 4.2 / 7.2 | 1.1 / 2.0 | 0 / 0 |
| circle 4 cm/s, latency 150 ms, assumed 80 | 3.8 / 7.5 | 1.2 / 2.9 | 0 / 0 |
| circle 4 cm/s, 8 fps | 4.0 / 6.0 | 0.8 / 1.65 | 0 / 0 |
| circle 4 cm/s, v_max 0.03 (real-arm default) | 7.9 / 22.5 | 0.7 / 1.4 | 0 / 0 |
| circle 3 cm/s + identical cup 8 cm away | 2.5 / 3.2 | 0.5 / 1.0 | 0 / 0 |
| stop-and-go 5 cm/s, acc_sigma 0.3 | 4.2 / 12.2 | 0.5 / 4.65 | 0 / 0 |
| circle 4 cm/s, acc_sigma 0.3 | 3.8 / 5.7 | 0.7 / 1.3 | 0 / 0 |

Plot and frames of the circle case: [sim-circle/plot.png](sim-circle/plot.png), [sim-circle/frames/](sim-circle/frames/) (green cross = track, white circle = image centre).

- **Observation:** with the real-arm default `v_max 0.03`, a 4 cm/s cup outruns the camera (7.9 mm median, 22.5 mm p95), but the track holds. Raise `--v-max` once following a still cup and a slow cup look right.
- **Observation:** acc_sigma 0.3 vs 0.6 makes little difference here. Keep 0.6 (it handles hand-pushed starts and stops) unless the real feedforward jitters.
- **Insight:** the worst simulated case is still the line toward the base (9 mm median). The camera-down pose gets awkward near the reach's inner edge and near j1's site limit. Expect the same on the arm when the cup comes close to the base.
- **Environment check:** `pytest interfaces/tests/test_follow.py` → 15 passed (serenity, 1 min). The full suite has 40 failures from before this work (see PROGRESS.md 29 Oct), none in the files touched here.

**Next (real arm):** HANDOVER §0b. Run `--dry-run` first (track only, nothing moves) and measure the camera latency from its log.
