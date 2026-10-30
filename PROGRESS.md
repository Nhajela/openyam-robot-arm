# Progress log

What got done, when, how it was checked, and what's next. Newest session on top. Use one entry per working session, with commit hashes. For the physical setup and detailed measurements, see the lab notebook (`experiments/*/LOG.md`). To start a new session, read [HANDOVER.md](HANDOVER.md) first.

**Legend:**
- ✅ done and tested (where it was tested is written next to it)
- 🧪 built, but only tested in simulation
- ⏳ not started or blocked
- ❗ needs a decision from Naman

---

## 2026-10-30 (13:30–14:35 IST): arcade phone page (serenity; no arm), branch `arcade-phone`

- 🧪 **Open play, arcade version** ([experiments/2026-10-30-arcade-phone/LOG.md](experiments/2026-10-30-arcade-phone/LOG.md), [openplay/README.md § Arcade](openplay/README.md#arcade-the-same-thing-as-a-claw-machine-game)): a second phone page at `/arcade` that plays like a claw machine instead of a set of levers. Drag on an angled map to steer, hold DROP / let go to grab and come back, stars, gold dive stars, combos, score, emotes, Ask it. Same protocol and safety model; STOP above every overlay. `--phone-ui arcade` puts it on the QR (default stays classic).
- 🧪 Server: `/arcade` + `/classic` routes, `--phone-ui`, `welcome.limits` (the daemon's soft joint limits), `ui` in the hello trace. New e2e test `test_arcade_page`.
- Checked on the mock daemon with headless Chromium at phone size: steering moved the tip 14 cm in the dragged direction, DROP lowered it 12 cm, letting go shut the claw and ran Ready, stars scored. `openplay/tests`: only the 7 chess failures that clean `main` also has on serenity.

### Next
1. ⏳ Laptop: `--phone-ui arcade` on the mock with a real phone, then DimOS sim, then the real arm with someone at the E-STOP (LOG "Next").

---

## 2026-10-29 (midday, 11:40–12:40 IST): reuse report, pick up anything, explainer QR (serenity; no arm)

- ✅ **Reuse report** ([experiments/2026-10-29-reuse/REPORT.md](experiments/2026-10-29-reuse/REPORT.md)). DimOS already ships three things: YOLO-E open-vocabulary detection, a CSRT + PID follower (for drones) and an agentic pick-and-place (xArm + depth only). Nobody has published RGB-only YAM picking. Replace our hand-built finders; keep armctl, calibration and the arm-side limits.
- 🧪 **Pick up anything you name** ([experiments/2026-10-29-pick-anything/LOG.md](experiments/2026-10-29-pick-anything/LOG.md)): a detector seam (YOLO-E via DimOS, Claude fallback), `resolve_prompt`, `DetectorFinder` and the `pick_anything` CLI. Synthetic: names the blue box among three objects and picks it. Real YOLO-E not run yet: it runs on the laptop's GPU.
- 🧪 `interfaces.follow.run --prompt X`: follow a named object.
- 🧪 **Open play:** a second, quieter QR on the display ("How does it work?") → the public explainer at `nhajela.github.io/openyam-playground/learn/` (`--learn-url`).
- **Fix:** GrabCut is now seeded (`cv2.setRNGSeed(0)`), so tap segmentation is reproducible. A tap-to-pick test had been failing depending on test order.

- 🧪 **Phone "Pick it up" uses the detector:** lists what it sees, detector outlines for taps, names without Claude, re-detection while picking (`--pick-detector`).
- ❗ **Grasp floor fixed and made table-relative:** the table is 3.1 cm below the base, and a base-frame floor had made every object under ~5 cm (all chess pieces) "too low". The fingertip may now go 3.1 cm lower, to 1 cm above the calibrated table. Foam first.
- 🧪 **`place_spots`** check/teach for the Grab spots and the chess board (the old spots were out of reach in today's table frame).
- Laptop GPU: Lenovo Legion, RTX 3060 6 GB.

### Next
1. ⏳ Laptop: `pick_anything --bench` on the GPU, detect-only on the live camera, then `--execute` on a foam block (HANDOVER §0c).
2. ⏳ Teach spots A/B/C and the chess board (`place_spots`), then Grab on the real arm.

---

## 2026-10-29 (late morning, 10:55–11:40 IST): follow, tracking + visual servoing (serenity; no arm)

**Where:** serenity, repo `.venv` (Python 3.12.3, OpenCV 5.0.0, no DimOS), on top of `e5a64ec`. Notebook: [experiments/2026-10-29-follow/LOG.md](experiments/2026-10-29-follow/LOG.md). Code: [`interfaces/follow/`](interfaces/follow/README.md).

- 🧪 **Tracker** (`350e2ce`): a fast patch tracker (~7 ms/frame), re-anchored by `CupFinder` in a thread, and a Kalman filter with capture-time stamps. Estimate error 0.5–1.7 mm median in every simulated case.
- 🧪 **Controller** (`abfd12a`): the camera stays 0.15 m over the object, looking at it. Joint-velocity output, translation first, then the soft box lookahead and the table guard.
- 🧪 **Virtual-clock sim** (`9737fa7`): circle 4 cm/s gives 3.5 mm median centring. A still cup ~1 mm. A removed cup stops the arm. A teleported cup is followed. An identical cup 8 cm away is ignored. A cup at 8 cm/s is too fast for the joint caps (lags 3.5 cm), but the tracker keeps it.
- 🧪 **Program** (`2e248a4`): `interfaces.follow.run` through the daemon. Mock daemon + synthetic camera end to end: match error 0.5 mm median. `--dry-run` for the first real step.
- ✅ Tests (`7fd256e`): `pytest interfaces/tests/test_follow.py` gives 15 passed (serenity). README (`77e7561`).
- **Full suite on serenity:** 432 passed, 26 failed, 14 errors **before** this work, all environmental or pre-existing: no `dimos` in the repo `.venv` (20, gravity + gains), OpenCV 5.0 dropped `cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC` and `cv2.undistortPointsIter` (7, wristcam calib), and chess/spots/engine tests read the real 29 Oct `workcell.json` (calibrated table, `max_width` 0.11) (13). Not touched here.

### Next
1. ⏳ Real arm: follow `--dry-run` (track only), measure the camera latency, then follow a still cup at `--v-max 0.02` (HANDOVER §0b).
2. ⏳ Servoed descent into a grasp; a tap seed (`SeededFinder`); a second camera in the filter.
3. ❗ The pre-existing test failures above: make the chess/spots/engine tests use a fixture workcell instead of the real file? (Naman)

---

## 2026-10-28 (late afternoon/evening): real-arm day: phone, harness, layout, wrist camera, gravity cause

**Where:** lab laptop, real arm. Notebook: [experiments/2026-10-28-laptop-systems-check/LOG.md](experiments/2026-10-28-laptop-systems-check/LOG.md); research: [experiments/2026-10-28-prior-art/](experiments/2026-10-28-prior-art/REPORT.md), [j4 gravity](experiments/2026-10-28-j4-gravity-scale/LOG.md).

- ✅ **Phone on the real arm:** open play with moves (dance moves slowed to fit the caps), Joints, Say it (Claude), Home/Ready, game-style mode menu, thumb-reach dock, "too low to steer" bar.
- ✅ **j4:** holding error tracks gravity (0.015–0.109 rad); kp 22 halves it. **Cause found:** DimOS's gravity URDF gives the gripper 1.19 kg vs i2rt's 0.60–0.69 kg → ~1 N·m excess on j4. `--gravity-scale j4=…` built (not yet run on the arm).
- ❗ **Incident: arm went limp** (j1 sent to +1.62 past DimOS's +1.571 adapter limit). **Fixed with a harness:** limits = i2rt ∩ DimOS ∩ site; a model-file check at connect; a last-line gate; an edge-of-reach watchdog for Tilt/Sticks; fault → E-STOP + banner. Per-site limits in `armctl/site_limits.json`.
- ✅ **Layout:** the base was rotated (folded gripper points down the table); glass on the arm's right; temporary site limit j1 soft −0.6..+1.2.
- 🧪 **Wrist camera:** a bracket camera ~60° off the fingers (measured). Lens calibrated (fisheye, fx ≈ 537, 44 views, RMS 0.79). Hand-eye **not yet accepted**: the printed board is not the 29.97 mm it claims (the fit says ~22.9 mm); the new gates refuse bad results.
- 🧪 Merged from agents: chess on a real board (clock + vision v1), flowing sequences, wristcam pipeline, hand-eye gates/solvers.
- ❗ **End of day:** j4 stopped answering on CAN after running hot (all LEDs red). Power off overnight.

### Next (tomorrow)
1. Power on → probe: j4 must answer.
2. Gravity: `--gravity-scale j4=0.83` (sim plumbing, then real, short holds, watch j4 temperature).
3. **First pick (Route A, no camera):** measure the cup's position/size and the gripper gap with a ruler → plan (top-down or rim pinch) → sim → real at 0.2 rad/s.
4. Ruler: board width (7 squares) + base board thickness → re-solve hand-eye (`WRISTCAM_SQUARE_MM`, `--table-z`) → camera pick.

---

## 2026-10-28 (afternoon): laptop systems check + first real-arm steps (lab laptop, arm)

**Where:** lab laptop `pheonix-drone`, Ubuntu 24.04, DimOS 0.0.14, `main` @ `2bb9b6d` + the tolerance change. Notebook: [experiments/2026-10-28-laptop-systems-check/LOG.md](experiments/2026-10-28-laptop-systems-check/LOG.md).

- ✅ Pulled fork `main` onto the laptop; installed the interfaces deps (mediapipe, chess, anthropic).
- ✅ CAN + read-only probe: all 7 motors at about 0, no faults. 291 unit tests passed. DimOS-sim e2e 27/27 on the laptop.
- ✅ **Real arm, HANDOVER §5 steps 1–3:**
  - REAL coordinator + daemon on 224.0.0.123:7449
  - `ready` and `home` verified
  - gripper **0 = closed, 1 = open** (confirmed by eye)
  - `disable_all` after each run; LEDs red
- ✅ `REACH_TOL_BY_BACKEND["dimos"]` wrist tolerance raised from 0.09 to 0.12, because ready's j4 was off by 0.094.
- ❗ **j4 (wrist pitch) holding error:** +0.06 folded and +0.094 → +0.116 at ready, growing while held; j4 runs the warmest. Being investigated (gains, gravity) before step 4.
- Found: `grip` reads 1.0 before any gripper command (raw was partly open), so don't trust it until a command has been sent. A sim coordinator segfaulted on Ctrl-C (no hardware involved). `tmux -t 0` targeted the wrong session, so use pane IDs.

### Next
1. ⏳ Explain and fix j4 (gain or gravity feedforward, or mechanical), then HANDOVER §5 step 4 (phone joystick) and step 5 (moves at 0.3 rad/s).

---

## 2026-10-28: interfaces, open play, DimosBackend (serenity server; no arm)

**Branch:** `interfaces-mirror-timeline-grasp`, 22 commits on top of `d4a5c10` (PR #2). **Tests:** `pytest armctl/tests interfaces/tests openplay/tests` gives 253 passed, 2 skipped. **Where it ran:**
- armctl mock backend
- DimOS 0.0.14 `--simulation`
- real phones' browsers simulated with Playwright

**Never on the real arm.** Notebook: [experiments/2026-10-28-interfaces/LOG.md](experiments/2026-10-28-interfaces/LOG.md).

### Morning: interfaces (10:13)
- 🧪 `4db9e1e`: a numpy FK/IK (`armctl/kin.py`, matches pinocchio) plus three new ways to move the arm:
  - **pose mirror**: a camera watches a person and the arm copies their arm
  - **timeline coach + recorder + editor**: says in words what's too fast, with stretch/shrink fixes
  - **pick-and-place planner + chess rules**

  Found along the way: a pose-following path could dip toward the table between two safe targets. A table guard was added and checked.

### Open play, built by five agents in parallel against one contract (10:16–10:46)
- ✅ `ad7dc4b`, `2df2e02`: [openplay/PROTOCOL.md](openplay/PROTOCOL.md), the phone ⇄ server ⇄ laptop contract and the server rules.
- 🧪 `e233a5e`: **phone page**. Lobby and queue, turn timer, STOP always visible. Tabs: drive (tilt, sticks, joints, gripper), moves, Say it, poses, grab.
- 🧪 `0bec759`, `08690fb`: **laptop display**. The show (3D arm, QR, now playing, queue, captions, Claude card with a ghost preview) and an always-on operator rail: E-STOP, approve, auto-play, pause, skip, extend, kick, health, joints.
- 🧪 `bcce7ac`, `5d3056a`, `f00bc5c`: **server**.
  - queue and turn rules
  - one armctl program holding the lease and deciding whose commands reach the arm
  - Say it via Claude, with a library fallback
  - an e2e test with 2 phones and a display
- 🧪 `d7619ba`: **13 new moves**, 17 on the phone. All validated and checked against the table with FK. A new `bow` replaces `graceful-bow`, which never lowered the tip.
- 🧪 `3c6698e`: **pose recorder** (save poses, then play them as one smooth sequence within the speed caps).
- 🧪 `587a6a3`: **grab between named spots** (the spots are placeholders).
- 🧪 `bf63389`: the mock arm's Cartesian twist now uses the real Jacobian, so phone tilt moves the sim correctly.
- ✅ `3faa221`: **chess with a real set**, design only ([interfaces/grasp/CHESS.md](interfaces/grasp/CHESS.md)): overhead camera, legal-move matching, tap-to-move fallback.

### The path to the real arm (11:00–11:07)
- 🧪 `98ebd08`, `1fbfbd1`: **`DimosBackend`**: the armctl daemon now drives a DimOS coordinator.
  - Results against DimOS `--simulation`: 27/27 checks pass, final error 0.0000, first motion ~30 ms, deadman stop 0.3–0.4 s, E-STOP 8 ms.
  - The zenoh bus defaults now follow CLAUDE.md (sim 7450; real 7449 only when exported).
- ✅ `b25fbc6`: daemon fixes needed for hardware:
  - per-joint arrival tolerance for the real arm's sag
  - E-STOP reset clears the backend's latch
  - `relax` gets a clean refusal instead of crashing
- 🧪 `84ddb31`: the display said "REAL ARM" on the DimOS simulator. Sim vs real now comes from the daemon, and anything unknown is shown as real.
- 🧪 Full stack on the DimOS sim: coordinator → daemon (DimOS) → open play → phone. Auto-play stayed off, the operator approved, and DimOS played the move as a 227-point trajectory.

### Docs (11:03–11:09)
- ✅ `9fb380d`, `45e5baf`, `7fdc799`:
  - [HANDOVER.md](HANDOVER.md): pull, index, status, rehearsal, a step-by-step first hardware session
  - the lab log and the indexes
  - this file

### Incidents
- On serenity, the phone listener was bound to the public IP for a few seconds (behind the room key). It was restarted on loopback, and the README now says to use `--phone-host 127.0.0.1` on servers.

### Next
1. ⏳ **Laptop + arm:** rehearse on DimOS sim, then the first hardware session in [HANDOVER.md §5](HANDOVER.md). The order: coordinator `--real`, ready/home, gripper direction, joystick, moves at 0.3 rad/s, open play with auto-play off, mirror with `--clutch`, calibration, first grasp.
2. ✅ Merged to `main` and pushed to the fork `Nhajela/openyam-robot-arm` (Edge-City is pull-only for our account). ❗ Still open: an Anthropic key and spending cap for Say it, and the event network (LAN self-signed vs our domain vs tunnel).
3. ⏳ Chess mode in open play (phone board + display panel), then board vision v1.
4. ⏳ A spline ease (flowing sequences), gravity feedforward or measured sag, and torque state from CAN.
