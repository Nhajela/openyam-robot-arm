# Handover: OpenYAM interfaces, open play, DimosBackend (2026-10-28)

The starting point for a local session on the lab laptop with the real arm. It says what was built on the server (serenity: no arm, no camera, no CAN), how it was tested, and the step-by-step plan for the first hardware session. **Keep it current:** update the status table and "Next" whenever something changes.

> **Status at a glance:** everything runs in armctl's **mock** sim, and the full stack (DimOS coordinator → armctl daemon on `DimosBackend` → open play → phone) also runs in DimOS **`--simulation`**. **On 28 Oct the laptop ran §5 steps 1–3 on the real arm (ready, home, gripper); nothing beyond that has moved it.** The code lives on Naman's fork, **`Nhajela/openyam-robot-arm` `main`**. Our GitHub account can't push to `Edge-City/openyam-robot-arm` (pull-only), so work goes to the fork; a PR to Edge-City is optional.

## 0. Where we stopped (29 Oct, 17:00): pick works; chess next
- **Arm:** folded home, motors disabled (j4 ~52 °C). Probe before re-enabling. j4 heats when the arm is **held** under torque (ready or look poses, open play idling): fold home or disable between sessions; the guard stops picks at 57 °C.
- **Picking works on the real arm** (`experiments/2026-10-29-first-pick/LOG.md`): purple ball picked seamlessly (purple4) with `look_and_grasp --finder blob --hsv 145,150,70,165,255,255 --grasp-frac 0.25 --min-tip-z 0.03 --open-margin 0.04 --realign-max 0.012 --no-grip-check` and the guarded auto-answer. Keep objects 25–35 cm from the base (j2 load).
- **Chess** (`experiments/2026-10-29-chess/LOG.md`): YOLO-E (CPU; NVIDIA driver not loaded: install `nvidia-driver-595`) finds pieces only obliquely; `--finder fixed` goes for a known position. Last try: the pinch closed on the piece's round head (height guessed) and slipped. **Next:** a side camera (phone via `dashboard/phonecam.py`) for neck heights and a lift check; then pinch under the head; verify from home with the detector.
- Phone/open play: the room key changes every start (rescan the QR); `.env` holds `CLAUDE_API_KEY`, pass it as `ANTHROPIC_API_KEY`; after an E-STOP: reset on the display, then approve open play in the console (:8090).
- Viewer: `.venv/bin/python -m interfaces.wristcam.viz --latest-under <runs dir> --port 8097` (live camera panel).

## 0a. Where we stopped (29 Oct, 10:47)
- **Arm:** motors disabled (folded home) to let **j4 cool** (rotor 58–60 °C after holding look poses). Probe before re-enabling; keep look/hold time short.
- **Calibration:** wrist-camera hand-eye **accepted as PROVISIONAL** by Naman (35 views of the 25 mm tag sheet; position spread 4.2 mm, board scale ×1.001; reprojection 4.8 px / rotation 1.4° miss the strict gates). Joint zero corrections in `armctl/kin_corrections.json` (vision/grasp only). To clear "provisional": a rigid flat board + redo.
- **Cup:** the dry run on bare wood finds it to ±1 mm (`experiments/2026-10-29-first-pick/cup-dryrun3/look.jpg`), and the pick plan validates (fingertip grasp just below the rim, `--grasp-frac 0.85`; workcell gripper opening 0.11 m from Naman's measurements). **The real closed-loop pick has not been run yet.** The cup has since been moved: re-look first.
  - Real pick command (after a probe + real stack): `.venv/bin/python -m interfaces.wristcam.look_and_grasp --execute --object cup --finder cup --grasp-frac 0.85 --expect <table x,y> --place <same x,y> --settle 1.0` (each motion asks y/N).
- **Known issues:** the tap path (`target.resolve`) refuses taps on the cup ("bare_table"); a white cup on white paper isn't segmented; `look_and_grasp --expect` doesn't narrow the dry-run search (it found the tape roll).
- **Next idea (Naman):** continuous object tracking + visual-servo following. **Built in the cloud on 29 Oct (11:00–11:30, serenity, sim only): see §0b.**

## 0c. Pick up anything you name (built 29 Oct, sim only; detector runs on the laptop GPU)
- **Code:** `interfaces/wristcam/detect.py` (YOLO-E via DimOS, Claude fallback), `target.resolve_prompt` / `DetectorFinder`, CLI `interfaces/wristcam/pick_anything.py`. Notebook: [`experiments/2026-10-29-pick-anything/LOG.md`](experiments/2026-10-29-pick-anything/LOG.md). Why: [`experiments/2026-10-29-reuse/REPORT.md`](experiments/2026-10-29-reuse/REPORT.md) (DimOS already ships detection; reuse it).
- **First steps on the laptop:**
  1. `.venv/bin/python -m interfaces.wristcam.pick_anything --bench 30 --prompt "cup,marker,tape,box"`. Nothing moves. The YOLO-E weights download on first use. Check that the GPU is used (`"device": "cuda"`).
  2. Detect only on the live camera (`--source dashboard --prompt "marker"`): check `look.jpg` (masks, footprint, closing line) against reality.
  3. `--execute` on a foam block or a marker. Every motion asks y/N.
  4. Phone: `openplay.server ... --pick-detector yoloe` (the default). After Look, "Pick it up" lists what it sees; taps and names use the detector.
- **Grasp floor is now table-relative** (`Workcell.floor_base()`): the fingertip may go to 1 cm above the calibrated table, which is 3.1 cm lower than before. Try the first low grasp on foam.
- **Grab spots + chess board:** `.venv/bin/python -m interfaces.grasp.place_spots check`, then `teach A` / `teach B` / `teach C` with the fingertip over taped marks (Joints mode). For the board: `board a1`, then `board h8`. Nothing moves; `workcell.json` is backed up first.

## 0b. Follow: track an object and keep the camera over it (built 29 Oct, sim only)
- **Code:** [`interfaces/follow/`](interfaces/follow/README.md). Notebook: [`experiments/2026-10-29-follow/LOG.md`](experiments/2026-10-29-follow/LOG.md).
- **What it does:** two-rate tracking. A fast patch tracker (~7 ms/frame) is re-anchored by `CupFinder` in a thread (~1 Hz), fused by a Kalman filter with capture-time stamps. The arm streams joint velocities at 30 Hz through the daemon's deadman to keep the camera 0.15 m over the cup, looking down. It sends nothing when the track is lost or frames/state are stale.
- **Tested:** virtual-clock sim (still cup ~1 mm; 4 cm/s circle 3.5 mm median centring, estimate 0.7 mm; a removed cup stops the arm; an identical cup nearby is ignored) and end to end through the mock daemon with the synthetic camera. `pytest interfaces/tests/test_follow.py`: 15 tests. **Never on the real arm.**
- **First real steps** (after the probe + real stack, `interfaces/follow/README.md` "Real-arm procedure"):
  1. `--dry-run` (track only, nothing moves): `.venv/bin/python -m interfaces.follow.run --source dashboard --finder cup --dry-run --duration 30 --frames-every 5`. Slide the cup by hand, check the track, and measure the camera latency (→ `--latency`).
  2. Follow a still cup at `--v-max 0.02`, then slide it slowly.
  - Keep runs short: j4 heats at look poses.

## 0. Start here (next session, after 28 Oct)

1. The arm's power supply was switched OFF overnight because **j4 overheated and stopped answering on CAN** (all LEDs red). Power on, then `.venv/bin/python -I tools/probe_motors.py can0`: all 7 must answer with no faults. If j4 is still silent, inspect its connector/cable before anything else.
2. Real-arm start line (site limits in `armctl/site_limits.json` load automatically):
   `ZENOH_SCOUT_ADDR=224.0.0.123:7449 .venv/bin/python -m armctl.dimos_coordinator --real --can-port can0 --kp j4=22 --kd j4=2`
   **plus `--gravity-scale j4=0.83`** (tested 29 Oct: j4 error ≤0.023 rad, peak 37 °C; `experiments/2026-10-29-first-pick/LOG.md`). Fold home when idle.
3. First pick (no camera): measure the cup's position from the base centre with a ruler, plus its rim diameter and height and the open gripper gap → plan top-down or rim-pinch → sim → real.
4. Camera route: measure the printed board (7 squares wide) and the base board's thickness → `WRISTCAM_SQUARE_MM=…` → `handeye solve --dir interfaces/wristcam/data/he1 --write-workcell --table-z …` (gates must pass).
5. Open play: `openplay.server --daemon ws://127.0.0.1:8091/armctl --auto-play off`. Don't `export` `OPENPLAY_NO_CARTESIAN` in a shared shell.

## 1. Pull it onto the laptop

```sh
cd ~/openyam-robot-arm                       # the laptop checkout with the DimOS .venv (README §2)
git remote add fork https://github.com/Nhajela/openyam-robot-arm.git   # once
git pull fork main
uv pip install --python .venv/bin/python -r interfaces/requirements.txt qrcode anthropic
.venv/bin/python -c "import cv2; cv2.aruco; import mediapipe, chess, dimos"   # if aruco broke: interfaces/README.md "Setup"
.venv/bin/python -m pytest -q armctl/tests interfaces/tests openplay/tests    # expect ~253 passed
```

The serenity checkout (`~/src/openyam/openyam-robot-arm`) has `main` tracking `fork/main`, so a plain `git push` there goes to the fork.

Read, in order:
1. `CLAUDE.md` (safety rules)
2. this file, then [PROGRESS.md](PROGRESS.md) (dated progress log with commit hashes)
3. `openplay/README.md`
4. `interfaces/README.md`
5. `experiments/2026-10-28-interfaces/LOG.md` (the full notebook, with numbers)

## 2. Index: where everything is

| Path | What | Tested on |
|---|---|---|
| `armctl/` | safety layer: envelope, daemon (lease, deadman, gate, E-STOP), playables, LLM author, music | real arm, 16 Oct |
| `armctl/backend.py` `DimosBackend` | daemon → DimOS ControlCoordinator (`armctl/dimos_coordinator.py`): trajectories, twists, joint streams, gripper, E-STOP | DimOS sim (27/27 e2e) |
| `armctl/kin.py` | FK, IK, Jacobians in numpy (no pinocchio) | unit tests, matches pinocchio |
| `openplay/` | **open play**: QR → queue → turns; phone controls; laptop show + operator rail; Claude "Say it" | mock + DimOS sim |
| `openplay/PROTOCOL.md` | phone/display/server contract and server rules | |
| `playables/` + `openplay/moves.json` | moves library (17 on the phone; 13 new on 28 Oct) | new ones: validator + FK only |
| `interfaces/mirror/` | camera pose → arm copies your arm (joints or hand/IK) | mock + photo |
| `interfaces/timeline/` | coach (what's too fast and why), recording → playable, pose sequences, editor `:8097` | mock |
| `interfaces/grasp/` | pick and place, spots, chess rules → moves; `CHESS.md` = real-set design | mock |
| `dashboard/` | lab console `:8090`, phone cameras, tilt/joystick pages | real (console) |
| `vision/` | calibration targets, ArUco tag poses | camera only |
| `experiments/` | lab notebook (index: `experiments/README.md`) | |
| `PROGRESS.md` | progress log: one entry per working session, with commits, test state, next | |

## 3. Status

| Component | State | Before / on the real arm |
|---|---|---|
| `DimosBackend` | DimOS sim 27/27 (laptop + serenity). **Real arm 28 Oct (laptop): §5 steps 1–3 done**: ready/home verified, gripper 0 = closed / 1 = open confirmed by eye | **j4 (wrist pitch) holding error** of +0.04 to +0.12 rad, growing while held; wrist tolerance raised to 0.12 with no headroom, so investigate before step 4. Still open: torque state (TODO: CAN status nibble), held-tick ripple at kp 80. Log: `experiments/2026-10-28-laptop-systems-check/LOG.md` |
| Open play | full flow on mock and DimOS sim; display shows sim vs real from the daemon | phone tilt over the real Wi-Fi + self-signed HTTPS on an iPhone and an Android; Claude key for Say it |
| Moves library | 17 moves validate and clear the table | watch each at reduced speed first (`--vel-cap 0.3`), then at dance caps |
| Pose recorder / timeline | works in sim | record from the real arm |
| Pose mirror | works with a synthetic person and photos | live camera; tune mapping and smoothing; measure lag; first real run with `--clutch` |
| Grasp / spots / chess | plans and plays in sim; spots and table frame are placeholders | calibrate the table; measure the gripper; foam block before pieces |

## 4. Rehearse on the laptop (no motors)

```sh
# A. mock (fastest)
.venv/bin/python -m armctl.daemon --backend mock --auto-approve --vel-cap 0.6 --acc-cap 2.0
.venv/bin/python -m openplay.server        # display: http://127.0.0.1:8451 (F11); phones: the QR
# B. DimOS sim: the same software path the real arm uses (sim bus per CLAUDE.md)
ZENOH_SCOUT_ADDR=224.0.0.124:7450 .venv/bin/python -m armctl.dimos_coordinator --simulation --traj-vel-limit 0.7
ZENOH_SCOUT_ADDR=224.0.0.124:7450 .venv/bin/python -m armctl.daemon --backend dimos --auto-approve --vel-cap 0.6 --acc-cap 2.0
ZENOH_SCOUT_ADDR=224.0.0.124:7450 .venv/bin/python armctl/tests/e2e_dimos_sim.py   # 27 checks
.venv/bin/python -m openplay.server
```

Then test with two real phones on the Wi-Fi:
- accept the certificate warning, then join
- tilt (iPhone asks for motion permission), sticks, moves
- Say it (export `ANTHROPIC_API_KEY` first)
- poses, grab
- on the laptop: approve, skip, kick, E-STOP

## 5. First hardware session: plan (person at the arm, every step agreed)

**Before power:**
- Arm folded on its stops; workspace clear; e-stop in reach.
- A line taped on the table that people stay behind.
- Hands and objects out of the gripper jaws: every coordinator start runs a close/open **calibration sweep** and leaves the gripper open (measured 28 Oct).
- `.venv/bin/python -I tools/probe_motors.py can0`: every joint about 0, no faults (check j1: it can arrive about 180° off).

**Steps:**
1. **Coordinator + daemon at safe caps.** The real bus has to be exported explicitly:
   ```sh
   ZENOH_SCOUT_ADDR=224.0.0.123:7449 .venv/bin/python -m armctl.dimos_coordinator --real --can-port can0 --kp j4=22 --kd j4=2   # stiffer j4: halves its sag (28 Oct j4 test)
   ZENOH_SCOUT_ADDR=224.0.0.123:7449 ARMCTL_DIMOS_EXPECT=real .venv/bin/python -m armctl.daemon --backend dimos
   ```
   The daemon log must say `DimOS REAL` with the three expected tasks; the console shows the adapter.
2. **Console:** `ready`, then `home`. Check the arrival errors in the daemon log against the new tolerances (j2/j3 0.05, wrist 0.09). If they still fail, record the sag numbers in the LOG and adjust `REACH_TOL_BY_BACKEND`.
3. **Gripper:** `do gripper 1` should OPEN. If it closes, the mapping is inverted: fix `DimosBackend.gripper` before anything else.
4. **Phone joystick** (via phonecam or open play) from `ready`, small moves. Then tilt. Watch for the held-tick ripple.
5. **Library moves at 0.3 rad/s** (daemon default caps): nod, wave-hello, bow. Then restart the daemon with `--vel-cap 0.6 --acc-cap 2.0` and try two dance moves.
6. **Open play dry run** with auto-play **off**: approve each move on the laptop.
7. **Mirror** with `--clutch` (`interfaces.mirror.run --source /dev/video0 --clutch`), the person standing outside the arm's reach.
8. **Calibration for grasping:**
   - jog the tip to 3–4 table points, `fit_table_frame`, set `calibrated: true` in `interfaces/grasp/workcell.json`
   - measure the gripper width at grip 0/0.5/1
   - find where the pads sit relative to `gripper_tip`
   - check whether a closed grip on a block reads higher than an empty one (`holding()`)
9. **First grasp:** a foam block from spot A to B (`interfaces.grasp.spots`). Then a chess piece.

**After every stop:** support the arm, Ctrl-C both processes, run `.venv/bin/python -I tools/disable_all.py can0`, and check every LED is steady red. Log everything in a new `experiments/YYYY-MM-DD-real-arm-interfaces/LOG.md` (CLAUDE.md rules).

## 6. Decisions made

- **DimOS limit harness (after the 28 Oct j1 fault, arm went limp):** armctl's hard limits are the intersection of `i2rt/yam.urdf` and the DimOS adapter's `yam_gripper_gravity.urdf` (j1 ≤ 1.571, j6 ≤ 1.833). `DimosBackend` checks the installed model file at connect, gates every trajectory and stream step 0.03 rad inside the adapter limits, stops Tilt/Sticks at the edge of reach, and on a latched DimOS feedback fault E-STOPs and shows ARM FAULTED everywhere (no auto-recovery). DimOS itself can't hold position on a fault yet (it disables the motors); a 'hold' option in the adapter is the proposed upstream change.
- Every interface is an armctl program. In open play, phones never touch the arm: `openplay.server` holds the lease and arbitrates.
- **Auto-play** (players' moves play without operator approval) defaults to on only for the mock. With it off, the operator's Approve on the laptop is the Play press (one approval).
- The laptop display is show + always-on operator rail (`h` hides the rail).
- **The sim/real label comes from the daemon's `adapter`.** Unknown is treated as real.
- **Zenoh buses** (CLAUDE.md): sims 224.0.0.124:7450, real 224.0.0.123:7449, and the real bus only when exported. The coordinator and backend enforce this.
- Pose mirror uses a mirror convention; playables that touch the table on purpose carry `min_tip_z`.
- **Chess with a real set:** game-state tracking plus legal-move matching from an overhead camera, with tap-to-move as the fallback (`interfaces/grasp/CHESS.md`).

## 7. Open questions for Naman

- Open a PR from the fork to `Edge-City/openyam-robot-arm`? It needs someone with write access there to merge it. The code is on the fork's `main`.
- Which Anthropic API key (and spending cap) should open play use at events?
- Event network: LAN with a self-signed cert (warning on each phone), our domain with a real cert pointed at the laptop's LAN IP, or a tunnel (see `openplay/README.md`).
- Turn length for busy nights (3 / 5 / 10 min)?

## 8. Next (after the first hardware session)

- 🧪 **Pick up anything** (§0c): YOLO-E bench on the GPU, then detect-only, then a real pick. After that, chess pieces (a small named object on a known square).
- 🧪 **Follow** (`interfaces/follow/`, §0b): real-arm dry run, then following. Next to build: the servoed descent into a grasp (lower `follow_h` while correcting, then the engine's CLOSE/LIFT/VERIFY), a tap seed via `target.SeededFinder`, and a second camera in the same filter.

- Chess mode in open play: built in the mock (real board + phone chess clock + display panel; `--chess-sensor fake|camera`; refused on the real arm until `workcell.json` is calibrated). Next: the board camera (`interfaces/chess_vision/camera.py`), then calibration (`CHESS.md` §5).
- 🧪 A spline ease, so pose sequences and recordings flow instead of stopping at each keyframe: built (`flow` ease, `interfaces/timeline/flow.py`, `style="flow"`, `record --flow`), tested in pure code only; try it in `--simulation` before the real arm.
- A per-joint `REACH_TOL` and a gravity feedforward from measured sag.
- A torque-state readout from CAN replies inside `DimosBackend`.
