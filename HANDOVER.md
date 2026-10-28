# Handover: OpenYAM interfaces, open play, DimosBackend (2026-10-28)

The starting point for a local session on the lab laptop with the real arm. It says what was built on the server (serenity: no arm, no camera, no CAN), how it was tested, and the step-by-step plan for the first hardware session. **Keep it current:** update the status table and "Next" whenever something changes.

> **Status at a glance:** everything runs in armctl's **mock** sim, and the full stack (DimOS coordinator → armctl daemon on `DimosBackend` → open play → phone) also runs in DimOS **`--simulation`**. **On 28 Oct the laptop ran §5 steps 1–3 on the real arm (ready, home, gripper); nothing beyond that has moved it.** The code lives on Naman's fork, **`Nhajela/openyam-robot-arm` `main`**. Our GitHub account can't push to `Edge-City/openyam-robot-arm` (pull-only), so work goes to the fork; a PR to Edge-City is optional.

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
   ZENOH_SCOUT_ADDR=224.0.0.123:7449 .venv/bin/python -m armctl.dimos_coordinator --real --can-port can0
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

- Chess mode in open play: built in the mock (real board + phone chess clock + display panel; `--chess-sensor fake|camera`; refused on the real arm until `workcell.json` is calibrated). Next: the board camera (`interfaces/chess_vision/camera.py`), then calibration (`CHESS.md` §5).
- A spline ease, so pose sequences and recordings flow instead of stopping at each keyframe.
- A per-joint `REACH_TOL` and a gravity feedforward from measured sag.
- A torque-state readout from CAN replies inside `DimosBackend`.
