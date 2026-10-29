# Experiments

A shared lab notebook for everyone working on the OpenYAM arm. The aim is to share what we learn and build great interfaces and interactions with the arm. Write down everything, including failures, so the next person doesn't repeat them.

## How to add an experiment

1. Make a folder `experiments/YYYY-MM-DD-<slug>/` with a `LOG.md` (copy the shape of an existing one).
2. At the top, list **Experimenters** (who ran it), **Goal** and **Status** (active / paused / done / abandoned).
3. Keep a dated running log: what you tried, the exact commands, what happened, what you learned. Put any scripts, recordings or photos next to the log.
4. Add a row to the table below.
5. Once something is a stable procedure, move it into the main README or `tools/`, and link to it from the log.

Safety rules carry across all experiments: start folded on the stops, probe before enabling, keep a hand ready to support the arm, run `tools/disable_all.py` after every stop, check that every LED is steady red.

## Index

| Experiment | Experimenters | Goal | Status |
|---|---|---|---|
| [2026-10-16-linux-real-arm](2026-10-16-linux-real-arm/SUMMARY.md) ([log](2026-10-16-linux-real-arm/LOG.md)) | Naman (+ Claude) | Drive the real arm from the Ubuntu laptop and go beyond keyboard jogging | done: see SUMMARY.md for next steps |
| [2026-10-28-interfaces](2026-10-28-interfaces/LOG.md) | Naman (+ Claude) | More ways to move the arm: pose mirror, record + timeline with limits, grasp and chess ([`interfaces/`](../interfaces/README.md)), open play ([`openplay/`](../openplay/README.md)), DimosBackend | active: mock + DimOS sim; hardware next ([HANDOVER.md](../HANDOVER.md)) |
| [2026-10-28-j4-gravity-scale](2026-10-28-j4-gravity-scale/LOG.md) | Naman (+ Claude) | Why j4 holds off target: DimOS's gravity URDF gripper is 1.19 kg vs i2rt's 0.69 (+1 N·m on j4); fits; `--gravity-scale j4=…` per-joint feedforward scale in armctl.dimos_coordinator | code + offline analysis done; real-arm test next |
| [2026-10-28-laptop-systems-check](2026-10-28-laptop-systems-check/LOG.md) | Naman (+ Claude) | Pull fork `main` onto the lab laptop; check CAN, motors (read-only), deps, tests, DimOS-sim e2e before the first hardware session | active: all green, real-arm steps next |
| [2026-10-28-prior-art](2026-10-28-prior-art/REPORT.md) | Naman (+ Claude research agent) | Find repos, models, datasets and people on the same infra (i2rt YAM, Damiao, DimOS) to reuse: j4 gravity, wrist-camera calibration, cup grasp, chess, phone teleop, LLM control | done (desk research) |
| [2026-10-28-j4-gravity-scale](2026-10-28-j4-gravity-scale/LOG.md) | Naman (+ Claude agent) | Why j4 holds off target: DimOS's gravity URDF gives the gripper 1.19 kg vs i2rt's 0.60–0.69 kg (~1 N·m extra on j4); `--gravity-scale j4=…` override | built + unit-tested; real-arm test next |
| [2026-10-29-visual-choreo](2026-10-29-visual-choreo/LOG.md) | Naman (+ Claude agent) | Better choreography: Claude Fable drafts a playable, sees render sheets of it from the audience's position, and revises it in 2–3 passes; camera snapshot only for prompts about the real scene | built + mocked tests; one real API run; real-arm playback next |
| [2026-10-29-openplay-event-log](2026-10-29-openplay-event-log/LOG.md) | Naman (+ Claude agent) | Why Tilt/Sticks "disconnect" on the real arm: time-synced trace phone → server → armctl, per-hold stream stats, `/events` viewer, `python -m openplay.eventlog summarize`. Found the daemon's "needs the arm unfolded" refusal was never relayed to the phone | built + mock-tested; real-arm session next |
| [2026-10-29-kin-calib](2026-10-29-kin-calib/LOG.md) | Naman (+ Claude agent) | Why hand-eye fails: joint fit of joint zero offsets + camera mount + board poses over he1/he2. j4 −41 and j5 −24 mrad offsets cut reprojection 8.9→3.6 px, fixed the board scale (×1.08→×1.00); correction in `armctl/kin_corrections.json`, vision/grasp only | offline; physical zero check + wider-j1 view set next |
| [2026-10-29-pick-anything](2026-10-29-pick-anything/LOG.md) | Naman (goal) + Claude (cloud build) | Pick up anything you name: open-vocabulary masks (YOLO-E via DimOS, Claude fallback) into the tap path's geometry and closed loop; `pick_anything` CLI; follow by name | built; synthetic tests green; YOLO-E on the laptop GPU next |
| [2026-10-29-reuse](2026-10-29-reuse/REPORT.md) | Naman (question) + Claude (research agents) | Who already solved detect / follow / pick: DimOS ships YOLO-E open-vocabulary detection, a CSRT + PID drone follower and an agentic pick-and-place (xArm + depth only); what to replace in our code and what to keep | done (desk research); port next |
| [2026-10-29-follow](2026-10-29-follow/LOG.md) | Naman (idea) + Claude (cloud build) | Continuous tracking + visual-servo following: the wrist camera tracks the object at frame rate (fast patch tracker re-anchored by the slow finders, Kalman with capture-time stamps), and the arm keeps the camera over it by streaming joint velocities through the deadman ([`interfaces/follow/`](../interfaces/follow/README.md)) | built; synthetic sim + mock-daemon e2e green; real-arm dry run next |
| [2026-10-28-wristcam](2026-10-28-wristcam/LOG.md) | Naman (+ Claude) | Wrist camera: calibrate (lens, hand-eye, table) from one taped sheet; find a cup and grasp it in a closed loop | active: synthetic tests green; real-arm calibration next |

## Ideas backlog

Interfaces and interactions worth trying. Add yours, and say who to talk to.

- **Continuous object tracking + visual-servo following (Naman, 29 Oct):** instead of look, then plan, then move, a feedback loop that keeps tracking the target in the wrist camera (and any extra cameras) at frame rate and steers the arm toward it continuously, so it follows an object that moves and corrects grasps on the way in. **Following built (29 Oct, sim + mock daemon):** [`interfaces/follow/`](../interfaces/follow/README.md), notebook [2026-10-29-follow](2026-10-29-follow/LOG.md). Still open: the servoed descent into a grasp, a tap seed (`target.SeededFinder` as the anchor), extra cameras as a second measurement in the same filter.
- **Re-choreograph the library with the visual author** (`armctl.visual_author`): wave, nod, happy-dance read as 1–3 cm hand moves from the audience today. Run each through 2–3 visual passes, compare sheets side by side, keep the winners. (Naman, 29 Oct)
- **"Dance" caps for authored moves** after a hardware test at 0.6 rad/s: at the default 0.3 rad/s a 15 s piece only has room for a few big moves. Human decision, per move (`--caps dance`).

- **Arm scans the board itself (chess, after grasping works):** a small wrist camera; the arm roams to 3–4 top-down poses over the board, takes a picture at each, and a Claude vision model reads the position (constrained to legal moves) alongside the per-square occupancy diff. No separate overhead camera, and the same camera helps with grasp precision. Costs: about 10–15 s per scan, the arm moves over the human's side of the board, plus wrist-camera cabling and hand-eye calibration. See `interfaces/grasp/CHESS.md` § Camera placement. (Naman, 28 Oct)
- **armctl wrapper on top of DimOS**: home(), ready(), named poses, slow joint moves, limit-aware jogging. Never leaves a soft-limit box inside the URDF limits. (Naman)
- Go-home / named poses from a script (`openyam-planner-coordinator` + `Arm` SDK).
- **Programs over one protocol** (see ARMCTL-SPEC § Programs): phone tilt, phone joystick, English → playable motion sequences (dance, wave, nod), keyboard, console. Switchable, one control lease. (Naman)
- Natural-language control: an agent that turns "pick up the cup" into arm actions. **First version built (sim):** open play "Pick it up" (tap or say the object, the wrist camera homes in): `experiments/2026-10-29-first-pick/TAP-TO-PICK.md`. Next: a place target ("put it in the bowl"), two-view height.
- Gamepad or phone teleop instead of the keyboard.
- Record and replay demonstrations (teach by moving the arm by hand with torque off, then play it back). Recording → playable and the timeline editor exist now (`interfaces/timeline/`); teach-by-hand needs a torque-off state stream from `DimosBackend`.
- **Spline ease** for playables (smooth through keyframes, stops only at the ends): recorded moves become stop-and-go and about 2× longer with `cubic`. Needs spec 1.1 in the lab and the playground together. (Naman)
- Camera reads the chess board (which squares changed) so the human doesn't type their move. Design: `interfaces/grasp/CHESS.md`.
- Chess mode in open play: phone board (tap legal moves), display panel with board, clock and eval bar.
- Spline ease so pose sequences and recordings flow instead of stopping at every keyframe.
- Mirror with both arms: the second arm's hand opens and closes the gripper.
- Camera + pick and place: planner and chess in `interfaces/grasp/`; next is calibrating the table frame and measuring the gripper.
- A live web dashboard: joint states, temperatures, faults, LED state.
