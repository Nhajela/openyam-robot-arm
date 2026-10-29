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
| [2026-10-28-wristcam](2026-10-28-wristcam/LOG.md) | Naman (+ Claude) | Wrist camera: calibrate (lens, hand-eye, table) from one taped sheet; find a cup and grasp it in a closed loop | active: synthetic tests green; real-arm calibration next |

## Ideas backlog

Interfaces and interactions worth trying. Add yours, and say who to talk to.

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
