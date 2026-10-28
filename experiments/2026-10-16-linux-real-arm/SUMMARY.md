# Summary: real OpenYAM arm from Linux (2026-10-16)

**Experimenters:** Naman (at the arm) with Claude Code and its subagents
**Time:** 14:05 to 17:00 IST (machine clock), one afternoon at Edge City India
**Status:** done. Follow-ups listed at the end. The full running log is in [LOG.md](LOG.md).

## What we set out to do

Drive the physical OpenYAM arm from an Ubuntu laptop, interface with it properly, and make it do things beyond keyboard jogging, with interfaces other people can use and build on.

## What works now

| Capability | How | Hardware-tested |
|---|---|---|
| Talk to all 7 motors from Linux | SocketCAN `can0` (kernel gs_usb) + `tools/` (now cross-platform via `tools/openyam_bus.py`) | yes |
| Live state, 3D model, cameras, guidance | Lab console `dashboard/` at http://127.0.0.1:8090 (REAL and SIM views, coach panel, phones, playables, music) | yes (read-only on the bus) |
| Safe joint-space moves | `armctl_real.py` (jog / ready / home) through a soft-limit gate | yes |
| Motion sequences ("playables") | `playables/*.json` → validator → `play_real.py` → DimOS JointTrajectory | yes: nod ×2, wave ×2, happy-dance, beat-dance ×4 (with music), rom-arm, rom-wrist, ABBA "I Have a Dream" (concert wave + hype), bhangra |
| One-command session | `session_play.sh <name> [--caps dance] [--audio wav]`: coordinator up → play → home → down → torque off (trap) | yes |
| English → motion | `armctl/llm_author.py` (Claude CLI), validated; FK-aware version in the public playground | sim/preview only |
| Dance to any song | `armctl/music.py` + `armctl/choreo.py` + console `/music`; plan JSON as an editable "intelligence layer" (`step_beats` for half-time songs) | yes: ABBA (Claude-written plan + auto hype), synthesized bhangra |
| Phone programs | Phone camera streaming, **tilt** (with lift-to-move up/down) and **joystick** pages → armctl daemon (lease, deadman, E-STOP) | mock arm only |
| Vision groundwork | `vision/`: printable ChArUco/ArUco targets, intrinsics calibration, tag pose | synthetic + live frames (no tags yet) |
| Public playground + spec | https://github.com/Nhajela/openyam-playground: **LLM-facing spec v1.0.0** (`site/spec/v1.md`, point your own AI at it), reference simulator + Python validator (agree with the lab on 89 cases), static viewer/validator/QR on GitHub Pages; the Next.js app for hosted generation | live at https://nhajela.github.io/openyam-playground/ (spec: `/spec/v1.md`, viewer: `/viewer.html`) |
| Playable format versioning | `"format": "openyam-playable", "version": "1.x"`; lab accepts any 1.x, refuses other majors (`armctl.playable.compat`) | n/a |

## Key findings

**Platform**
- On Linux, `can_motor_control` only has `SocketCanBus`; `GsUsbBus` exists only on macOS. DimOS's `--can-port` is an interface name (`can0`) on Linux and a USB serial on macOS. Bring `can0` up at 1 Mbit/s (`can0-up.sh` + a udev rule that does it on every replug).
- The torque-on signal you can trust is the status nibble in each motor reply (`b0 >> 4`: 0 = disabled, 1 = enabled). `is_enabled` is not trustworthy, and the coordinator logs `deactivate returned False` on most stops. Always run `disable_all.py` and check the wire.

**Arm and control**
- The arm can arrive with joint 1 about 180° off home (−3.10 rad, outside the −150° to +180° range). Folded doesn't mean home: probe joint 1 at every session start.
- Keyboard teleop is Cartesian (Pink IK). From the folded home, j2 and j3 sit *on* their lower limits, so the IK fails ("QP solver did not find a solution") and pushes the wrist into its limit. **Start any Cartesian control from a "ready" pose.**
- DimOS does **not** check position limits on raw trajectories, so our armctl gate is the only one. Never send single-point targets (they jump at 1 rad/s).
- ~~The PD control has no gravity feedforward.~~ **Correction 2026-10-28:** DimOS does feed gravity forward (`gravity_comp=True`, pinocchio on `yam_gripper_gravity.urdf`), but the model misses part of the load; see [2026-10-28-laptop-systems-check](../2026-10-28-laptop-systems-check/LOG.md) 12:55. Expect a steady sag of about 0.03 rad on j2/j3 and about 0.06 rad on the wrist (kp 10), so arrival tolerances must be per joint.
- Repeatability is excellent: repeated runs end within 0.001-0.002 rad of each other.
- Gripper motor −2.9 rad = fully open (DimOS opens it during startup calibration); about 0 = closed.
- Motor temperatures stayed at 26-34 °C over about 10 powered sessions.

**Motion design**
- Cubic (smoothstep) easing peaks at 1.5·d/T velocity and 6·d/T² acceleration. At the "dance" caps (0.6 rad/s, 2 rad/s²), a 2-beat move at 100 BPM can be at most about 0.45 rad.
- A language model given only joint names and limits gets geometry wrong: its "bow" lifted the tip. Forward kinematics shows +j2 alone lowers the tip and moves it forward, +j3 lifts it and moves it back. Feeding the model tip positions and an FK check fixes this; the playground does both.

**Infrastructure**
- DimOS 0.0.14 processes find each other through zenoh multicast. Real runs use a private bus (`224.0.0.123:7449`) and sims use another (`224.0.0.124:7450`).

## Incident

From 15:48:54 to 15:50:58 a subagent accidentally ran a real-arm coordinator while testing that a launcher *refuses* `--real` with an unset bus (it passed the real `can0`, and a double-import bug let it through). The arm was folded at home, the IK refused every tick, and nothing moved. Motors were found disabled afterwards, and `disable_all.py` was sent anyway. Fixes: the launcher bug, `CLAUDE.md` rules (never pass `can0` to anything under test; one wide "is DimOS running" check), a wider guard in `session_play.sh`. Lesson: test refusal paths with inputs that are harmless if the refusal fails.

## Mistakes we corrected along the way

- A pushed log contained the phone-camera token. The token was rotated and the log stopped printing it; logs that might hold secrets are now gitignored.
- `session_play.sh`'s readiness check matched the *previous* run's log line (a wave request did nothing). It now reads only this launch's lines.
- Timestamps in the log had been estimated and drifted 30-50 min. They were corrected from the run logs; the rule is to take times from the machine clock.

## How to run things (from the repo root)

```sh
dashboard/start-all.sh                                   # console :8090, phonecam :8443/:8444, armctl mock daemon :8091
.venv/bin/python -I tools/probe_motors.py can0           # read-only: every joint ~0, no faults
experiments/2026-10-16-linux-real-arm/session_play.sh nod                     # with the human's go
experiments/2026-10-16-linux-real-arm/session_play.sh beat-dance --caps dance --audio experiments/2026-10-16-linux-real-arm/media/audio/beat-dance-100bpm.wav
.venv/bin/python -m armctl.llm_author "wave hello" --save                     # English → playable
.venv/bin/python -m armctl.choreo music/inbox/<song>.mp3 --style groove --save <name>
```

## Added after the first wrap-up (16:00-17:00)

- **Public spec v1.0.0 for bring-your-own-AI**, plus a static viewer, a format/version scheme the lab enforces, and a reference simulator. The playground repo has everything (`playground/` here mirrors its README).
- **ABBA "I Have a Dream"** (first minute): the analysis said 104 BPM, and the human heard the real ~52 BPM pulse. New plan field `step_beats` maps each movement onto real beats and bars; new motif **lighter-wave** (concert phone-in-the-air sway). Played on hardware, plus an auto "hype" version.
- **Bhangra**: `make_bhangra_beat.py` synthesizes a 112 BPM swung dhol *chaal* with tumbi, chimta and "hoi" accents. A bass pulse on every beat keeps the beat tracker off the swung off-beats. Choreographed and played on hardware.
- **Safety:** `session_play.sh` now holds and folds home before torque-off on any failure or Ctrl-C (it used to cut torque wherever the arm was). The validator no longer crashes on a single-keyframe playable.

## Next steps

1. **Real-arm joystick/tilt.** Finish `DimosBackend` (the launcher `armctl/dimos_coordinator.py` exists; the backend is designed but not built) in sim on the sim bus, then a supervised real run from `ready`.
2. **Deploy the hosted playground** on Coolify (the static spec + viewer are already live on GitHub Pages) (Cloudflare A record + Coolify app, `LLM_PROVIDER=openrouter`), plus a QR "station" in the console that fetches `/api/p/<id>`, re-validates, previews and queues for Play.
3. **Print the calibration targets** (`vision/targets/`, 100%); calibrate both phones; do hand-eye calibration with the gripper tag; then tag-based pick and place.
4. **Faster, livelier motion:** measure the beat-to-motion lag on video and add `--audio-offset`; consider gravity feedforward or a higher wrist kp.
5. **Safety:** a hardware e-stop on the 24 V line; a person-in-workspace pause from the phone cameras; an origin check on the armctl daemon WebSocket.
6. Open questions: the license of the i2rt YAM meshes; what the j6 settle at +0.11 rad after the last wave means (hand, or cable twist?).
