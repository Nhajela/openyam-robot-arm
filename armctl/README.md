# armctl: one instruction layer for the OpenYAM arm

Every program (phone tilt, phone joystick, LLM playables, console, scripts) talks to one daemon. The daemon holds a single control lease, runs a deadman on streamed setpoints, and checks every motion against the safety envelope. It logs every instruction and every refusal. Spec: [`experiments/2026-10-16-linux-real-arm/ARMCTL-SPEC.md`](../experiments/2026-10-16-linux-real-arm/ARMCTL-SPEC.md), section "Programs".

**Status:** `MockBackend` (a kinematic sim, no hardware; Cartesian twists use the real Jacobian since 2026-10-28). `DimosBackend` (2026-10-28) drives `armctl/dimos_coordinator.py` and passes `armctl/tests/e2e_dimos_sim.py` against DimOS `--simulation`. **It has not run on the real arm yet**; the plan is in `HANDOVER.md`.

```bash
# DimOS sim + armctl on the DimOS backend (the sim bus, CLAUDE.md)
ZENOH_SCOUT_ADDR=224.0.0.124:7450 .venv/bin/python -m armctl.dimos_coordinator --simulation
ZENOH_SCOUT_ADDR=224.0.0.124:7450 .venv/bin/python -m armctl.daemon --backend dimos
```

- **Arrival check per backend** (`REACH_TOL_BY_BACKEND`): 0.02 rad on the mock. On DimOS it's wider (j2/j3 0.05, wrist 0.09), because the real arm sags without gravity feedforward.
- **E-STOP reset:** resetting E-STOP calls `backend.clear_estop()`.
- **`relax`:** a backend that can't drop torque refuses `relax` with a reason.

```
programs  tilt.html · joystick.html (phone, via phonecam's HTTPS relay) · LLM playables · scripts
             │ WebSocket JSON  ws://127.0.0.1:8091/armctl
armctl.daemon   lease · operator approval · deadman · envelope gate · log
             │ Backend interface
MockBackend (100 Hz sim)   |   DimosBackend (DimOS ControlCoordinator, sim-tested)
```

## Run

```bash
.venv/bin/python -m armctl.daemon --backend mock                 # ws://127.0.0.1:8091/armctl
.venv/bin/python -m armctl.daemon --backend mock --auto-approve  # local dev: acquire is granted at once
```

Options: `--host 0.0.0.0` (expose on the LAN; operator endpoints stay loopback-only), `--port`, `--margin 0.1`, `--vel-cap 0.3`, `--acc-cap 1.0`, `--log` (default `experiments/2026-10-16-linux-real-arm/logs/armctl-daemon.log`). It normally runs in tmux window `claude:armctl`.

Phones: `dashboard/phonecam.py` serves `/tilt` and `/joystick` on its HTTPS port (8443) with the same token, and relays WebSocket `/armctl` to the daemon. Phones can't reach 127.0.0.1, and an https page can't open `ws://`. Use the URL in `dashboard/certs/phone-url.txt` with the path `/` replaced by `/tilt` or `/joystick`, or follow the links at the bottom of the phone-camera page. phonecam must be restarted after an update to pick up the routes.

Console: the **Programs** panel at http://127.0.0.1:8090 shows connected programs, the lease holder, pending requests (Approve / Deny / Give control), a pending playable (Play / Cancel), refusals, the armctl log, and a big **E-STOP**. `dashboard/server.py` proxies `/armctl/*` to the daemon. Anyone who can reach the console can operate the arm, so keep the console on 127.0.0.1.

## Modules

- `envelope.py` (stable API, imported by others). It holds joints j1..j6, the URDF limits, and the soft box (URDF shrunk by a margin of 0.1 rad, configurable per joint). It also holds speed caps (0.3 rad/s) and acceleration caps (1.0 rad/s²), plus the named poses `home` (zeros) and `ready` `[0, 0.5, 0.8, 0, 0, 0]` (**to be verified on hardware**). Its functions:
  - `check_point(q)`: a list of `Violation`s, each with a precise `.message`, e.g. `j4 target 1.620 > soft limit 1.471 (URDF 1.571 - margin 0.10, past URDF limit 1.571)`. Home passes as a special target.
  - `check_path(samples, dt, allow_home=, from_outside=)` checks every sample, plus velocity and acceleration. `allow_home` lets j2/j3 go down to 0 when the path ends at home. `from_outside` lets a path that starts outside the box (at home, or past a limit) move only monotonically back towards it.
  - `clamp_step(q, qd, dt)` is the streaming guard. It zeroes velocity components that would leave the box and returns the reasons. `lookahead()` covers the braking distance.
  - `cap_velocity`, `describe()` (JSON for clients and LLM prompts), `as_vector`, `joint_index`.
- `backend.py`. `Backend` interface: `state`, `execute(traj)`, `move_joints`, `set_joint_velocity(qd, ttl)`, `set_twist(v, ttl)`, `gripper(width)`, `stop`, `estop`, `relax`, `enable`. It also has `plan_move` (min-jerk at the caps) and `plan_home` (slow final 0.1 rad onto the j2/j3 stops at 0.05 rad/s).
  - `MockBackend` simulates at 100 Hz. It applies the caps and deadman expiry, runs the per-tick `clamp_step` guard, and treats the URDF limits as physical stops. Cartesian twists use a fixed, made-up `J_APPROX` map, not real kinematics.
- `daemon.py` is the protocol, lease, gate and logging. `Daemon` is transport-independent (tested directly); `build_app` wires FastAPI and WebSockets to it.
- `gains.py` (pure) parses and caps per-joint holding-gain overrides for `dimos_coordinator.py`: `--kp j4=22 --kd j4=2` (repeatable, or `--kp j4=22,j5=15`) replaces single entries of DimOS's `WholeBodyConfig` (kp 80,80,80,10,10,10 / kd 5,5,5,1.5,1.5,1.5); joints not named keep the default, the gripper is never touched. Caps (refused otherwise): kp <= 120 on j1-j3, <= 40 on j4-j6; kd <= 8 on j1-j3, <= 5 on j4-j6. The effective gains are in the coordinator's startup line and in `armctl_info()["gains"]`. Motivation: j4 sags 0.06-0.12 rad at kp 10 (`experiments/2026-10-28-laptop-systems-check/LOG.md`, 12:55). **Try in `--simulation` first** (the mock adapter ignores gains, so this only checks the plumbing). On the real arm watch j4's temperature and listen for buzz or chatter; stop and drop the override at the first sign of either.
- Visual choreography: `visual_author.py` (draft + review passes where Claude sees its own motion), `choreo_render.py` (offscreen render sheets: audience view first, keyframes + in-betweens, timeline with limit bands and speed headroom; `python -m armctl.choreo_render playables/wave-hello.json --out DIR`), `choreo_scene.py` (camera router + wrist-camera snapshot), `choreo_guidelines.md` (the choreography rules in the prompt), `audience.json` (where the audience stands, per site). CLI: `python -m armctl.llm_author "a happy wave" --visual --passes 3 --model claude-fable-5-1 --out playables/happy-wave.json --sheets-dir DIR [--camera auto|always|never]`. Notes: [experiments/2026-10-29-visual-choreo](../experiments/2026-10-29-visual-choreo/LOG.md).
- `playable.py`, `llm_author.py` are owned by the playables agent. The daemon imports `armctl.playable` lazily and uses `validate(playable)` and `sample(playable, rate)`.

## Protocol (as implemented)

Client → daemon (every message may carry an `id`, which is echoed back):

| op | notes |
|---|---|
| `hello {program, name}` | required first. Reply: `{"op":"hello", client, protocol:"armctl/0", lease, estop, envelope, poses, ttl_ms}` |
| `acquire` | `ack {pending:true}` until the operator approves (or granted at once with `--auto-approve`). Then everyone gets `{"op":"lease", holder, client, name}` |
| `release` | stops the arm, frees the lease |
| `do {cmd, ...}` | `home`, `ready`, `goto {name}`, `move_joints {q, speed}`, `nudge {joint, delta ≤ 0.2}`, `gripper {width 0..1 \| open \| close}`, `stop`, `relax`, `save_pose {name}`. Arguments go next to `cmd` (or in `args`, or as `cmd: {"goto": {"name": "ready"}}`). Reply: `ack {duration, target}`, then `done` after execute-then-verify (within 0.02 rad), or `refused {reason}` |
| `stream {v:{x,y,z,roll,pitch,yaw} \| qd:[6], grip?, ttl_ms}` | velocity setpoint (m/s, rad/s). ttl defaults to 200 ms and is capped at 500. No ack per message; guard refusals come as `refused {stream:true, joint, reason}`, at most once per second per joint |
| `playable.validate {playable \| name}` | reply `{"op":"playable.result", name, ok, violations:[str]}`. A `name` loads `playables/<name>.json` |
| `playable.play {name}` | needs the lease and a successful validate from this program. `ack {pending:true}`; the arm moves only after the operator presses Play in the console (`POST /playable/confirm`), then `ack {playing}` and `done` |
| `state`, `ping` | one state message / `pong` |

Daemon → clients: `state` at 20 Hz (`q, qd, grip, grip_target, torque, mode, moving, lease, estop, busy, streaming, pending, pending_play`), `lease`, `estop {latched}`, `ack`, `done`, `refused`, `playable.result`.

Operator HTTP (loopback only): `GET /status`, `POST /lease {client | program | revoke:true}`, `POST /lease/deny {client}`, `POST /estop`, `POST /estop/reset`, `POST /enable`, `POST /playable/confirm {name}`, `POST /playable/cancel`.

### Rules the daemon enforces
- One lease holder. A switch (approve, give control, revoke, release, holder disconnect) always stops the arm first and waits until it is still (up to 3 s).
- `do stop` is accepted from any connected program. Every other motion needs the lease.
- Out-of-envelope targets and paths are refused with the reason, never clamped. Streaming is the only exception: it slides along the boundary.
- A discrete move refuses while another is running (`busy`, send `stop` first). Streaming also refuses during a move.
- E-STOP halts at once (torque stays on), revokes the lease, drops pending requests and playables, and latches. Every motion is refused until the operator resets it.
- After `relax` (torque off), motion is refused until the operator presses Enable in the console.
- The deadman works on two layers: the backend stops following a setpoint when its ttl expires and decelerates within the accel cap, and the daemon logs it. A disconnect counts as a release.

### Where this deviates from or extends the spec
- The spec doesn't say what `grip` in `stream` means. Here it is an absolute target from 0 (closed) to 1 (open), and it is ignored when absent. The phone pages send `do gripper` instead.
- Additions: `done` (completion after verification), `ack {pending}` for acquire/play, `playable.result`, the `hello` reply, `save_pose`, operator `/enable`, and a latched E-STOP with `/estop/reset`.
- `ttl_ms` above 500 is capped to 500 rather than refused.

## Tests

```bash
.venv/bin/python -m pytest armctl/tests                    # envelope, lease, deadman, refusals, playables
.venv/bin/python armctl/tests/e2e_ws.py                    # real WebSockets vs the running mock daemon (refuses non-mock)
.venv/bin/python armctl/tests/browser_phone_pages.py --token T --base https://127.0.0.1:8453 --daemon http://127.0.0.1:8191
```

The browser check needs a mock daemon with `--auto-approve` on 8191, and a phonecam phone app on 8453 relaying to it with a throwaway token T. Use `phonecam.build_phone_app(store, T, "ws://127.0.0.1:8191/armctl")`. It needs selenium and geckodriver.
