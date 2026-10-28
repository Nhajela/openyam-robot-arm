# OpenYAM robot arm (Edge City India)

Setup notes and helper scripts for the Anvil Robotics **OpenYAM** arm at Edge City India, running on Dimensional's **DimOS** software.

- Interactive setup guide: https://claude.ai/artifact/CreHEQvaLaHGjhm7SYRQ4p
- DimOS install docs: https://docs.dimensional.org/installation/
- DimOS OpenYAM integration: https://docs.dimensional.org/capabilities/manipulation/openyam_integration/
- DimOS source: https://github.com/dimensionalOS/dimos
- Help: the DimOS Discord (https://discord.gg/dimos). Tule (`1tule`) offered a tested setup blueprint for our unit; ask him for it.

> **Safety first.** Every `dimos ... run` command **without** `--simulation` selects the real arm and switches the motors on. Use the simulation until the hardware session is planned.

## 1. What you need

- Ubuntu 24.04 (preferred) or macOS (Apple Silicon tested).
- About 16 GB RAM and at least 10 GB free disk. No GPU is needed for arm control.
- For the physical arm only: a gs_usb/candleLight-compatible USB-to-CAN adapter (USB ID `1d50:606f`), CAN wiring, motor power and correct bus termination. macOS needs no extra USB or CAN drivers.

## 2. Install DimOS

Clone this repo, then run the official installer **inside the repo folder** so it creates `.venv` here:

```sh
git clone https://github.com/Edge-City/openyam-robot-arm.git
cd openyam-robot-arm
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --mode library --extras manipulation --project-dir "$PWD"
```

Tested with DimOS **0.0.14** (Python 3.12). `requirements-tested.txt` lists every package version from the working laptop install. If you installed DimOS somewhere else, point the scripts at it with `export OPENYAM_VENV=/path/to/that/.venv`.

### macOS only: keyboard window fix

On macOS the stock 0.0.14 keyboard window fails with `pygame.error: x11 not available`. Apply the local fix once after installing:

```sh
.venv/bin/python macos-keyboard-fix/apply.py
```

It only patches the exact 0.0.14 release file (checked by SHA-256) and refuses anything else. It runs the keyboard window in a child process using Cocoa, stops jogging when the window loses focus or input goes stale, and cleans up on shutdown. Details: `macos-keyboard-fix/macos-keyboard.patch`. Linux does not need it.

## 3. Run the virtual arm

Double-click `scripts/launch-simulation.command` (macOS), or run:

```sh
./scripts/launch-simulation.command
```

It runs exactly `dimos --simulation run keyboard-teleop-openyam` and takes a few seconds to start. With the macOS fix applied, typing that command yourself (in the activated `.venv`) works too.

1. Open http://127.0.0.1:8095 to see the arm. It may look small at first; zoom toward the center of the grid.
2. Click the small **OpenYAM Keyboard Teleop** window to focus it, then:

| Keys | Motion |
|---|---|
| W / S | forward / back |
| A / D | left / right |
| Q / E | up / down |
| R / F | roll |
| T / G | pitch |
| Y / H | yaw |
| `[` / `]` | open / close gripper |

Release the key (or click away from the window) to stop moving. Esc closes the keyboard window.

**To stop:** Ctrl-C in the terminal, or double-click `scripts/stop-simulation.command`. The stop script only stops this repo's OpenYAM simulation.

Quick check before a demo: launch, click the keyboard window, hold D and watch the arm move in the browser. Restart the demo if the laptop went to sleep.

The simulation is an in-memory arm model with a 3D view. It does not model contact or gravity. Only keyboard jogging has been tested; planning and Execute workflows have not.

## 4. The physical arm

Arm facts from the DimOS docs:

- 6 joints plus a gripper.
- Motors: joints 1-3 are Damiao DM4340 (CAN IDs 0x01-0x03), joints 4-6 are DM4310 (0x04-0x06), the gripper is a DM4310 (send 0x08, reply 0x18). Replies come back on send ID + 0x10.
- Classical CAN at 1 Mbit/s.
- Opening the gripper decreases the motor position (about −2.9 rad fully open, about 0 closed; DimOS opens it during startup calibration).
- Motor supply 24 V. Home = folded rest pose, every joint about 0. Reach about 0.63 m forward, tip up to about 0.78 m (forward kinematics on the DimOS model).
- Joint limits (rad): j1 [−2.618, 3.142], j2 [0, 3.665], j3 [0, 3.142], j4 [−1.693, 1.571], j5 [−1.571, 1.571], j6 [−2.094, 2.094]. Home sits on the j2/j3 lower limits.

Commands (tested 2026-10-16; on macOS the port is the USB serial, on Linux `can0`; see section 5 for the full Linux routine):

```sh
dimos hardware can list                      # macOS, read-only: prints the adapter's USB serial
dimos --can-port <USB-SERIAL|can0> run coordinator-openyam
```

Before first activation:

- Keep the workspace clear and the emergency stop within reach.
- Startup can calibrate the gripper, so expect it to move.
- Stopping DimOS does not reliably drop torque: run `tools/disable_all.py` and check that the LEDs are steady red, or check the console, which reads torque state from the wire.

Not confirmed yet (no manual found): what's in the box, mounting hardware, and whether an e-stop is included. A hardware e-stop on the 24 V line is recommended.

## 5. Real arm from Linux, and the lab tools (2026-10-16)

Tested on Ubuntu 24.04 with DimOS 0.0.14. Full story, findings and an incident report: [experiments/2026-10-16-linux-real-arm/SUMMARY.md](experiments/2026-10-16-linux-real-arm/SUMMARY.md). Running log: [LOG.md](experiments/2026-10-16-linux-real-arm/LOG.md). Working rules for people and agents: [CLAUDE.md](CLAUDE.md).

**One-time Linux setup** (needs sudo): `sudo bash experiments/2026-10-16-linux-real-arm/linux-sudo-setup.sh` (apt packages, udev, LCM buffers), then `sudo bash experiments/2026-10-16-linux-real-arm/can0-up.sh` (brings `can0` up at 1 Mbit/s and on every replug). On Linux the kernel's gs_usb driver owns the adapter; `can_motor_control` uses SocketCAN there, and `--can-port` / the tools take `can0`, not the USB serial.

**Session:**

```sh
dashboard/start-all.sh                                    # console http://127.0.0.1:8090 (+ phone cameras, armctl mock daemon)
.venv/bin/python -I tools/probe_motors.py can0            # read-only; every joint ~0 (check joint 1: it can arrive ~180° off)
experiments/2026-10-16-linux-real-arm/session_play.sh nod # coordinator up → play → home → down → torque off; with the human's go
.venv/bin/python -I tools/disable_all.py can0             # if anything ever looks enabled; the console shows torque state from the wire
```

**What's here:**

- `dashboard/`: lab console with live joints, REAL and SIM 3D views, coach panel, phone cameras/tilt/joystick, playables, music.
- `armctl/`: the safety layer every interface goes through: soft-limit envelope, playable validator, armctl daemon (control lease, deadman, E-STOP), English → playable (`llm_author.py`), song analysis and choreography (`music.py`, `choreo.py`), forward kinematics check (`fk.py`). Spec: [ARMCTL-SPEC.md](experiments/2026-10-16-linux-real-arm/ARMCTL-SPEC.md).
- `playables/`: motion sequences (nod, wave, happy-dance, beat-dance, rom-arm, rom-wrist, …). Validated, previewable, playable on the arm.
- `music/`: drop a song in `music/inbox/`, then analyze → plan → choreography (songs are not committed).
- `vision/`: printable calibration board and tags, camera calibration, tag pose detection.
- `playground/`: docs for the public "type English → motion" web app ([github.com/Nhajela/openyam-playground](https://github.com/Nhajela/openyam-playground)).

Played on the real arm so far: joint jog / ready / home, nod, wave, happy-dance, beat-dance with music, and the range-of-motion tours (rom-arm, rom-wrist). The phone joystick/tilt drive the simulated arm only; the real-arm backend is the next step.

## 6. More interfaces: pose mirror, timeline, grasp, chess (2026-10-28, sim only so far)

[`interfaces/`](interfaces/README.md) adds more ways to move the arm, all as armctl programs (same lease, envelope, deadman, E-STOP):

- **Pose mirror:** a camera (USB, phone, video or photo) and MediaPipe pose, and the arm copies a person's arm, live. There are two mappings: joint angles (copies elbows and posture) or hand position (IK). It guards against the table and stops when it loses the person; `--clutch` makes it follow only while the other hand is raised.
- **Record + timeline editor** (http://127.0.0.1:8097): record any interface, turn the recording into keyframes, and edit them on per-joint lanes. The page shows what each joint can reach in the time given, and says in words which segment is too fast and what it needs. "Stretch" slows it down, "Shrink" keeps the beat.
- **Grasp:** top-down pick and place planned with the new numpy IK (`armctl/kin.py`, no pinocchio needed).
- **Chess:** reach map for the board; legal moves, captures, castling and en passant become pick and place sequences.

Log: [experiments/2026-10-28-interfaces/LOG.md](experiments/2026-10-28-interfaces/LOG.md). **Open play** ([`openplay/`](openplay/README.md)): people scan a QR code on the laptop screen, queue for a turn and drive the arm from their phones (tilt, joysticks, moves, "Say it" with Claude, poses, grab). The laptop screen is both the show and the operator dashboard. `DimosBackend` connects armctl to DimOS and is tested in DimOS `--simulation`. **Start a hardware session with [HANDOVER.md](HANDOVER.md).**

## Real arm on macOS: notes (first tested 2026-10-16)

The steps above are not enough to run the physical arm from a Mac. Found during the first hardware session. Ticked items are now in this repo; the rest still need work.

What worked on 2026-10-16 (support the arm whenever torque may drop):

```sh
uv pip install --python .venv/bin/python -e ./yammac   # once: macOS keyboard that also works on hardware
dimos hardware can list                                 # note the adapter's USB serial
.venv/bin/python -I tools/probe_motors.py <USB-SERIAL>  # read-only; folded pose should read ~0 on every joint
dimos --can-port <USB-SERIAL> run yammac.keyboard-teleop-openyam
# ...jog, return toward folded, hold the arm, Ctrl-C, then:
.venv/bin/python -I tools/disable_all.py <USB-SERIAL>   # force torque off; check every LED is steady red
```

- [x] **Keyboard teleop on the real arm.** `macos-keyboard-fix` is simulation-only: on hardware it raises `Local macOS keyboard bridge is simulation-only`, so there is no keyboard. `yammac/` (vendored here) runs the keyboard window in its own process and adds the blueprint `yammac.keyboard-teleop-openyam`, which works in simulation and on hardware. Still open: decide whether to keep two macOS keyboard fixes or merge them.
- [x] **Wiring notes.** Plug the MKS CANable V2.0 (gs_usb `1d50:606f`) straight into the Mac; through a hub or a charge-only cable it is not detected. On the base cable, thin red wire to CANH, black to CANL, GND empty. Adapter switches: 120 Ω on, BOOT off. Motor supply is 24 V (round DC socket on the base cable).
- [x] **Read-only motor probe.** `tools/probe_motors.py` connects over gs_usb and reads all 7 motors (position, temperature, fault) without enabling torque. Run it before every start.
- [x] **Joint zero check.** In the folded rest pose (upper arm flat pointing back, forearm folded on top, gripper level, joints 2 and 3 on their stops) every joint must read about 0, which is the DimOS model's zero and home pose. Our unit read about -2π on joints 2 and 3, so DimOS refused to drive it. Fixed by a Damiao set-zero on motors 2 and 3 at the folded pose (`tools/rezero_j2_j3.py`, dry run by default). The zero survives a power cycle. Script added; the check is in the commands above.
- [x] **Joint 1 orientation.** Joint 1 reads 0 when the folded arm faces model +x (gripper pointing forward). Positive is counterclockwise seen from above. The range is -150° to +180°, and beyond it DimOS refuses to drive.
- [x] **Stopping does not reliably remove torque.** After Ctrl-C, motors stayed enabled and holding (green LED). `tools/disable_all.py` sends Damiao disable to all 7 motors and flushes it to the bus. Run it after every stop. Support the arm first. Note that `can_motor_control` only sends queued commands on `robot.tick()`, and its `is_enabled` flag read `False` even while LEDs were green, so trust the LEDs.
- [x] **LED meanings** (Damiao, per roboticscenter.ai wiki): green steady = enabled/holding, red steady = powered, torque off, red flashing = fault. A power cycle clears a fault.
- [x] **Joint-limit lockout.** (Also from folded home: j2/j3 sit on their limits and the Cartesian IK fails; start Cartesian control from a ready pose. armctl's soft-limit gate prevents this for scripted motion.) Keyboard jogging to full forward stretch pushed joint 3 past its 3.14 rad limit; DimOS then ignores every key. Recovery: hold the arm, stop DimOS, disable torque, fold by hand. Warn users not to jog to full stretch.
- [x] **No "go home" in keyboard mode.** Solved with the `coordinator-openyam` blueprint plus armctl joint trajectories (`armctl_real.py home`, `session_play.sh`), tested on hardware 2026-10-16. The keyboard blueprint has no joint trajectory task. Moving to a joint target (for example folded home, all zeros) needs `openyam-planner-coordinator` with the `Arm` SDK from `dimos shell`, which is untested on this arm. Test it in `--simulation` first, then on hardware.
- [x] **Session routine.** Start folded on the stops; probe; start; jog; return toward folded; hold the arm; stop; run `disable_all.py`; check all LEDs steady red.
- [x] Update section 4 once the above is done (remove "not yet tested", drop the 24 V item from "Not confirmed yet").

## Known warnings (harmless in the simulation)

- A Pink joint-limit warning at the zero home pose on startup.
- Stale model state for a moment before streams arrive.
- Collision-mesh display unavailable.
- On shutdown, a message about a missing `joint_trajectory` task (the keyboard blueprint doesn't have one).

## Repo layout

```
scripts/              launch, stop and environment scripts (simulation only)
yammac/               macOS keyboard teleop package for simulation and the real arm
experiments/          shared lab notebook: one folder per experiment, with a running log
dashboard/            lab console (live joints, REAL/SIM 3D, cameras, phones, playables, music)
armctl/               safety layer + instruction set: envelope, playables, daemon, LLM author, music/choreo, FK
playables/            motion sequences (JSON) that run on the real arm
music/                song inbox, analysis and choreography plans (songs gitignored)
vision/               calibration targets, camera calibration, tag detection
playground/           docs for the public English → motion web app (separate repo)
tools/                real-arm helpers (macOS gs_usb and Linux SocketCAN): read-only probe, torque-off, joint 2/3 re-zero
macos-keyboard-fix/   macOS keyboard window patch, its helper, tests, and the original release file
sign/                 printable A4 robot-table sign with a QR code to this repo
requirements-tested.txt   package versions from the working install
```

**Robot-table sign:** print `sign/robot-table-sign.pdf` (A4). The QR code opens this repo, so the repo must be public for visitors to see it. To point it somewhere else: `uv run --with "qrcode[pil]" sign/make_sign.py <url>`.

`macos-keyboard-fix/test_keyboard_bridge.py` exercises the keyboard bridge's watchdog and cleanup without any robotics stack: `cd macos-keyboard-fix && python3 -m unittest test_keyboard_bridge`.

DimOS is Apache-2.0 licensed by Dimensional Inc. `macos-keyboard-fix/keyboard_teleop_module.release-0.0.14.py` is an unmodified copy of their release file, kept so the fix can verify what it patches.
