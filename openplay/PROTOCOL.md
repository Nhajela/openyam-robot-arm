# Open play: protocol and module contracts

Open play turns the arm into a public station. The laptop is connected to the arm and shows a QR code on its screen. People scan it with their phone, join a queue, and get a **turn** (default 10 minutes) during which they hold the arm. On their phone they can tilt to steer, use a joystick, tap dance moves, type what they want ("wave at the kids", sent to Claude), record poses and play them back, and grab things. The laptop screen is part of the show: a QR, the arm in 3D, who's playing, the queue, live captions of what people asked for.

```
phones (https, LAN) ──wss /ws──┐
                               openplay.server ── one armctl program "openplay" ──> armctl daemon ──> backend (mock | DimOS)
laptop display (http 127.0.0.1) ──ws /display/ws──┘        (holds the lease, arbitrates players)
```

`openplay.server` is the **only** armctl client. It holds the daemon lease the whole time (program `openplay`) and decides which player's commands reach the arm. Every motion still passes the daemon's envelope gate, deadman and E-STOP.

## Ports and processes

| What | Default | Notes |
|---|---|---|
| armctl daemon | `ws://127.0.0.1:8091/armctl` | run with `--vel-cap 0.6 --acc-cap 2.0` so library "dance" moves pass the daemon's gate |
| phone listener | `https://0.0.0.0:8450` | self-signed cert from `dashboard/certs/` (`phonecam.ensure_cert`); every route needs the room key: `?k=<key>` once, then cookie `op_key` |
| display + operator listener | `http://127.0.0.1:8451` | loopback only, no key; the laptop browser opens `/` (full screen) |

The join URL is `https://<laptop LAN IP>:8450/?k=<room key>`, shown as a QR on the display. The room key is random per server start unless `--key` is given.

## Phone ⇄ server (`wss://…:8450/ws`)

JSON objects, one per WebSocket message. Unknown ops get `refused`.

### Phone → server

| op | fields | meaning |
|---|---|---|
| `hello` | `pid` (client-made id kept in localStorage), `name` (≤ 24 chars) | first message; reconnecting with the same `pid` keeps your place and turn |
| `join` / `leave` | | enter or leave the queue (leaving during your turn ends it) |
| `stream` | `v: {x,y,z,roll,pitch,yaw}` each in −1..1, **or** `qd: [6]` each in −1..1; optional `grip` 0..1 | live control. Send at 10–20 Hz while the finger/tilt is active; when you stop sending, the arm stops (300 ms). The server scales −1..1 to its configured max speeds |
| `move` | `name` | play a library move (see `moves` in `welcome`) |
| `say` | `text` (≤ 200 chars) | English to motion via Claude; progress comes back as `say` messages |
| `grip` | `value` 0 (closed)..1 (open) | |
| `ready` | | go to the ready pose |
| `home` | | fold the arm onto its stops (the daemon's `home`; the last stretch is slow). Status `Going home` then `Home`. Live tilt and sticks need the ready pose, so after `home` the phone sends `ready` before driving again (its get-set step does this) |
| `stop` | | stop whatever is happening (accepted from the active player at any time) |
| `pose.save` | | remember the arm's current pose (q + grip) in your pose list |
| `pose.delete` | `i` | |
| `pose.clear` | | |
| `pose.goto` | `i` | move to saved pose `i` |
| `seq.play` | `style`: `gentle` \| `lively` \| `flow` | play your saved poses in order as one smooth timeline that respects the arm's speed limits (`flow`: passes through the poses without stopping) |
| `grab` | `from`, `to` (spot names from `welcome.spots`) | pick an object at one spot and put it at another |
| `ping` | | → `pong` |

### Server → phone

| op | fields |
|---|---|
| `welcome` | `pid, name, moves: [{name, title, emoji, duration_s, blurb}], spots: [{name, title}], turn_s, features: {say, grab, tilt, joystick, poses, chess, pick}, speeds: {lin_mps, ang_rps, joint_rps}` |
| `lobby` | `queue: [{name, you}]`, `position` (1-based, null if not queued), `active: {name, remaining_s} \| null`, `you_active: bool`, `eta_s` (null if unknown). Sent on every change and once a second while a turn is running |
| `turn` | `remaining_s`: your turn started (and once a second during it; `warning: true` in the last 30 s) |
| `turn_end` | `reason`: `time` \| `left` \| `idle` \| `skipped` \| `operator` |
| `arm` | `q: [6], grip, moving, busy` (name of the running action or null, e.g. `ready`, `home`, a move title), `mode`. 10 Hz to the active player, 2 Hz to everyone else |
| `status` | `text, kind`: `info` \| `busy` \| `done` \| `error`: human-readable result of the last command ("Playing Happy dance (8 s)") |
| `say` | `stage`: `thinking` \| `playing` \| `done` \| `error`; `text` (what they asked), `title`, `duration_s`, `note` (e.g. "no Claude key: playing the closest library move") |
| `poses` | `list: [{i, q, grip}]` |
| `seq` | `stage`: `playing` \| `done` \| `error`, `duration_s`, `notes: [str]` (e.g. "pose 2 → 3 takes at least 1.6 s: j3 moves 1.2 rad") |
| `refused` | `reason` ("not your turn", "busy: playing wave", "E-STOP is on", …) |
| `pong` | |

## Display ⇄ server (`ws://127.0.0.1:8451/display/ws`)

### Server → display

| op | fields |
|---|---|
| `hello` | `join_url, backend, adapter` (`mock` \| `sim` \| `real`; unknown on a non-mock backend means treat it as real), `turn_s, auto_play, paused, moves, spots, envelope` (armctl envelope `describe()`), `poses` (named poses), `urdf: bool` (true when `/urdf/i2rt/yam.urdf` is served), `lease` (does openplay hold the daemon lease), `daemon` (`connected` \| `waiting for lease` \| `estop` \| `offline`). Right after `hello` the server also sends the current `lobby`, `pending` (or null) and the in-progress `say`, so a reconnecting display shows the full state |
| `arm` | `q, grip, moving, busy, mode, estop, lease, adapter` at 20 Hz |
| `lobby` | `queue: [{pid, name}], active: {pid, name, remaining_s} \| null, players: int` (the `pid`s are needed for kick) |
| `event` | `t` (unix s), `who` (name or null), `kind`: `join` \| `leave` \| `turn` \| `turn_end` \| `move` \| `say` \| `seq` \| `grab` \| `chess` \| `stop` \| `refused` \| `operator` \| `estop`, `text` (a caption to show) |
| `say` | as for the phone, plus `who`, plus `keyframes` (the playable's keyframes, for a ghost preview) on `playing` |
| `pending` | `pending: {id, who, title, duration_s, keyframes} \| null`: an action waiting for the operator (only when `auto_play` is off) |
| `config` | `auto_play, paused, turn_s` after a change |
| `refused` / `error` | `reason`: shown in the operator rail's error list |

### Display → server (operator; loopback only)

| op | fields |
|---|---|
| `approve` / `deny` | `id` of the pending action |
| `skip` | end the current turn now |
| `extend` | `s`: add seconds to the current turn |
| `kick` | `pid` |
| `auto_play` | `on: bool` |
| `pause` | `on: bool`: no new turns start (the current one finishes) |
| `turn_s` | `s` |
| `estop` / `estop_reset` | forwarded to the daemon's operator API |

HTTP on the display listener: `GET /` (display page), `GET /qr.png` (QR of the join URL), `GET /api/join` (`{url}`), `GET /urdf/...` (yam_description if found), `GET /chess/frame/<name>` (latest board-camera JPEG), `GET /static/...`.

## Chess (a real board, the chess clock, the cameras)

One game at a time: there is one real board on the table. Humans move their own pieces by hand and then press the chess clock ("I've moved"). The server compares a reference snapshot of the board with a new one (`interfaces/chess_vision/sensor.py`: `BoardSensor.snapshot()`, `rank_moves`, `decide`) to find which legal move was made. The arm plays the engine's moves on the board (pick and place, captures to the graveyard row, castling, en passant; a promotion asks a human to swap the piece) through the normal validate → operator approval → Play path. Sensor: `--chess-sensor fake` (default; `FakeBoardSensor`, which sees nothing a human does, so in mock/sim rehearsals humans use the tap fallback) or `--chess-sensor camera` (`interfaces.chess_vision.camera.CameraBoardSensor`; if it can't be imported or started, chess is refused with the reason).

### Phone → server

| op | fields | meaning |
|---|---|---|
| `chess.new` | `opponent`: `engine` \| `human`, `color`: `white` \| `black` | active player only. Refused while another game with moves is on and its players are still here, and on a real arm while `workcell.json` has `calibrated: false` |
| `chess.accept` | | play the other side of a waiting human game (anyone connected, while the game's owner has the arm) |
| `chess.clock` | | "I've moved": the player to move pressed the clock |
| `chess.pick` | `uci` | choose one of the `candidates` the camera was unsure between |
| `chess.move` | `uci` (e.g. `e2e4`, `e7e8q`) | the tap-to-move fallback: "this is the move I made on the board". Checked with python-chess; illegal moves are refused with the squares that piece can go to |
| `chess.resign` | | resign (or cancel a game nobody accepted) |

The game's moves are accepted only while the active player is one of its players (the opponent in a human game may move from the lobby screen). Human moves never move the arm; only the engine's moves do.

### Server → phone and display: `chess`

`{op: "chess", game, calibrated, arm_ok, refusal, sensor}` on every change, after `welcome` and after the display's `hello`. `refusal` is why chess can't run now (uncalibrated board on a real arm, camera unavailable) or null; `calibrated` mirrors `workcell.json`. The display's copy adds `frames` (camera names; `GET /chess/frame/<name>` on the display listener returns the latest JPEG) and `frames_t`.

`game` is null or: `id, opponent, status` (`waiting` \| `playing` \| `over`), `fen, turn, check, white, black` (`{name, engine}`; the display's copy adds `pid`), `last` (`{uci, san, from, to, by, how}`; `how`: `camera` \| `picked` \| `tap` \| `arm`), `san` (move list), `captured, moving` (the arm is playing this SAN), `thinking, looking` (reading the board), `candidates` (`[{uci, san, score}]` to pick from), `unseen` (text: "I couldn't see your move", show the tap board), `attention` (something on the real board needs a human: the operator presses Board fixed), `clock` (`{white, black, running, clock_s}`, seconds left; nobody loses on time), `eval` (`{pawns, source}`, white's material lead), `result, reason`. The phone's copy adds `you` (`white` \| `black` \| null), `your_move`, `owner` and `legal` (UCI moves, only when it's your move).

### Display → server (operator)

| op | meaning |
|---|---|
| `chess.clock` | press the clock for the player to move (key `c`; a USB/BT button can send `c`. Space stays E-STOP) |
| `chess.fixed` | the board has been fixed by hand (`attention` cleared). If it's the arm's move, it tries again |
| `chess.reset` | end the game |

### Flow

1. Human to move: reference snapshot, the human's clock runs.
2. `chess.clock`: snapshot, rank the legal moves. One confident move → applied ("You played Nf3" on phone and display). Several close (always for a promotion) → `candidates`. Nothing → `unseen` and the clock runs again (press again, or tap the move).
3. Engine game: the arm's clock runs while it thinks and moves; with auto-play off the operator approves once per move (a capture is two pick and places, one approval). The playables end at `ready`, clear of the board; then a new reference snapshot and the human's clock.
4. A move that stops half-way (STOP, E-STOP, turn end) is recorded and `attention` asks for a human to finish it by hand; a denied move sets `attention` and the operator's Board fixed retries it.

## Pick it up (tap or say a thing on the table, the arm picks it up)

`openplay/pick_play.py` (server), `static/pick.js` (phone), `static/display_pick.js` (display); vision and the closed loop are `interfaces/wristcam/target.py` + `engine.py`. `welcome.features.pick` is true when the module loads. The mode is **refused** with `camera not calibrated yet (…)` unless `interfaces/grasp/workcell.json` has `calibrated: true` and `interfaces/wristcam/calib/handeye.json` and `wristcam.json` exist (the `pick` message's `refusal` says so before anyone taps). Frames come from the lab console (`$VISION_DASHBOARD_URL`, default `http://127.0.0.1:8090/camera.jpg`).

### Phone → server (active player only)

| op | fields | meaning |
|---|---|---|
| `pick.look` | | move the camera over the table (`look_xy`, `look_h` in `interfaces/wristcam/pick_config.json`, defaults table (0, −0.15), 0.25 m), operator-approved with auto-play off; then a still frame and the joint angles it was taken at (the arm must be still across the grab) |
| `pick.tap` | `frame` (the still's id), `x`, `y` (0..1 across the still) | "this one": the server finds the object's region and its footprint/height on the table → `pick` with `stage: target`. Refused (`error`, picture kept) on the jaws, bare table, off the table, out of reach, cut off at the picture's edge |
| `pick.ask` | `frame`, `text` (≤ 200 chars) | "the cup on the table": Claude (vision, `$WRISTCAM_PICK_MODEL`, default `claude-opus-5-5`, needs `ANTHROPIC_API_KEY`) points at it → same as a tap. Not found → `text` says why; ambiguous or unsure → `candidates` to tap |
| `pick.confirm` | `target` (the target's `id`) | "Pick this": operator approval (auto-play off), then the closed loop. Refused if the target has a `refusal` (too wide / flat / tall) |
| `pick.cancel` | | "Not that": forget the target, keep the picture |

`stop` aborts a running pick at the engine's next step (the arm is stopped at once, as for any action) and keeps the picture; a turn end forgets it.

### Server → phone and display

- `pick.frame` `{kind: "still" | "live", id, w, h, jpeg}`: `jpeg` is a `data:image/jpeg;base64,…` URL (≤ 960 px wide). `still` is the picture to tap (phone and display); `live` frames are the engine's looks while it picks (display only).
- `pick` `{stage, text, error, refusal, frame, target, candidates, progress, result, engine_state}` on every change; the display's copy adds `who`.
  - `stage`: `idle` → `moving` (to the look pose) → `frame` (tap or say) → `finding` → `target` ("Pick this?") → `picking` → `done` | `failed`.
  - `target`: `{id, label, kind (object | cup), xy_table, height, size [long, short] (m), yaw_table, width_mm, height_mm, height_known, sigma_xy, refusal, outline [[x, y] 0..1], point [x, y] 0..1, footprint_table}`.
  - `candidates`: `[{x, y, label}]` (0..1) from Claude when it isn't sure.
  - `progress`: `[{t, text}]` ("👀 Looking for it", "🎯 Centring over it", "🔎 Moving closer", "⬇️ Going down", "✊ Grabbing", "🎉 Lifted it!", "😅 Missed: trying again", "📦 Putting it back", …).
  - `result`: `{ok, lifted, reason, trace, observations}` once the pick ends.

### Display → server (operator)

| op | meaning |
|---|---|
| `pick.cancel` | abort the running pick (the arm stops) or close the panel |

### Flow and safety

1. Look: one `move_joints` (0.25 rad/s) to a camera-down pose over the table, approved like a playable (`pending` with two keyframes).
2. Tap / prompt: nothing moves. The server shows the outline it found and the size; the player confirms.
3. Pick: one operator approval, then `GraspEngine` with the tapped target's tracker: re-centre → closer (0.18 / 0.12 / 0.08 m above the top) → fingers open, down in 2 cm steps → close across the narrow side → lift 8 cm → verify (grip stalled + no longer on the table) → miss: retry once → put it back where it was → ready. Every motion is a daemon `move_joints` / `gripper` (envelope, speed caps ≤ 0.3 rad/s, lease, E-STOP) and is checked by the engine first (soft box, table guard, camera above the object).

## Server rules

- Only the active player's motion ops reach the arm; anyone else gets `refused {reason: "not your turn"}`. `stop` from the active player always goes through.
- One discrete action at a time (move, say, seq, grab, ready, home, pose.goto). A new one while busy is refused with what's running. A `stream` during an action is refused too (send `stop` first).
- A turn ends at time-out, on `leave`, after 60 s with no command, after 20 s disconnected, or by the operator. At turn end the arm stops and returns to `ready`, then the next player starts.
- `auto_play` on: playables are sent with the daemon's Play pressed by openplay (POST `/playable/confirm` on the daemon's loopback operator API). This is an explicit operator choice, shown on the display. It defaults to on for the mock backend and off otherwise. When off, each playable is shown as `pending` until the operator approves it on the laptop. That Approve is the operator pressing Play: openplay then confirms on the daemon itself (the display is loopback-only, like the console), so there is one approval, not two.
- `say` is limited to one at a time per player, 6 per turn, 60 s timeout. Without `ANTHROPIC_API_KEY` (or if Claude fails), it plays the closest library move by keyword and says so.
- Streams: `v` is scaled to `lin_mps` (default 0.08 m/s) and `ang_rps` (default 0.6 rad/s); `qd` to `joint_rps` (default 0.5 rad/s). ttl 300 ms.

## Module contracts (Python)

```python
# openplay/moves.py  (moves catalogue; the data lives in openplay/moves.json)
def catalogue() -> list[dict]          # [{name, title, emoji, duration_s, blurb, tags}] — only moves that validate
def load(name: str) -> dict            # the playable (from playables/<name>.json)
def match(text: str) -> str | None     # closest move for free text ("do a little dance" -> "happy-dance"), keyword based

# interfaces/timeline/poses.py  (pose recorder -> playable)
def sequence_playable(poses: list[dict], name: str = "my-sequence", style: str = "lively",
                      caps: str = "dance", start_from=None) -> tuple[dict, list[str]]
    # poses: [{"q": [6], "grip": 0..1}], in order. Returns a valid playable (starts at 'ready',
    # visits each pose, returns to 'ready') and human notes. Raises ValueError(reason) if a pose is
    # outside the soft box or too close to the table.

# interfaces/grasp/spots.py  (named places on the table)
def list_spots() -> list[dict]         # [{name, title, xyz_table}] from workcell.json "spots"
def grab_playable(src: str, dst: str, obj: str = "block") -> tuple[dict, dict]
    # pick_place between two spots for a known object ("block": 4 cm cube). Raises ValueError/Unreachable.

# openplay/say.py  (English -> playable)
async def author(text: str, timeout_s: float = 60) -> dict
    # {"playable": dict, "title": str, "source": "claude" | "library", "note": str}
```
