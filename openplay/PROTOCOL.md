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
| `stop` | | stop whatever is happening (accepted from the active player at any time) |
| `pose.save` | | remember the arm's current pose (q + grip) in your pose list |
| `pose.delete` | `i` | |
| `pose.clear` | | |
| `pose.goto` | `i` | move to saved pose `i` |
| `seq.play` | `style`: `gentle` \| `lively` | play your saved poses in order as one smooth timeline that respects the arm's speed limits |
| `grab` | `from`, `to` (spot names from `welcome.spots`) | pick an object at one spot and put it at another |
| `ping` | | → `pong` |

### Server → phone

| op | fields |
|---|---|
| `welcome` | `pid, name, moves: [{name, title, emoji, duration_s, blurb}], spots: [{name, title}], turn_s, features: {say, grab, tilt, joystick, poses}, speeds: {lin_mps, ang_rps, joint_rps}` |
| `lobby` | `queue: [{name, you}]`, `position` (1-based, null if not queued), `active: {name, remaining_s} \| null`, `you_active: bool`, `eta_s` (null if unknown). Sent on every change and once a second while a turn is running |
| `turn` | `remaining_s`: your turn started (and once a second during it; `warning: true` in the last 30 s) |
| `turn_end` | `reason`: `time` \| `left` \| `idle` \| `skipped` \| `operator` |
| `arm` | `q: [6], grip, moving, busy` (name of the running action or null), `mode`. 10 Hz to the active player, 2 Hz to everyone else |
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
| `event` | `t` (unix s), `who` (name or null), `kind`: `join` \| `leave` \| `turn` \| `turn_end` \| `move` \| `say` \| `seq` \| `grab` \| `stop` \| `refused` \| `operator` \| `estop`, `text` (a caption to show) |
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

HTTP on the display listener: `GET /` (display page), `GET /qr.png` (QR of the join URL), `GET /api/join` (`{url}`), `GET /urdf/...` (yam_description if found), `GET /static/...`.

## Server rules

- Only the active player's motion ops reach the arm; anyone else gets `refused {reason: "not your turn"}`. `stop` from the active player always goes through.
- One discrete action at a time (move, say, seq, grab, ready, pose.goto). A new one while busy is refused with what's running. A `stream` during an action is refused too (send `stop` first).
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
