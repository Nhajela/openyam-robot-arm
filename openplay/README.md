# Open play: people drive the arm from their phones

The laptop is connected to the arm and shows a QR code on its screen. Anyone can scan it, join a queue, and get a turn (default 10 minutes). During your turn you can:

- **Drive:** tilt the phone (gyroscope) or use on-screen joysticks, open and close the gripper.
- **Moves:** tap a dance or gesture from the library.
- **Say it:** type what you want ("wave at the kids"). Claude choreographs it, and the arm's safety layer checks it before it plays.
- **Poses:** drive the arm to a position, save it, repeat, then play them all as one smooth sequence. The arm's speed limits are respected, and it tells you where it had to slow down.
- **Grab:** pick an object from one spot on the table and put it on another.
- **Chess:** play the arm (or another player) on a real board. Move your piece by hand and hit the big clock button on your phone; cameras work out your move (a tap board is the fallback) and the arm plays its reply. Chess stays off on the real arm until the board is calibrated (`interfaces/grasp/workcell.json` `calibrated: true`). `--chess-sensor camera` uses the board cameras; the default `fake` has none, so rehearsals use the tap board.
- **Pick it up:** the arm points its wrist camera at the table and the phone shows the picture. Tap the thing you want, or type / say it ("the cup on the table": Claude finds it, and asks you to tap if it isn't sure). The phone outlines what it found ("Pick this?"); on yes the arm homes in with the camera, grabs it across its narrow side, lifts it, checks it's really holding it (one retry), puts it back and returns to ready. Each arm step goes through the lease and the daemon's checks; with auto-play off the operator approves the look and the pick. Off ("camera not calibrated yet") until the wrist camera is calibrated (`workcell.json` `calibrated: true`, `interfaces/wristcam/calib/handeye.json` + `wristcam.json`). Vision: [interfaces/wristcam/README.md](../interfaces/wristcam/README.md#tap-or-say-what-to-pick-targetpy); protocol: [PROTOCOL.md](PROTOCOL.md#pick-it-up-tap-or-say-a-thing-on-the-table-the-arm-picks-it-up).

On the phone a turn plays like a game: **pick a mode** (big cards; modes whose server feature is off are hidden) → **get set** (a short how-to, one tap to start) → **play**. For **Tilt** the get-set step is the start pose: hold the phone in both hands, screen up, top tipped ~30° toward you, elbows at your sides; a dial fills when you hold within ~8° for 1 s and that becomes "still" for the whole turn. Pressing the pad more than ~10° away from it shows "back to your start pose" and sends nothing until you're there (no jumps). Get set sends `ready` for the driving modes. **STOP**, **◀ Modes** and **🏠 Home** (folds the arm on its stops; driving starts from Ready again) are always in the dock. Modes live in one list, `MODES` in `static/play.js`. Screenshots: [screenshots/](screenshots/) (made with `tools/phone_screens.py`, headless Chromium, no arm needed).

The laptop screen is part of the show. It shows the arm live in 3D, the QR, who's playing and who's next, captions of what people asked for, and a "ghost" preview of Claude's choreography. The person at the laptop is the operator: E-STOP, approve moves, skip, extend, pause.

Contract: [PROTOCOL.md](PROTOCOL.md). Handover and status: [../HANDOVER.md](../HANDOVER.md).

## Arcade: the same thing as a claw-machine game

A second phone page, `/arcade` ([`static/arcade.html`](static/arcade.html)), for events where people just want to play. No mode cards and no levers: one game, one verb per thumb.

- **Drag anywhere on the map** to steer the claw (a floating stick under your thumb; Cartesian x/y, like Sticks' left stick). The map is an angled view into a claw machine: the table is the floor, and the claw hangs on its cable at its real height (forward kinematics from the arm's joints).
- **Hold DROP** to lower the claw. **Let go**: it shuts and comes back up to Ready.
- **Stars** to fly the claw into (+100), **gold ⬇ stars** you dive to (+250), combos for quick chains, a score and a best score on the phone. **Emotes** are the library moves; **✨ Ask it** is Say it.

Two more games when the wrist camera is available (tabs at the top of the game; `--cam`):

- **👀 Robot Eyes:** the phone shows what the robot sees. Drag to look around (the base turns, the wrist tilts: joint velocities), pick a filter (🤖 Robot HUD, 🔥 Thermal, 👾 8-bit, 💥 Comic), **📸 SNAP** for a 3-2-1 framed photo ("Seen by the robot", your name, the date) to save or share, 👋 to make it wave at you.
- **🎥 Claw Cam:** the claw game from the claw's own eye: the same steer and DROP, the camera picture instead of the map.

The frames come from the lab console's camera (`--cam console`, the default; the server relays them only to the player whose turn it is, ≤ 10 fps at 640 px), `--cam test` draws a synthetic picture for rehearsals, `--cam off` hides the camera games.

It speaks the same protocol (`stream`, `grip`, `ready`, `move`, `say`, `stop`), so everything in the safety model below applies unchanged; STOP stays on screen above every overlay. Two phone-side conveniences are *not* safety limits: DROP stops at 6 cm above the table (`DROP_FLOOR_Z`), and soft walls keep the claw inside the reach ring and the daemon's j1 soft limits (`welcome.limits`).

Run open play with `--phone-ui arcade` to put it on the QR. `/arcade` and `/classic` always work, whichever `/` serves, and each page links to the other. Notebook: [experiments/2026-10-30-arcade-phone](../experiments/2026-10-30-arcade-phone/LOG.md).

## Chess and Pick it up: off on the real arm unless you ask

`--chess` and `--pick` take `auto` (default), `on` or `off`. **auto** offers them on phones only in rehearsal (mock backend or the DimOS sim), because on the real arm neither works reliably yet: the purple ball was picked with the hand-tuned `look_and_grasp` CLI, the pinch on chess pieces slipped, and the phone's Pick it up has never run on the arm. **on** offers them on the real arm too, and only while they could run (board / camera calibrated). **off** refuses their ops. The classic page's mode cards follow `welcome.features`, so nothing else changes.

## Safety model

Phones never talk to the arm. `openplay.server` is the only armctl program, and it holds the daemon lease. So the armctl envelope (soft joint limits, speed and acceleration caps), the 300 ms deadman on live control (`--stream-ttl-ms`, see "Stream gaps" below), and the daemon's E-STOP apply to everything.

- A turn's motion comes only from the active player; everyone else is refused.
- Playables (moves, Claude's choreography, pose sequences, grabs) are validated by the daemon. The operator presses Play for each one, unless they turn **auto-play** on in the operator drawer (it defaults to on only for the mock backend).
- Keep people out of the arm's reach: tape a line on the table. The operator stays at the laptop with E-STOP within reach. Use a hardware e-stop on the 24 V line when there is one (see the main README).

## Run it

```sh
# 1. the arm: mock for rehearsal; on hardware, the DimOS backend (not built yet: see HANDOVER.md)
.venv/bin/python -m armctl.daemon --backend mock --auto-approve --vel-cap 0.6 --acc-cap 2.0
# 2. open play
.venv/bin/python -m openplay.server                  # phones: https://<laptop-ip>:8450/?k=<key>, display: http://127.0.0.1:8451
# 3. on the laptop: open http://127.0.0.1:8451 full screen (F11). The operator rail is on the right ("h" hides it, Space = E-STOP).
```

For "Say it" with Claude, set `ANTHROPIC_API_KEY` in the server's environment. Without it, Say it plays the closest library move and says so.

**Visual choreography (opt-in):** `OPENPLAY_SAY_VISUAL=1` switches Say it to `armctl.visual_author`: Claude Fable drafts the move, sees render sheets of it from the audience's position and revises it over `OPENPLAY_SAY_PASSES` (default 2) review passes; a live wrist-camera snapshot is added only when the request refers to the real scene ("point at the cup"). Expect 1–4 minutes and roughly $1–3 per request (see [the experiment](../experiments/2026-10-29-visual-choreo/LOG.md)); the phone shows "drafting / reviewing pass 1/2 / final". Timeout `OPENPLAY_SAY_VISUAL_TIMEOUT` (240 s) and the library fallback still apply; short requests that name a library move ("wave", "nod") play the stored move directly.

On the real arm, run the daemon on `DimosBackend` instead of the mock: see [HANDOVER.md](../HANDOVER.md) §4–5. Auto-play then starts **off**, so you approve each move on the laptop (Enter / Esc).

The phone listener binds `0.0.0.0` so phones on the LAN can reach it. That's right on the laptop. **Never run it that way on a server with a public IP:** use `--phone-host 127.0.0.1`.

## Logs: what the phone sent and what we did with it

Every server start writes a time-synced trace, one JSON line per step: each phone message (its send time on the phone, converted to the server clock, when it arrived, the gap since the previous one, what the server did with it), each armctl message (with ack/done latency), daemon refusals, the arm's state at 2 Hz, connection opens and closes (with WebSocket close codes), and phone-side events the server can't see (page hidden, pointer cancelled, socket closed, send timer late). Format: [PROTOCOL.md § Event log](PROTOCOL.md#event-log-time-synced-trace).

- **File:** `experiments/<today>-*/logs/openplay-events-<YYYYMMDD-HHMMSS>.jsonl` (a folder with "openplay" in its name if there is one today). `--trace-log PATH` to choose, `--trace-log ''` for memory only. The path is printed at start-up.
- **Live:** open `http://127.0.0.1:8451/events` on the laptop: phone → server → arm lanes on the server clock, red bands where a stream gap is longer than the ttl, DEADMAN markers, a per-hold table, filters by player and op. `GET /api/streamstats` gives the same per-hold stats as JSON. Loopback only.
- **Server log:** one line per hold: `stream hold #12 Ana (v): 85 msgs in 5.6 s (15.0 Hz), arrival gaps p50/p95/max 66/90/410 ms, phone send p95 70 ms, 1 > ttl 300 ms (1 deadman stops; causes network), release (up)`.
- **Afterwards:** `.venv/bin/python -m openplay.eventlog summarize <file>` prints the per-hold table, the worst gaps with their cause (`phone`: the phone sent late; `network`: sent on time, arrived late; `phone-gate`: the Tilt start-pose gate), connection closes, phone events and a verdict.

Reading the daemon log: it prints "deadman: no setpoint for 300+ ms" after **every** hold, because the release zero is itself a setpoint whose ttl then runs out. Those are harmless. The trace separates them (`deadman_after_release`) from deadman stops in the middle of a hold (`deadman_in_gaps`).

## Stream gaps: if Tilt and Sticks keep stopping

First look at the trace (`summarize`). What each finding means:

1. **Refusals, not gaps.** On 28 Oct the real arm refused 580 Tilt/Sticks setpoints in one session ("Cartesian streaming needs the arm unfolded (j2 < 0.3)"), in runs of 3–6 s at the full 15 Hz. The daemon sends that refusal without a request id, and until 29 Oct open play didn't pass it on, so the phone kept steering an arm that wasn't moving. That looks like a disconnect. **Fixed:** the server relays it to the driver (at most once a second), and the phone shows its "too low: tap Ready" bar and stops streaming. In the trace: `server/relay` and `act: refuse` with that reason.
2. **Gaps with cause `phone`** (the phone's own send times are far apart): the page's 66 ms timer was throttled (screen dimming, low power mode, the page hidden, a busy main thread). Look for `phone/diag` `late_tick`, `hidden` and `pointercancel` around them. Network settings won't help; keep the screen awake (the page asks for a wake lock), and close other tabs.
3. **Gaps with cause `network`** (sent on time, arrived late, often several messages together: TCP head-of-line blocking on Wi-Fi). Mitigations below.

Mitigations, none on by default. The deadman exists to stop the arm when the phone vanishes (dropped, walked out of Wi-Fi, a browser crash), so each one trades some of that:

| Option | What it does | Safety cost |
|---|---|---|
| `--stream-ttl-ms 450` (built; daemon caps at 500) | each setpoint lives 450 ms instead of 300, so a 300–450 ms gap no longer stops the arm | when the phone really vanishes, the arm keeps going 150 ms longer at the last speed before braking: at the defaults +1.2 cm (0.08 m/s) or +0.075 rad (0.5 rad/s). A normal lift still stops at once (the release zero). Envelope gate, edge watchdog and E-STOP are unchanged. Use it only if the trace shows gaps mostly in 300–500 ms |
| Hold-last-setpoint bridging in the server (not built) | for a gap of ≤ N ms the server re-sends the phone's last setpoint, decaying (e.g. halving every 100 ms) | the server, not the daemon, decides that the phone is still there; a server bug could keep the arm moving (the daemon's deadman still covers the server dying). Safer than a long ttl because the speed decays |
| WebRTC data channel, unordered and unreliable (not built) | a lost packet no longer holds up the ones behind it, so `network` bursts disappear | aiortc + ICE on the LAN; more to break at an event. Only helps cause `network` |

Recommendation: run a session with the relay fix and the trace on. If gaps > ttl are rare after that, do nothing more. If they are mostly `network` and under 500 ms, try `--stream-ttl-ms 450` in `--simulation` first, then on the arm with someone at the E-STOP.

## Network: how phones reach the laptop

Phones need **HTTPS**: browsers only give motion sensors (tilt) to secure pages. Latency matters for tilt and joysticks, not for moves or Say it.

| Option | Latency | Setup | Catch |
|---|---|---|---|
| **LAN + self-signed cert** (default) | lowest (same Wi-Fi) | none; the cert is generated in `dashboard/certs/` | every phone sees a certificate warning once ("Show details → visit this website" on iPhone, "Advanced → proceed" on Android). The event Wi-Fi must allow phone-to-laptop traffic (no client isolation) |
| **LAN + real cert on our domain** (recommended upgrade) | lowest | a human adds a DNS record `arm.positivesumexperiments.com → <laptop LAN IP>` and we get a Let's Encrypt cert with a DNS-01 challenge | some routers block public names that resolve to private IPs (DNS rebinding protection); test the venue's Wi-Fi |
| **Cloudflare Tunnel / Tailscale Funnel** | + internet round trip (~50–200 ms, jitter) | `cloudflared tunnel --url https://localhost:8450 --no-tls-verify` or `tailscale funnel 8450` | works from anywhere and on any Wi-Fi with no warnings, but tilt feels laggier. Fine for moves, Say it, poses |
| Tailscale (tailnet only) | low on LAN, relayed otherwise | phones must be on our tailnet | not for the public |

Start with LAN + self-signed at the event. If the Wi-Fi isolates clients, run the laptop's own hotspot and put the QR's Wi-Fi details on the sign.
