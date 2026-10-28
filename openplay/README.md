# Open play: people drive the arm from their phones

The laptop is connected to the arm and shows a QR code on its screen. Anyone can scan it, join a queue, and get a turn (default 10 minutes). During your turn you can:

- **Drive:** tilt the phone (gyroscope) or use on-screen joysticks, open and close the gripper.
- **Moves:** tap a dance or gesture from the library.
- **Say it:** type what you want ("wave at the kids"). Claude choreographs it, and the arm's safety layer checks it before it plays.
- **Poses:** drive the arm to a position, save it, repeat, then play them all as one smooth sequence. The arm's speed limits are respected, and it tells you where it had to slow down.
- **Grab:** pick an object from one spot on the table and put it on another.
- **Chess:** play the arm (or another player) on a real board. Move your piece by hand and hit the big clock button on your phone; cameras work out your move (a tap board is the fallback) and the arm plays its reply. Chess stays off on the real arm until the board is calibrated (`interfaces/grasp/workcell.json` `calibrated: true`). `--chess-sensor camera` uses the board cameras; the default `fake` has none, so rehearsals use the tap board.

The laptop screen is part of the show. It shows the arm live in 3D, the QR, who's playing and who's next, captions of what people asked for, and a "ghost" preview of Claude's choreography. The person at the laptop is the operator: E-STOP, approve moves, skip, extend, pause.

Contract: [PROTOCOL.md](PROTOCOL.md). Handover and status: [../HANDOVER.md](../HANDOVER.md).

## Safety model

Phones never talk to the arm. `openplay.server` is the only armctl program, and it holds the daemon lease. So the armctl envelope (soft joint limits, speed and acceleration caps), the 300 ms deadman on live control, and the daemon's E-STOP apply to everything.

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

On the real arm, run the daemon on `DimosBackend` instead of the mock: see [HANDOVER.md](../HANDOVER.md) §4–5. Auto-play then starts **off**, so you approve each move on the laptop (Enter / Esc).

The phone listener binds `0.0.0.0` so phones on the LAN can reach it. That's right on the laptop. **Never run it that way on a server with a public IP:** use `--phone-host 127.0.0.1`.

## Network: how phones reach the laptop

Phones need **HTTPS**: browsers only give motion sensors (tilt) to secure pages. Latency matters for tilt and joysticks, not for moves or Say it.

| Option | Latency | Setup | Catch |
|---|---|---|---|
| **LAN + self-signed cert** (default) | lowest (same Wi-Fi) | none; the cert is generated in `dashboard/certs/` | every phone sees a certificate warning once ("Show details → visit this website" on iPhone, "Advanced → proceed" on Android). The event Wi-Fi must allow phone-to-laptop traffic (no client isolation) |
| **LAN + real cert on our domain** (recommended upgrade) | lowest | a human adds a DNS record `arm.positivesumexperiments.com → <laptop LAN IP>` and we get a Let's Encrypt cert with a DNS-01 challenge | some routers block public names that resolve to private IPs (DNS rebinding protection); test the venue's Wi-Fi |
| **Cloudflare Tunnel / Tailscale Funnel** | + internet round trip (~50–200 ms, jitter) | `cloudflared tunnel --url https://localhost:8450 --no-tls-verify` or `tailscale funnel 8450` | works from anywhere and on any Wi-Fi with no warnings, but tilt feels laggier. Fine for moves, Say it, poses |
| Tailscale (tailnet only) | low on LAN, relayed otherwise | phones must be on our tailnet | not for the public |

Start with LAN + self-signed at the event. If the Wi-Fi isolates clients, run the laptop's own hotspot and put the QR's Wi-Fi details on the sign.
