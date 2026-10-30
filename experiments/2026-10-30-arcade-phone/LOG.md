# Arcade phone page: open play as a claw-machine game

**Experimenters:** Naman (idea) + Claude (build on serenity, no arm)
**Goal:** A second phone experience for open play that feels like a game, not like driving a JCB. The classic page (`openplay/static/play.html`) gives people levers: a Slow/Normal switch, a grip slider, two joysticks labelled away/toward/up/down/twist, joint bars. Make one more version where each thumb has one verb and there is something to *do* (a goal, a score), on the same protocol and safety model.
**Status:** built (claw game + Robot Eyes + Claw Cam); mock-daemon end to end green with a synthetic camera; real phone, camera and arm next (laptop).

Branch `arcade-phone` (fork). Page: `openplay/static/arcade.{html,css,js}`, served at `/arcade`; `--phone-ui arcade` puts it on the QR.

## 2026-10-30 13:30–14:35 IST: design and build (serenity, mock daemon)

**Decision: a claw machine.** Everyone already knows how one works: steer the claw over the prize, drop, it grabs and comes back. That maps onto what the arm does well in Cartesian streaming, and it removes every lever:

| Classic page | Arcade page |
|---|---|
| pick one of 9 mode cards, get-set step | one game; a 3-step "how to play" card, PRESS START, 3-2-1-GO |
| two joysticks (x/y and z/yaw), speed switch | **drag anywhere on the map**: a floating stick under your thumb steers the claw in x/y. Speed follows how far you drag (softer near the centre) |
| Up/Down buttons, grip slider, Open/Close | **hold DROP**: the claw lowers; **let go**: it shuts (`grip 0`) and comes back up (`ready`) |
| numbers in radians, "away/toward" labels | an angled view into a claw machine: the table as the floor, the claw hanging on its cable at its real height (forward kinematics from the arm's `q`), its shadow and trail on the floor, the play zone drawn as a lit sector |
| Moves list | **emotes** row (same library moves), +25 the first time each |
| Say it textarea | ✨ **Ask it** sheet (same `say`) |
| no goal | **stars** to fly the claw into (+100), **gold ⬇ stars** you must dive to (+250, the tip below 17 cm), **combo** x2..x5 for chains within 5 s, +50 for a claw grab that went ≥ 4 cm down, a score, game over screen with stats and the best score on this phone |

Kept from the classic page, unchanged in meaning: STOP always on screen (it stays above every overlay), hold-to-move with an explicit zero on lift, everything released on page hidden / socket close / pointer cancel, the "needs the arm unfolded" refusal → "Too low! Reset the claw" (sends `ready`), the fault banner, the wake lock, the event-log fields (`seq`, `t`, `ep`, `why`, `diag`, ping/sync), so `python -m openplay.eventlog summarize` works on arcade sessions too. `hello` now carries `ui: "arcade"` (and the classic page `ui: "classic"`), logged in the trace.

**Server changes (small):** `/arcade` and `/classic` routes (both always on, both need the room key); `--phone-ui classic|arcade` picks what `/` (the QR) serves, default classic; `welcome.limits` = the daemon's soft joint limits, so the page keeps stars and side walls inside the site's j1 range; `ui` in the hello trace. PROTOCOL.md updated.

**Problem / Fix:** the daemon has no table floor for Cartesian streaming (the classic Down button can push the gripper into the table). The arcade's DROP stops sending `v.z < 0` once the FK tip is at `DROP_FLOOR_Z` = 0.03 m in the base frame (about 6 cm above the table, which is 3.1 cm below the base). **This is a phone-side convenience, not a safety limit**: it uses the reported `q` (10 Hz, so ~1 cm overshoot at 0.08 m/s) and uncorrected FK. Objects on the table can still be hit. The daemon's envelope, the deadman and E-STOP remain the safety.

**Problem / Fix:** the first star layout used ±55° around the base. On the lab table j1 is soft-limited to −0.6..+1.2 rad (glass wall on the arm's right, `armctl/site_limits.json`), so stars on the right were unreachable and the claw hit "The arm can't go further that way (j1 would reach -0.6…)". On the YAM the tip's angle on the map equals j1 exactly (checked with `kin.tip` at j1 = −0.6, 0, 0.6, 1.2: atan2(y, x) = j1). Now the server sends the limits, stars spawn inside them, and the page drops the turning component at `J1_EDGE` = 0.12 rad inside them. The daemon's refusal, if it still comes, shows as "🧱 Wall! Steer the other way".

**Problem / Fix:** after letting go of DROP the claw shut but never came back up. A `grabbing` flag was cleared by the next arm-state message (10 Hz) before the 800 ms timer that sends `ready`. Now an explicit state (`closing` → `returning`, cleared when the `ready` it sent has run, or after 3 s if it was refused).

**Insight:** the first flat top-down map used a third of the screen (the reach is a half-disc, the phone is tall). The angled "inside a claw machine" view uses the height for the claw's altitude, so one picture shows where it is over the table *and* how high, which is exactly what a drop needs.

### Test (mock daemon, serenity)

Environment: serenity (Linux 6.8.0-147, Python 3.12.3, FastAPI 0.162.0), branch `arcade-phone` from fork main `cf9a852`, mock backend, headless Chromium (Playwright's headless shell 1248) at 390×844 @3x with touch.

```sh
.venv/bin/python -m armctl.daemon --backend mock --auto-approve --host 127.0.0.1 --port <D>
.venv/bin/python -m openplay.server --daemon ws://127.0.0.1:<D>/armctl --phone-host 127.0.0.1 --phone-port <P> \
    --local-port <L> --key testkey --host-ip 127.0.0.1 --turn 120 --trace-log /tmp/arcade-test/trace2.jsonl --events-log '' --phone-ui arcade
CHROME_PATH=<chrome> NODE_PATH=<dir with playwright> node experiments/2026-10-30-arcade-phone/arcade_smoke.cjs \
    "https://127.0.0.1:<P>/arcade?k=testkey" experiments/2026-10-30-arcade-phone/screens
```

[`arcade_smoke.cjs`](arcade_smoke.cjs) joins, presses start, drags up on the map for ~1.7 s, holds DROP 1.5 s, lets go, then steers toward the nearest star a few times, opens the menu and ends the turn. Result:

```
{"t0":[0.205,0.000,0.432],"t1":[0.345,-0.029,0.426],"dx_away":0.140,"z0":0.426,"z1":0.306,"dropped":0.120,
 "busyAfterDrop":"ready","gripAfter":0,"scoreBefore":250,"scoreAfter":350,"errors":[]}
```

- **Observation:** dragging up moved the tip 14 cm away from the base (+x), the slight right drag gave −y: the mapping matches the classic Sticks. Holding DROP lowered it 12 cm; letting go shut the claw and ran `ready`. Stars were collected by steering alone. No page errors.
- **Observation:** stream holds at 15.2–16.1 Hz, arrival gaps p95 ≤ 68 ms, 0 gaps over the 300 ms ttl (server log `stream hold #…`, loopback, so it says nothing about Wi-Fi).
- `pytest openplay/tests`: 139 passed, 7 failed. The same 7 `test_chess.py` failures happen on clean `main` on serenity (main: 8, one of them flaky): they read the real calibrated `workcell.json`, which existed before this work. New: `test_server_e2e.py::test_arcade_page` (routes, room key, `welcome.limits`).

Screens ([screens/](screens/)): [lobby](screens/1-lobby.png), [how to play + PRESS START](screens/2-intro.png), [steering](screens/3-steering.png), [dropping](screens/4-dropping.png), [grab → coming back](screens/5-grab-return.png), [score and a gold dive star](screens/6-score.png), [menu](screens/7-menu.png), [game over](screens/8-game-over.png).

## 2026-10-30 14:35–14:50 IST: camera games, chess / pick off on the real arm (serenity, mock daemon)

Naman: "is pick up object working at all? … two more game modes … disable chess and pickup if we can't pick up. Game mode 1: look at the camera, just stream the camera, and have fun with it. Game mode 2: … at least a camera feed into the phone."

**Observation: where picking stands (from the notebooks, nothing run today).** On the real arm the purple ball was picked up on 29 Oct with the CLI (`look_and_grasp --finder blob` with hand-tuned flags, [first-pick](../2026-10-29-first-pick/LOG.md)). Chess pieces failed: the pinch closed on a round head and slipped ([chess](../2026-10-29-chess/LOG.md)). The phone's **Pick it up** mode has only run in simulation; open play on the arm was only used for driving and moves. So: picking works in a narrow, tuned case from the command line, not from the phone.

**Decision: `--chess` / `--pick auto|on|off`.** `auto` (default) offers them on phones only in rehearsal (mock backend or the DimOS sim); `on` also on the real arm, and only while they could run (calibrated board / camera); `off` refuses their ops. Visibility lives in `OpenPlay.phone_features()`; the server-side refusals (uncalibrated board, camera) are unchanged, so the existing chess and pick tests keep their meaning. The server re-sends `welcome` when the arm connects (that's when it knows mock / sim / real).

**Built: the camera relay** ([`openplay/cam.py`](../../openplay/cam.py)). One thread pulls JPEGs from the lab console (`--cam console`, default; `$VISION_DASHBOARD_URL`), shrinks them to 640 px and keeps the latest, only while someone asked in the last 3 s (plus a probe every 15 s so `features.cam` is current). `GET /cam.jpg?pid=&after=` on the phone listener, active player only (Wi-Fi), long-polls for a newer frame. `--cam test` draws a synthetic scene (wall with three cartoon faces, table, blobs) that slides with j1/j4, for rehearsals and these screenshots. Read-only.

**Built: two camera games in the arcade** (tabs at the top: 🕹️ Claw · 👀 Robot Eyes · 🎥 Claw Cam; the tabs only appear while the camera answers):
- **👀 Robot Eyes:** the phone shows the wrist camera with a robot HUD. Drag to look around: joint-velocity stream `qd = [−0.6·dx, 0, 0, −0.6·dy, 0, 0]` (base turns, wrist tilts; full drag 0.3 rad/s at the default `joint_rps`). Directions from FK: at ready the camera looks down the table and a little up (direction (0.96, 0, 0.30)); +j4 raises it ((0.62, 0, 0.78) at j4 = 0.6), +j1 turns it left. Filters drawn on the phone: 🤖 Robot (green scanlines), 🔥 Thermal, 👾 8-bit, 💥 Comic. **📸 SNAP:** 3-2-1, flash, a framed 1080×1350 photo ("SEEN BY THE ROBOT", name, date) to save or share (Web Share with a file where the phone supports it). +50 for the first three photos. 👋 plays the library's wave.
- **🎥 Claw Cam:** the claw game from the claw's eye: the same steer (base frame, never flipped) and DROP, the camera picture with a crosshair and the claw's height above the table.

**Problem / Fix:** the new tab buttons were styled with `.game`, which is also the class of the game screen itself (`section.screen.game`): the whole turn screen became a pill shape, clipping the HUD and swallowing the menu tap. Renamed to `.gametab`.

**Problem / Fix:** a 16:9 camera frame cover-fitted into the tall arena showed only the middle third. The viewfinder now fits the whole frame (`contain`); only the photo crops (`cover`).

**Insight:** at the ready pose the claw points forward and slightly up, not down. Cartesian streaming keeps that orientation, so the claw game's DROP lowers a forward-pointing gripper (it grabs from the side). A fingers-down "claw pose" (taught with Poses on the arm, then used as the claw game's start instead of Ready) would make it a real claw machine. Next on the arm.

### Test (mock daemon + `--cam test`, serenity)

Same environment as above; open play started with `--phone-ui arcade --cam test`. [`arcade_smoke.cjs`](arcade_smoke.cjs) now also opens Robot Eyes, drags up-right for 1.2 s, picks Thermal, snaps a photo, and opens Claw Cam:

```
{"dx_away":0.140,"dropped":0.123,"busyAfterDrop":"ready","gripAfter":0,"scoreBefore":50,"scoreAfter":350,
 "eyes":{"dj1":-0.254,"dj4":0.310,"photos":1},"errors":[]}
```

- **Observation:** drag right turned the base right (j1 −0.25 rad), drag up raised the camera (j4 +0.31 rad); the photo was made; no page errors.
- `pytest openplay/tests`: 146 passed, 7 failed, all in `test_chess.py` and all failing on clean `main` on serenity too (the repo's `workcell.json` is now `calibrated: true`, which these tests assume is false; one flaky). New: [`openplay/tests/test_modes.py`](../../openplay/tests/test_modes.py) (auto / on / off visibility and refusals, the relay's frames and failure reporting, the synthetic frame).

Screens: [Robot Eyes](screens/9-robot-eyes.png), [Thermal](screens/10-robot-eyes-thermal.png), [the photo](screens/11-photo.png), [Claw Cam](screens/12-claw-cam.png) (the test camera).

### Next (laptop, with the arm)
1. Mock first on the laptop: `--phone-ui arcade`, play on a real phone over the LAN; check the feel of the floating stick and DROP with real touch latency. With the lab console running, `--cam console`: does the relay keep up on the event Wi-Fi (frame rate on the phone, the console's camera load)?
2. Robot Eyes on the arm: is 0.3 rad/s a good look-around speed? Do people find the photo? Watch j4's temperature (looking up holds the wrist under load).
3. Teach a fingers-down claw pose and make it the claw game's start pose.
4. DimOS sim, then the real arm with someone at the E-STOP: does the angled map match what people see from where they stand? (Menu → Flip the map if they stand across the table.) Does `DROP_FLOOR_Z` = 0.03 stop at a sensible height over the real table? Do the gold stars' dives stay clear of whatever is on the table?
5. Watch real people play: do they find DROP without being told? Is 0.08 m/s too slow for a game (`--lin-mps`)?

**Ideas:** a two-finger twist on the map for yaw; real prizes (a few foam blocks on the table and the wrist camera confirming a grab, from `interfaces/wristcam`) instead of +50 for any drop; the score and a "now playing" claw view on the laptop display; a shared high-score table on the server.
