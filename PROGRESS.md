# Progress log

What got done, when, how it was checked, and what's next. Newest session on top. Use one entry per working session, with commit hashes. For the physical setup and detailed measurements, see the lab notebook (`experiments/*/LOG.md`). To start a new session, read [HANDOVER.md](HANDOVER.md) first.

**Legend:**
- ✅ done and tested (where it was tested is written next to it)
- 🧪 built, but only tested in simulation
- ⏳ not started or blocked
- ❗ needs a decision from Naman

---

## 2026-10-28: interfaces, open play, DimosBackend (serenity server; no arm)

**Branch:** `interfaces-mirror-timeline-grasp`, 22 commits on top of `d4a5c10` (PR #2). **Tests:** `pytest armctl/tests interfaces/tests openplay/tests` gives 253 passed, 2 skipped. **Where it ran:**
- armctl mock backend
- DimOS 0.0.14 `--simulation`
- real phones' browsers simulated with Playwright

**Never on the real arm.** Notebook: [experiments/2026-10-28-interfaces/LOG.md](experiments/2026-10-28-interfaces/LOG.md).

### Morning: interfaces (10:13)
- 🧪 `4db9e1e`: a numpy FK/IK (`armctl/kin.py`, matches pinocchio) plus three new ways to move the arm:
  - **pose mirror**: a camera watches a person and the arm copies their arm
  - **timeline coach + recorder + editor**: says in words what's too fast, with stretch/shrink fixes
  - **pick-and-place planner + chess rules**

  Found along the way: a pose-following path could dip toward the table between two safe targets. A table guard was added and checked.

### Open play, built by five agents in parallel against one contract (10:16–10:46)
- ✅ `ad7dc4b`, `2df2e02`: [openplay/PROTOCOL.md](openplay/PROTOCOL.md), the phone ⇄ server ⇄ laptop contract and the server rules.
- 🧪 `e233a5e`: **phone page**. Lobby and queue, turn timer, STOP always visible. Tabs: drive (tilt, sticks, joints, gripper), moves, Say it, poses, grab.
- 🧪 `0bec759`, `08690fb`: **laptop display**. The show (3D arm, QR, now playing, queue, captions, Claude card with a ghost preview) and an always-on operator rail: E-STOP, approve, auto-play, pause, skip, extend, kick, health, joints.
- 🧪 `bcce7ac`, `5d3056a`, `f00bc5c`: **server**.
  - queue and turn rules
  - one armctl program holding the lease and deciding whose commands reach the arm
  - Say it via Claude, with a library fallback
  - an e2e test with 2 phones and a display
- 🧪 `d7619ba`: **13 new moves**, 17 on the phone. All validated and checked against the table with FK. A new `bow` replaces `graceful-bow`, which never lowered the tip.
- 🧪 `3c6698e`: **pose recorder** (save poses, then play them as one smooth sequence within the speed caps).
- 🧪 `587a6a3`: **grab between named spots** (the spots are placeholders).
- 🧪 `bf63389`: the mock arm's Cartesian twist now uses the real Jacobian, so phone tilt moves the sim correctly.
- ✅ `3faa221`: **chess with a real set**, design only ([interfaces/grasp/CHESS.md](interfaces/grasp/CHESS.md)): overhead camera, legal-move matching, tap-to-move fallback.

### The path to the real arm (11:00–11:07)
- 🧪 `98ebd08`, `1fbfbd1`: **`DimosBackend`**: the armctl daemon now drives a DimOS coordinator.
  - Results against DimOS `--simulation`: 27/27 checks pass, final error 0.0000, first motion ~30 ms, deadman stop 0.3–0.4 s, E-STOP 8 ms.
  - The zenoh bus defaults now follow CLAUDE.md (sim 7450; real 7449 only when exported).
- ✅ `b25fbc6`: daemon fixes needed for hardware:
  - per-joint arrival tolerance for the real arm's sag
  - E-STOP reset clears the backend's latch
  - `relax` gets a clean refusal instead of crashing
- 🧪 `84ddb31`: the display said "REAL ARM" on the DimOS simulator. Sim vs real now comes from the daemon, and anything unknown is shown as real.
- 🧪 Full stack on the DimOS sim: coordinator → daemon (DimOS) → open play → phone. Auto-play stayed off, the operator approved, and DimOS played the move as a 227-point trajectory.

### Docs (11:03–11:09)
- ✅ `9fb380d`, `45e5baf`, `7fdc799`:
  - [HANDOVER.md](HANDOVER.md): pull, index, status, rehearsal, a step-by-step first hardware session
  - the lab log and the indexes
  - this file

### Incidents
- On serenity, the phone listener was bound to the public IP for a few seconds (behind the room key). It was restarted on loopback, and the README now says to use `--phone-host 127.0.0.1` on servers.

### Next
1. ⏳ **Laptop + arm:** rehearse on DimOS sim, then the first hardware session in [HANDOVER.md §5](HANDOVER.md). The order: coordinator `--real`, ready/home, gripper direction, joystick, moves at 0.3 rad/s, open play with auto-play off, mirror with `--clutch`, calibration, first grasp.
2. ✅ Merged to `main` and pushed to the fork `Nhajela/openyam-robot-arm` (Edge-City is pull-only for our account). ❗ Still open: an Anthropic key and spending cap for Say it, and the event network (LAN self-signed vs our domain vs tunnel).
3. ⏳ Chess mode in open play (phone board + display panel), then board vision v1.
4. ⏳ A spline ease (flowing sequences), gravity feedforward or measured sag, and torque state from CAN.
