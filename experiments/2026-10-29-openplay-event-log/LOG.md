# 2026-10-29 open play event log (why Tilt and Sticks "disconnect")

**Experimenters:** Naman (+ Claude agent)
**Goal:** The station operator asked for "proper logs stream for what the phone sends and what we do with a synced time". On the real arm Tilt and Sticks "often disconnect" while buttons (Home etc.) work. Build an end-to-end, time-synced trace (phone → server → armctl), per-hold stream stats and a viewer, so the cause can be read off the log instead of guessed.
**Status:** built + tested in the mock harness; first real-arm session with the trace next.

## Environment
Lab laptop, Linux 6.8.0-48-generic, Python 3.12.3 (`/home/naman/openyam-robot-arm/.venv`), base commit `2d1ae2e`. No hardware used: everything ran against `MockBackend` (an in-process armctl daemon on an ephemeral loopback port, and the existing subprocess e2e on 8591/8550/8551). No CAN, no DimOS, no zenoh.

## Log

### 08:00 What the 28 Oct logs already say
Hypothesis from the station: Wi-Fi jitter or phone timer throttling makes stream messages arrive more than the deadman ttl (300 ms) apart.

Counts from [../2026-10-28-laptop-systems-check/logs/](../2026-10-28-laptop-systems-check/logs/):
```
phone2-daemon.log:     stream-start 142  deadman 111  "needs the arm unfolded" 133  edge of reach 66
phone-real-daemon.log: stream-start  34  deadman  34  "needs the arm unfolded" 580  edge of reach 0
```
`grep unfolded phone-real-daemon.log | awk '{print $2}' | uniq -c`: 52 runs of 14–15 refusals per second (the phone's full 15 Hz), each 3–6 s long, e.g. `13:39:37 14, :38 15, :39 15, :40 14, :41 15, :42 10`.

- **Insight:** the "deadman" lines are not evidence of jitter. The phone's release is an all-zero `stream`, which is itself a setpoint; its ttl runs out 300 ms later and the daemon logs "deadman" (and the text said "200+ ms" whatever the ttl was). So there is about one deadman line per hold (34 of 34 in phone-real). With 1 s timestamps nothing in these logs can show a 400 ms gap.
- **Insight (the likely "disconnect"):** `DimosBackend.set_twist` refuses Cartesian streaming when j2 < 0.3 or j3 < 0.5. The daemon sends that refusal with `id: null` and no `stream` flag. `openplay.arm.ArmBridge` passes id-less refusals to `OpenPlay._arm_event("refused")`, which only acted on `edge` or `stream` refusals, so **the phone was never told**. The phone kept streaming at 15 Hz into refusals for seconds at a time; the arm did nothing. Buttons work because `do ready`/`home` are not Cartesian streams. The phone already had a "too low: tap Ready" bar for exactly this refusal (`play.js` `lowBar`); it never fired on the real arm. The mock backend has no such check, which is why mock/sim rehearsals never showed it.

### 08:15 Built
- `openplay/eventlog.py`: `ClockSync` (NTP-style, min-RTT of the last 16 samples, uncertainty rtt/2), `StreamTracker` (holds = phone `ep`; gaps > ttl get a cause `phone` / `network` / `phone-gate` / `unknown`, and a deadman flag when the daemon's `streaming` flag dropped during the gap), `ArmTracer` (every armctl message, ack/done latency, 2 Hz state samples, inferred `stream_stop` causes), `EventLog` (JSONL + 20 000-record ring), `python -m openplay.eventlog summarize <file>`.
- `openplay/server.py`: one `phone/rx` record per phone message with the server's decision (`forward` / `refuse` / `drop` / `ignore` / `start`), connection open/close with close codes, hello/reconnect, sync, phone `diag`, display captions, operator commands, event-loop stalls; `/events` viewer, `/api/events`, `/api/streamstats` on the loopback display listener; `--trace-log` (default `experiments/<today>-*/logs/openplay-events-<ts>.jsonl`, preferring a folder with "openplay" in the name: today that is this folder) and `--stream-ttl-ms` (default 300, unchanged).
- `openplay/static/play.js`: `t` + `seq` on every message, `ep`/`skip`/`why`/`buf` on streams, ping burst + `sync`, buffered `diag` events (hidden/visible, pagehide, pointercancel, ws_close with code, late_tick, rx_silent).
- **Fix:** the server relays the daemon's id-less "stream v: … needs the arm unfolded" refusal to the driver as `refused` (once a second); the phone's lowBar now shows and streaming stops.
- **Fix:** `armctl/daemon.py` deadman log line prints the stream's real ttl (was hard-coded "200+").

### 08:40 Jitter test (mock, in-process)
`.venv/bin/python -m pytest -q -s openplay/tests/test_stream_jitter_e2e.py`: a simulated phone 4.2 s ahead of the server, 15 Hz Tilt setpoints, ttl 300 ms. Hold 1: the phone's timer stalls 600 ms. Hold 2: the phone sends on time, but 6 messages are held 600 ms and delivered in a burst.
```
clock sync (phone - server, min-RTT sample):
  p-jit: offset 4197.5 ms ± 7.7 ms (rtt 15.4 ms, n 5)
     # pid        kind  dur s    n    Hz    arr p50/p95/max   sent p50/p95/max >ttl dead  end
     1 p-jit      v       1.5   16    10         67/230/602         67/230/602    1    1  release (up)
     2 p-jit      v       1.5   20    13         67/129/675         68/102/102    1    1  release (up)
gaps > ttl (worst 12):
  08:40:21.157 hold   2 gap   675.0 ms (phone sent 68 ms apart) -> network  DEADMAN
  08:40:19.141 hold   1 gap   601.5 ms (phone sent 602 ms apart) -> phone  DEADMAN
verdict:
  2 gaps > ttl 300 ms over 2 holds; 2 became deadman stops; causes phone 1, network 1 (mostly phone)
  holds ended by: release 2; releases: release (up) 2
  2 daemon 'deadman' lines are release expiries (the arm was already told 0): harmless
```
- **Observation:** both stalls detected, attributed correctly, matched to the daemon's real deadman stops (its log has 4 "deadman: no setpoint for 300+ ms" lines: 2 mid-hold, 2 release expiries). Offset recovered to 2.5 ms with ±7.7 ms uncertainty.

Tests: `.venv/bin/python -m pytest -q openplay/tests armctl/tests` all green (see the commit).

### Next (on the arm, with the person at the E-STOP)
- **Idea:** run one session with the trace on (default), Tilt and Sticks, and `summarize` it. Expect: relayed "unfolded" refusals instead of silent ones; count real mid-hold deadman gaps and their cause.
- **Decision:** no deadman mitigation is on. `--stream-ttl-ms 450` exists for a test if the trace shows `network` gaps of 300–500 ms; bridging and WebRTC are documented in [openplay/README.md § Stream gaps](../../openplay/README.md#stream-gaps-if-tilt-and-sticks-keep-stopping), not built.
