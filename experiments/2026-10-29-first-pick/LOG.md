# 2026-10-29 first pick (gravity fix, Route A pick, camera pick)

**Experimenters:** Naman (+ Claude)
**Goal:** j4 cool and healthy; test the gravity-scale fix; first real pick of the white paper cup without the camera (ruler-measured positions, "Route A"); then finish the wrist-camera calibration for camera-based picking. Continues [2026-10-28-laptop-systems-check](../2026-10-28-laptop-systems-check/LOG.md).
**Status:** active

## Environment
Lab laptop `pheonix-drone`, Ubuntu 24.04, DimOS 0.0.14, repo `main` @ `9b941a2` (= fork/main). The base is rotated (forward = down the table), `armctl/site_limits.json` j1 soft −0.6..+1.2.

## Log

### 2026-10-29 morning: power-on probe
The arm's power was off overnight (j4 had overheated and gone silent on CAN, 28 Oct 17:40). Naman powered it on; `.venv/bin/python -I tools/probe_motors.py can0`:
```
joint1 0.036  joint2 -0.001  joint3 -0.001  joint4 0.021  joint5 -0.136  joint6 -0.068  gripper -2.899
all 27–28 °C rotor, enabled False, fault None; CAN sent 35 received 70, 0 errors
```
- **Observation:** **j4 answers again** and is cold (28 °C), so last night was its thermal protection, cleared by the power-cycle. j5 at −0.136 (turned by hand overnight), inside its range.

### Gravity-scale: sim plumbing check
`ZENOH_SCOUT_ADDR=224.0.0.124:7450 .venv/bin/python -m armctl.dimos_coordinator --simulation --kp j4=22 --kd j4=2 --gravity-scale j4=0.83`
→ `gains[OVERRIDE j4]: kp=[80,80,80,22,10,10] kd=[5,5,5,2,1.5,1.5]`, `gravity[OVERRIDE j4x0.83]`, `gravity feedforward at ready: j4 2.378->1.974 N·m (adapter mock_whole_body, mock: not applied)`, started. The wiring works; the mock has no gravity, so the real effect needs the arm.

### 08:15–08:21 Gravity-scale on the real arm: j4 hold test (stiffer + gravity 0.83)
Real coordinator `--kp j4=22 --kd j4=2 --gravity-scale j4=0.83`, adapter **`armctl_openyam_damiao`** (first real use; activation fine), startup `gravity feedforward at ready: j4 2.378->1.974 N·m`; the daemon's `armctl_info` confirms `scale j4 0.83, applied: True`. Lab console restarted (read-only) for torque/temperature. `j4_hold_test.py --console http://127.0.0.1:8090 --hold 60 --kp4 22` ([logs/j4-grav083.log](logs/j4-grav083.log), CSV [logs/j4-hold-20261029-081627.csv](logs/j4-hold-20261029-081627.csv)).

| j4 target | model τ4 (stock) | err Tue run 1 (kp 10) | err Tue run 2 (kp 22) | **err today (kp 22 + scale 0.83)** | measured τ4 | rotor °C |
|---|---|---|---|---|---|---|
| 0.0 | 2.378 | +0.0937 | +0.0484 | **+0.0147** | 1.858 | 29→33 |
| −0.3 | 2.560 | +0.1091 | +0.0503 | **+0.0226** | 2.013 | 33→36 |
| +0.3 | 1.984 | +0.0412 | +0.0252 | **−0.0007** | 1.620 | 37→37 |
| +0.7 | 1.191 | +0.0151 | +0.0081 | **−0.0076** | 1.014 | 37→35 |

`home` done, max error **0.0231** (Tue 0.0616 / 0.0319). No refusals or faults.
- **Result:** the worst-case j4 error is ~0.02 rad (≈2–3 mm at the tip), 4–5× better than the default; **j4 peaked at 37 °C** (Tue 62 °C).
- **Insight:** light poses now go slightly negative, matching the agent's affine fit (τ_true ≈ s·τ_model + c; friction/offset). A lower scale would trade heavy-pose error for more light-pose overshoot. **0.83 is a good single value.**
- **Decision:** the real-arm start line is now `--kp j4=22 --kd j4=2 --gravity-scale j4=0.83` (HANDOVER updated).

### Re-plan for today's layout; arm powered off for hand reach-checking; camera exposure
- The combined lens (`interfaces/wristcam/data/combo1`, 44 views) was re-solved into `interfaces/wristcam/calib/wristcam.json` (fx 537.1, fy 532.9, RMS 0.79; the arm-only solve is kept as `wristcam.intr1-only.json`).
- **Hand-eye re-solve of `he1` at 22.9 mm** ([handeye-he1-229.rejected-*.json](.)): RWHE-SHAH+refined, reprojection 4.37 px, spread 6.68 mm / 1.44°, camera t = (−85.8, 13.3, 61.3) mm in the tip frame, **optical axis 44.7° from the fingers**, 10.6 cm from the tip; board scale from kinematics ×0.979 → **square ≈ 22.42 mm**; table z −0.044 (old layout, probably the base board's thickness). Gates: reprojection, spread and table z fail narrowly; axis, camera distance and scale pass. Kept as a planning guess, not used for grasping.
- `handeye plan/collect` gained `--calib-guess` / `--handeye-guess` (plan visibility with the measured camera instead of the nominal). With them, 15 rotation-diverse views around the board (median 37.5°, axis spread 0.55) all see 4/4 corners.
- **Decision (Naman):** "we can print more sheets": tape one checkerboard **permanently**, flat, off to the side of the work area (arm's left, ~30–40 cm out) as the reference for today's hand-eye and future re-checks; the table centre stays free for objects. Reprints must use the same printer settings (this print came out at ~22.4 mm/square, not 30).
- Naman asked to power the arm off to move it by hand and check its reach: daemon + coordinator stopped, `disable_all` (all disabled, no faults; j4 rotor 43 °C after the gravity test). Back near home after (j1 +0.075).
- **Problem:** at home the wrist camera's auto-exposure (`auto_exposure=3`, aperture priority) blew the sunlit table out to white. **Fix:** manual exposure with `v4l2-ctl -d /dev/video0 -c auto_exposure=1 -c exposure_time_absolute=110 -c gain=60` (exposure 60 → mean 53/255, auto → nearly all clipped). Set again after a camera reconnect or reboot; tune per lighting.

### 2026-10-29 (sim only) Tap to pick / say what to pick
Built "pick up THAT": tap the thing (or say "the cup on the table", Claude finds it) on a still wrist-camera frame, and the closed-loop engine homes in on it and picks it up. It's open play's new **Pick it up** mode. Simulation and tests only: nothing ran on the arm. Write-up, results and what it needs from the real arm: [TAP-TO-PICK.md](TAP-TO-PICK.md).
- **Insight:** the silhouette alone can't give an object's height. A prism fit (top face = footprint) can: synthetic tap → box 1.4 mm, cup 0.4 mm.
- **Fix:** the engine refused short objects (approach and lift-clear below the 0.08 m tip guard). Fixed in `engine.py`; the cup tests are unchanged.

### Late morning: tag sheet as a board; three agent builds merged
- **Tag sheet as a calibration board:** Naman taped the 40 mm ArUco page (`vision/targets/aruco_tags_A4.pdf` p1, IDs 0–9) and measured its 50 mm bar at **48 mm → print scale 0.96** (tags 38.36 mm). New board kind `tags` in `interfaces/wristcam/board.py` (`interfaces/wristcam/print/printed_tags_40mm.json`; table origin = centre of the tag grid, 15.4 × 21.1 cm); detection, `board_points_base`, `with_measured` scaling. Wristcam tests pass. (The checkerboard print was ~0.75 scale, so each print gets its own measured scale.)
- **Camera exposure:** manual exposure 110 / gain 60 gives a well-exposed table under the morning sun.
- **Observation (camera, arm folded at home):** the sheet is taped at the **far end of the table on the front-edge side**; PnP with the combined lens + the he1 hand-eye guess puts its centre at (0.66, 0.25, 0.00) m, **r ≈ 0.71 m**, beyond the arm's useful reach (~0.45–0.5 m). Asked Naman to move it to ~35–45 cm from the base.
- **Merged (agents, all tests: 502 passed, 1 xfailed):**
  - **Visual choreography with Fable** (`experiments/2026-10-29-visual-choreo/`): render sheets (audience view first; keyframes + in-betweens; joint/limit and speed-cap plots), 2–3 critique/revise passes with a 5-point spectator rubric, `armctl/choreo_guidelines.md`, camera-snapshot router (only for real-scene prompts), `--visual --passes 3 --model claude-fable-5-1`; open play opt-in `OPENPLAY_SAY_VISUAL=1`. First real run ("a happy wave", 1 review): 71 s, ~$0.53; the review found the library wave "invisible (6 cm)" from the audience. Library audit: small amplitudes, constant tempo, few holds/gripper use.
  - **Time-synced open-play event log** (`experiments/2026-10-29-openplay-event-log/`): per-message phone→server→armctl timeline with clock-offset estimation, `/events` live view, `/api/streamstats`, `python -m openplay.eventlog summarize`. **Root cause of the Tilt/Sticks "disconnects":** the daemon's "needs the arm unfolded" refusals carry no request id and open play dropped them, so the phone kept streaming into refusals (580 in the 28 Oct log, in runs of 3–6 s). Now relayed (≤1/s), so the phone shows "too low: tap Ready". Deadman lines are mostly one per finger lift. Optional `--stream-ttl-ms 450` (not default).
  - **Tap-to-pick / say-to-pick** (`experiments/2026-10-29-first-pick/TAP-TO-PICK.md`): `interfaces/wristcam/target.py` (tap → table xy via ray–plane + GrabCut outline; `ClaudeLocator` for prompts, JSON point/box/candidates; `SeededFinder` tracks the target through the GraspEngine closed loop), open-play "Pick it up" mode with operator approval. Synthetic: tap 0.4–1.4 mm, prompt 0.2 mm, picks the tapped box not the cup. **Needs real calibration** (refuses until `workcell.json calibrated: true` + `handeye.json`), plus `pick_config.json` (look pose, table area), the real gripper opening, the jaw mask, frame freshness, and a segmentation check on the real table.

### 09:15–09:35 Hand-eye run 2 (he2) on the 25 mm tag page: rejected; diagnosis points at the kinematic model
- Naman taped a second page close to the arm: the **25 mm tag page** (vision/targets/aruco_tags_A4.pdf p2, its 50 mm bar also reads 48 mm → 0.96, tags 23.98 mm) → `interfaces/wristcam/print/printed_tags_25mm.json`. The far 40 mm page made duplicate IDs (4, 5, 6, 9 seen twice) → Naman hid it. Board centre ≈ (0.41, −0.02, −0.02), flat (<6°).
- Plan with the measured guesses: 15 views, 4/4 corners each, rotation diversity median 44°, axis spread 0.59. Probe clean (~30 °C), real stack `--kp j4=22 --kd j4=2 --gravity-scale j4=0.83`.
- **Problem:** `collect` aborted at view 7: `move_joints did not reach target: j2 error 0.055 rad > 0.05` (shoulder sag near full reach). **Fix:** `collect` now treats an arrival miss as "use the measured pose" and gained `--start N` to resume; finished all 15 (one short arrival).
- **Solve (gates): REJECTED.** PARK: reprojection **12.2 px**, spread 6.6 mm / 1.26°, camera t = (−74.5, 0.9, 41.3) mm, axis 45° from the fingers (15° from nominal), **table (0.425, −0.020, −0.020) m, tilt 6°** (matches the sheet), **board scale vs kinematics ×1.082** (the ruler says ×1.0). `workcell.json` untouched (gates refused).
- **Diagnosis:** each view's PnP fits ~1 px, but the board origin from FK·T_tip_cam·PnP scatters 1–2 cm per view, with a step between views 0–6 and 7–14 (the latter reach further, larger j2/j3). **Stale frames ruled out** (own pose 16 mm vs previous pose 46 mm). The 28 Oct ChArUco fit went the other way (×0.98). **Insight:** a joint-dependent error → the FK model (i2rt URDF vs Anvil's OpenYAM build: joint zero offsets/link lengths) is the likely cause, not the camera or the sheets.
- **Next:** offline kinematic calibration (joint offsets + hand-eye jointly over he1 + he2, 27 views) by an agent; no arm motion. Arm folded home (j4 41 °C).

### 10:10–10:25 Kinematic corrections merged; hand-eye run 3 (24 wide views)
- Merged the offline kinematic calibration (`experiments/2026-10-29-kin-calib/`): FK model correct (DimOS yam.urdf = armctl kin, 0.000 mm), **joint zero offsets j3 −13.8, j4 −41.0, j5 −23.9 mrad** (j1/j6 unobservable, j2 weak). `armctl/kin_corrections.json` applies to vision/grasp planning only (`kin.fk_vision`, `kin.ik(corr=…)`); commands, envelope and choreography stay nominal. On he1 + he2: reprojection 8.9 → 3.6 px, LOO board error 8.1 → 4.3 mm; he2 gates 6/7 (only reprojection 4.39 px failed), board scale ×0.995 vs the ruler (independent check).
- **he3** (24 views planned with wider j1 −0.48..+0.53, j4 −0.84..+0.60, j5 −0.62..+0.72; 25 mm tag sheet unchanged; 1 short arrival; `interfaces/wristcam/data/he3`): solved with the corrections → HORAUD+refined, **reprojection 5.29 px ✗**, spread 4.66 mm ✓ / **1.62° ✗**, table (0.414, −0.013, −0.029) ✓, axis 44.3° from the fingers ✓, camera 8.8 cm from the tip ✓, **board scale ×0.985 ✓** → 5/7, **workcell still refused** (unchanged). j4 53 °C after the run → folded home.
- **Next:** re-fit the joint offsets with he2 + he3 (39 views, same board pose) + he1 → `experiments/2026-10-29-kin-calib/logs/kincal_he23.log`.
- Side task (Naman): created the ECI event **"Dexterity: play with a robot arm"**, Thu 29 Oct 3–5 pm IST, custom location "Outside Riva: Garden Oasis Banquet Hall", public (EdgeOS returned status "draft").

### 10:20–10:30 Joint offsets refit on 51 views; hand-eye accepted as PROVISIONAL (operator decision)
- Refit (`experiments/2026-10-29-kin-calib/kincal.py` on he1 + he2/he3 merged = 39 views of today's board; `logs/kincal_he23.log`): offsets plateau at ~4 px whatever the model (links/compliance add little) → the remainder is likely the taped paper's flatness, lens edges, joint play. M1b: **j3 −21.8, j4 −34.1, j5 −27.7 mrad** (σ 1.4–1.7) → `armctl/kin_corrections.json` (previous values kept in its provenance).
- **Decision (Naman, "A"):** accept a result that misses only the precision gates, for cup grasps with visual re-centring. New `handeye solve --accept-provisional REASON`: accepted only if the position-critical gates (position spread, table z, mounting prior, camera distance, board scale) pass; records `provisional: true` + the reason + the failed gates in `handeye.json` and `workcell.json`.
- The permission classifier first blocked me running it ("weakening a safety gate"). Naman then explicitly authorised it ("A you have permission to run it").
- **Result (35 usable views):** HORAUD+refined, reprojection 4.84 px (✗, <3), spread **4.24 mm ✓** / 1.39° (✗, <1), table (0.411, −0.014, −0.031) ✓, axis 44.7° from the fingers ✓, camera 8.8 cm ✓, **board scale ×1.001 ± 0.003 ✓** (the arm's geometry and the ruler now agree to 0.1%).
- `interfaces/wristcam/calib/handeye.json` (PROVISIONAL), `interfaces/grasp/workcell.json` `calibrated: true, provisional: true`. **To clear "provisional":** a rigid flat board (foam/cardboard backing) + redo, aiming for <3 px.

### 10:35–10:55 Cup dry runs (camera-down look, nothing moved beyond the look pose)
- Look pose computed from a home-pose photo (pixel ray to rim height with the provisional calibration), camera ~31 cm above the cup, camera axis (−0.05, −0.01, −1.00).
- **Problem 1:** `Workcell.load` rejected the new `provisional` key. **Fix:** `provisional: bool = False` added to `interfaces/grasp/grasp.py:Workcell`.
- **Problem 2 (cup on the white tag sheet):** `look_and_grasp --finder cup` picked the **tape roll** (another circle, at (0.56, +0.16) m, which is correct for the tape roll, so the geometry was right); `--expect` doesn't narrow the dry-run search. The tap path (`target.resolve`) refused every tap on the cup ("bare_table"): **white cup on white paper**.
- Naman moved the cup onto the bare wood (folded home first; j4 had reached 57 °C at the look pose). New look pose q [0.371, 1.497, 1.303, −0.595, 0, 0].
- **Result:** `look_and_grasp --dry-run --finder cup --image cup-look2-local.jpg --q …` → **cup at base (0.397, +0.145, 0.049) m, σ 1.0 mm, rim diameter 76.5 mm**; the rim circle overlays the cup exactly ([cup-dryrun3/look.jpg](cup-dryrun3/look.jpg)). The pick-place playable validates (55 s, no violations, coach ok); closed-loop centring poses at h 0.18 and 0.12 reachable, 0.08 not.
- **Problem 3 (open):** the tap path still refuses on dark wood ("bare_table" at the cup centre): the table-colour check treats the cup's light colour as table. Fix later; the cup finder works.
- **Open question before a real grasp:** the real gripper opening (model 94 mm, i2rt stroke ~71 mm) vs the cup (76.5 mm rim, ~65–70 mm mid-height). Naman to measure the open pad-to-pad gap with the ruler.
