# 2026-10-29 kinematic calibration from the wrist camera

**Experimenters:** Naman (+ Claude agent)
**Goal:** the hand-eye solve on `he2` keeps failing its gates (12.2 px, board-pose spread 6.6 mm / 1.26°, board scale ×1.082 vs the ruler's ×1.0), and the per-view board origins scatter 1–2 cm while each view's PnP alone fits to ~1 px. That points at the arm's kinematic model, not the camera. Fit joint zero offsets (and optionally link lengths and joint compliance) together with the camera mount and the board poses, check the FK model against the other YAM URDFs, and re-run the gates. Continues [2026-10-29-first-pick](../2026-10-29-first-pick/LOG.md).
**Status:** offline analysis done. The correction is written for vision and grasp use only. Real-arm checks are still to do.

## Environment
- Offline only: no arm, no DimOS, no camera. Lab laptop `pheonix-drone`, Ubuntu 24.04, Python 3.12.3, numpy 2.5.4, scipy 1.18.1, OpenCV 4.14.0 (main `.venv`, read-only). Worktree branch from `main` @ `993b7b8`.
- Data (git-ignored, read from the main checkout):
  - `interfaces/wristcam/data/he1`: 12 views, 28 Oct, 7×5 ChArUco. Square size unknown, so its scale is a free parameter.
  - `interfaces/wristcam/data/he2`: 15 views, 29 Oct, 25 mm tag page `print/printed_tags_25mm.json`, print scale 0.96 from the ruler.
- Lens: `calib/wristcam.json` (fisheye, RMS 0.79 px).

## Log

### 09:00 What is observable (Insight)
- A **j1** offset turns the whole arm about base z, so a free board pose absorbs it. A **j6** offset turns about the last axis, so `T_tip_cam` absorbs it. Both are fixed at 0. Neither one matters for grasping in the calibrated table frame.
- A **j2** offset turns everything distal about the j2 axis. That axis only moves with j1, so with a free board pose d2 is observable only through the j1 spread of the views. In the fit, d2 correlates −0.95 with the he2 board's z.
- j2, j3 and j4 are parallel. Changing a link's *direction* in their plane equals a combination of offsets, so only link **lengths** are identifiable. The first attempt used xyz deltas on joint4's origin and the problem went singular (cond 4·10⁸). It was reparametrised as lengths `Lu` (j2→j3) and `Lf` (j3→j4).

### 09:15 Joint fit: model comparison
`kincal.py` minimises the reprojection of all 835 detected corners in both datasets (soft-L1 loss). Free: kinematic parameters, `T_tip_cam`, one board pose per dataset, and he1's scale (he2's scale is fixed by the ruler).

```
.venv/bin/python -I experiments/2026-10-29-kin-calib/kincal.py --data /home/naman/openyam-robot-arm/interfaces/wristcam/data --loo
.venv/bin/python -I experiments/2026-10-29-kin-calib/kincal.py --data ... --loo --models M0,M1,M1a,M1b --tag _v2
```

Raw output: [logs/kincal_loo.log](logs/kincal_loo.log) and [logs/kincal_loo_v2.log](logs/kincal_loo_v2.log) (the v2 run fixed the cross-dataset re-seeding). Numbers: [logs/kincal_results*.json](logs/).

LOO means leave-one-view-out. The "LOO board origin" column is the error of the held-out view's predicted board origin. Offsets are in mrad.

| model | params | rms px (he1 / he2) | spread he2 mm / ° | LOO px rms | LOO board origin rms mm | fitted |
|---|---|---|---|---|---|---|
| M0 hand-eye only | 19 | 8.86 (7.4 / 9.5) | 7.2 / 1.93 | 10.19 | 8.1 | – |
| M1 + d2..d5 | 23 | 3.39 (3.4 / 3.4) | 3.0 / 0.70 | 4.45 | 3.9 | d2 +41±6, d3 +1±2.5, d4 −41.0±1.5, d5 −24.2±1.6 |
| M1a + d4, d5 | 21 | 3.76 | 3.8 / 0.97 | 4.75 | 4.4 | d4 −54.0±1.1, d5 −26.6±1.8 |
| **M1b + d3, d4, d5** | 22 | 3.57 (3.1 / 3.8) | 3.7 / 0.85 | **4.43** | 4.3 | **d3 −13.8±1.4, d4 −41.0±1.6, d5 −23.9±1.6** |
| M2 + d2..d5 + lengths | 25 | 3.37 | 3.0 / 0.70 | 4.68 | 4.1 | Lu +0.8±0.8 mm, Lf −1.6±1.0 mm |
| M3 + d2..d5 + compliance j2..j4 | 26 | 3.25 | 3.2 / 0.79 | 4.48 | 3.8 | c3 +5.9±0.8, c4 −14±2 mrad/N·m. d2/d3 shift by 20 mrad |
| M4 all | 28 | 3.23 | 3.0 / 0.84 | 4.65 | 3.9 | d4 −52±3 |
| M5 d2..d5 per dataset | 27 | 3.01 | 2.6 / 1.10 | 4.21 | 4.7 | d5 −23.9 (he1) / −24.2 (he2). d4 −33±7 / −55±2. d2 −46 / +136 |

Cross-dataset test: fit the kinematics and hand-eye on one set, then fit only the other set's board pose.

| | M0 | M1b |
|---|---|---|
| he2 → he1 | 8.5 px | 3.7 px |
| he1 → he2 | 17.6 px | 10.5 px |

- **Result:** joint offsets on j4 and j5 explain most of the error. Reprojection drops from 8.9 to 3.4–3.6 px, LOO from 10.2 to 4.4 px, and the board-origin prediction from 8.1 to about 4 mm.
- **Insight (well determined):**
  - **d5 ≈ −24 mrad (−1.4°)** comes out the same in every model and in both datasets separately.
  - **d4 ≈ −41 mrad (−2.3°)**. The formal σ is 1.6 mrad, but it moves between −33 and −55 mrad with the model and the dataset, so it is really about ±10 mrad.
- **Insight (poorly determined):** d2 (j1 spread too small) and d3 (it trades against d2) are poorly determined. Link lengths come out below 2σ, about 1 mm, so the FK link lengths are fine. Compliance improves the fit slightly but not the LOO error, so it is not adopted. he1 alone (j4 span only 0.2 rad, j2/j3 narrow) constrains little: he1 → he2 transfers poorly.
- **Observation:** with the corrected model he1's square comes out at **20.3 mm** (×0.886 of 22.9). The earlier 22.4 mm was fitted with the wrong FK. Measure it with a ruler before using he1 again.
- **Decision:** adopt **M1b** (d3, d4, d5). It has the best LOO reprojection, no ill-determined d2, and condition number 47.

### 09:30 FK model check (agent, read-only)
- DimOS's planning model `~/.local/share/dimos/repo/data/yam_description/i2rt/yam.urdf` (with `tip_link = gripper_tip`, 0.1 m) is **identical** to `armctl/kin.py`: 0.000 mm over 200 random q.
- `yam_gripper_gravity.urdf` uses different frames but no sign flips. Its link-6 + 0.10 m point matches kin's tip to 0.3 mm RMS. Its own TCP sits 3 cm further out (0.13 m). Do not mix the two up.
- i2rt `yam.urdf` links match to 4·10⁻⁷ m. Its joint6 frame is rotated with axis +z, which changes the gripper orientation and j6 sign but not the wrist position.
- I could not open Anvil's OpenYAM repo, so I cannot say whether OpenYAM's link lengths differ. The fit says they match to about 1 mm.
- **Insight:** the URDF is not the problem. The joint **zeros** are.
- i2rt's zeroing is per motor: `i2rt/motor_config_tool/set_zero.py`. It writes the motor's zero over `can0`. **Do not run it** without a reviewed procedure and the person at the arm.
- A zero set while the arm sat a degree or two off its stops would give exactly this kind of error.

### 09:40 Correction as data, wired into vision only
- `armctl/kin_corrections.json`: `joint_offsets = [0, 0, −0.01376, −0.04102, −0.02394, 0]` rad (q_model = q_dimos + offset), with provenance, formal σ and the systematic note. It is `"enabled": true`. Set `false`, or the environment variable `ARMCTL_KIN_CORRECTIONS=off`, to disable it.
- `armctl/kin.py`:
  - New `KinCorrection` (joint offsets, origin deltas, tip delta, compliance) and `load_correction` / `vision_correction` / `fk_vision`.
  - `fk`, `frames`, `tip`, `approach`, `jacobian`, `clearance` and `ik` accept `corr=`. **The default is None, which is the nominal model**, so DimOS commands, the envelope, choreography and table-clearance safety are unchanged.
  - `ik(corr=...)` returns **DimOS joint targets**: the correction sits inside FK. Its table check uses the lower of the nominal and corrected clearances.
- Switched to `fk_vision` / `ik(corr=vision_correction())`:
  - `interfaces/wristcam/{camera,engine,handeye,locate,look_and_grasp,synth}.py`
  - `interfaces/grasp/grasp.py`
  - The hand-eye result records the correction it used (`extra.kin_correction`).
- **Consequence (Decision):**
  - If the offsets are real, the arm's **commanded** poses are also off. DimOS's own Cartesian IK and twist control, and every armctl pose computed with the nominal FK, put the gripper 1–2 cm from where the model says.
  - Vision and grasp are now self-consistent: they see with the corrected FK and command through corrected IK in DimOS joint space.
  - The table-clearance guards still use the nominal model. Keep their margins (≥ 1.5 cm) until this is fixed at the source.
  - The right fix is at the source: re-zero j4/j5 (motor zero) or add an offset in the adapter. Do not edit `.venv`. That changes every q, so do it only after the physical check below.
- Tests: `armctl/tests/test_kin_corrections.py`, synthetic. Known d2..d5 are injected and recovered within 4σ, and a set with no offsets is recovered as near 0. Also covered: JSON round-trip, `enabled: false`, and IK with a correction returning DimOS q. Result: 6 passed.
- **Problem:** with the correction file enabled, 13 synthetic `interfaces/tests` failed, because their worlds render with the nominal `kin.fk`. **Fix:** the root `conftest.py` sets `ARMCTL_KIN_CORRECTIONS=off`, the same pattern as `ARMCTL_SITE_LIMITS`. After that: `interfaces/tests` 128 passed, 1 xfailed; `armctl/tests` 242 passed.

### 09:45 Pipeline gates with the corrected FK
```
.venv/bin/python -I experiments/2026-10-29-kin-calib/gates.py --data /home/naman/openyam-robot-arm/interfaces/wristcam/data \
    --corr armctl/kin_corrections.json --he1-square 20.29 --out experiments/2026-10-29-kin-calib/handeye_he2_kincorr.json
```
[logs/gates.log](logs/gates.log)

| | reproj | spread mm | spread ° | table z | axis | cam dist | board scale |
|---|---|---|---|---|---|---|---|
| he2 nominal | 12.16 ✗ | 6.59 ✗ | 1.26 ✗ | −0.020 ✓ | 15.1° ✓ | 8.5 cm ✓ | ×1.082 ✗ |
| **he2 corrected** | **4.39 ✗** | **3.31 ✓** | **0.67 ✓** | −0.031 ✓ | ✓ | 8.8 cm ✓ | **×0.995 ✓** |
| he1 nominal (20.29 mm) | 5.63 ✗ | 5.92 ✓ | 1.44 ✗ | −0.031 ✗ | ✓ | ✓ | ×1.105 ✗ |
| he1 corrected (20.29 mm) | 3.16 ✗ | 3.87 ✓ | 1.00 ✗ | −0.051 ✗ | ✓ | ✓ | ×1.011 (circular) |

- **Result:** he2 passes 6 of 7 gates. Only reprojection fails: 4.39 px from the pipeline, which keeps the closed-form rotation, against 3.8 px from the joint fit.
- **Insight:** the **board scale moved from ×1.082 to ×0.995**, so the ruler and the kinematics now agree. That is independent evidence for the offsets, since scale was not part of the objective.
- Camera on the tip: t ≈ (−80, 3, 38) mm, optical axis 45° from the fingers.
- The hand-eye result is written to [handeye_he2_kincorr.json](handeye_he2_kincorr.json) (this folder; **not** the main calib, not the workcell).
- **Problem:** a 3–4 px floor remains. Candidates are the poorly pinned d2, the j4 offset varying between sessions (he1 −33 vs he2 −55 mrad: a power-cycle/zero issue?), link flex or backlash, and the lens edge.

### Ideas / next
- **Physical check first (Idea):** with the arm folded on its stops (motors off) read `probe_motors.py`. Then hold the j4 and j5 links against a level or square in a known pose. Is j4 really 2.3° off and j5 1.4° off? Do the values change after a power-cycle?
- **More views (Idea):** take one new hand-eye set of 20–25 views with:
  - j1 spread of at least ±0.6 rad (board placed so the arm can see it from left and right), to pin d2;
  - j4 over its range (−1..+1 rad) and j5 ±0.6 rad;
  - j2/j3 both near (folded) and far (reaching) poses;
  - the board at a second position on the table, or two boards;
  - everything in one power-on session.
  Then re-fit M1 / M5 and compare with today's offsets.
- Next session: also log `q_planned − q_measured` and the motor temperatures per view, to test sag and compliance directly.
