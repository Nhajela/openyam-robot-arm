# interfaces/chess_vision: board vision v1

This is the "camera + I moved" layer from [CHESS.md](../grasp/CHESS.md) §1. Cameras around a real board report, for every square, P(occupied) and P(white piece | occupied). The game keeps the position and matches the before → after change to a legal move (`sensor.rank_moves`, which the game implements). Nothing here recognises piece types, and nothing here talks to the arm.

```python
from interfaces.chess_vision.camera import CameraBoardSensor
sensor = CameraBoardSensor(["top=phone:top", "side=phone:side"], "interfaces/chess_vision/calib")
before = sensor.snapshot()     # Snapshot(t, occupancy{a1..h8}, colour{a1..h8}, frames{camera: JPEG})
# ... the human moves and presses "I moved" ...
after = sensor.snapshot()
```

| File | What it does |
|---|---|
| `sensor.py` | The shared interface: `Snapshot`, `BoardSensor`, and the `rank_moves` stub (the game implements it). |
| `board_geom.py` | Finds the board from 4 ArUco corner markers, solves the homography to a top-down 8×8 warp, refines the grid on the empty board, and reuses the last homography when a marker is hidden. |
| `occupancy.py` | Per-square features against the empty-board reference (with lighting normalisation and oblique-view sampling), plus a logistic regression per camera that gives calibrated probabilities. |
| `camera.py` | `CameraBoardSensor` (multi-camera fusion), `PosedFrame`, and calibration save/load. |
| `calibrate.py`, `watch.py` | CLIs (see below). |
| `make_markers.py`, `markers/board_markers_A4.pdf` | The printable corner markers. |
| `synth.py` | Synthetic renderer (board, markers, shaded pieces, any camera pose) for the tests. |

## 1. Print and place the corner markers

1. Print **`markers/board_markers_A4.pdf`** at 100% / "Actual size" on matte paper. The 50 mm ruler must measure 50.0 mm. The markers are DICT_4X4_50 **IDs 10-13**, 40 mm (39.96 mm as printed). IDs 0-9 are already used by `vision/targets` (table, gripper, objects).
2. Cut the markers out, keeping the white margin. Tape each one **diagonally outside its corner** of the playing area. Keep its edges parallel to the board edges and its inner corner about 5 mm from the squares, on the board's frame or on the table:

   ```
   ID 13 (a8)                  ID 12 (h8)        black's side
        +------------------------+
        |  8x8 playing area      |
        +------------------------+
   ID 10 (a1)                  ID 11 (h1)        white's side
   ```

   The marker IDs fix which corner is a1, so the board can sit any way round relative to the cameras. The rotation of each printed marker doesn't matter.
3. Measure the square size and the gap (from the marker's inner corner to the corner of the squares, along an edge), and pass them to `calibrate` (`--square-mm`, `--gap-mm`, `--marker-mm`). A few mm of error is fine: calibration finds the real 7×7 inner corners on the empty board and corrects for it (`refine_grid`). Once calibrated, **the markers must not move relative to the board**, so tape them down.

## 2. Camera placement

- **Overhead is best**: a phone or webcam 60-80 cm above the board, looking roughly straight down. It sees every square with little lean. On its own it got 100% occupancy and colour on the synthetic set.
- **Side cameras are OK as extras**: a phone on a stand at about 50° elevation, behind white or to one side. Tall pieces lean over the squares behind them, so a side camera alone is about 90% right. Fusion weights each camera per square by how oblique its view is, so adding side views never hurts the overhead result, and they cover for it when a hand or the arm blocks part of its view.
- Every camera must see **all 4 markers** at calibration. During play, one hidden marker (a hand) is fine. With 1-2 markers hidden, the last homography is reused if it is younger than `staleness_s` (30 s) and the visible markers still agree with it (camera not bumped).
- The arm parks at a "watch" pose out of the cameras' view whenever we take a snapshot.
- Phones stream through `dashboard/phonecam.py` (`phone:<name>`). USB cameras use `/dev/videoN`; when the lab console holds the camera, use `dashboard` (see `vision/README.md`). Lens calibration is optional (pass `--intrinsics name=vision/calib/<cam>.yaml`). It matters for wide-angle lenses.

## 3. Calibrate

```
.venv/bin/python -m interfaces.chess_vision.calibrate --source top=phone:top --source side=phone:side \
    --out interfaces/chess_vision/calib --square-mm 35 --marker-mm 40 --gap-mm 5
```

1. It asks for the **empty board**: markers only, with no hands or arm in view.
2. Then it asks for the **start position**, with white on the ID 10/11 side.

For each camera it prints the marker reprojection error, whether the grid was refined (and by how much), the estimated camera position, and the fit on the start position (it should be 100%). It writes `chess_vision.json`, the reference images and a `<camera>_check.jpg` per camera. The camera must not move between the two steps. Recalibrate if a camera, the board or the markers move, or if the lighting changes completely (normal light changes are handled).

Offline: each `--source` can be a folder whose sorted images are the empty board, then the start position (`--yes` skips the prompts). `python -m interfaces.chess_vision.synth --out DIR` writes such folders from the renderer.

## 4. Watch (debug view, no GUI)

```
.venv/bin/python -m interfaces.chess_vision.watch --source top=phone:top --calib interfaces/chess_vision/calib --out /tmp/chess_watch
```

Each snapshot writes `latest_<cam>.jpg` (markers, the projected grid, and a dot per occupied square, in black or white), `latest_<cam>_warp.jpg` (the warp with the sampled discs and probabilities) and `latest_board.png` (the fused board). It also prints an ASCII board and the squares that changed since the last snapshot.

## How it works

1. **Localise.** Detect the markers and solve the homography from all 16 marker corners. Each marker's rotation is solved for and cached. The empty-board grid correction is then applied. The camera position comes from `solvePnP` on the markers, with K guessed as f = 0.9·width unless intrinsics are given.
2. **Warp** to a canonical top-down image, 64 px per square, with a1 at the bottom left.
3. **Normalise lighting.** Fit a per-channel gain field (linear across the board) from patches at the 81 square corners, which pieces rarely cover. The fit uses trimmed least squares on log ratios against the empty reference.
4. **Sample each square** in a disc that is shifted *away from the camera*, by lean × 0.6 squares, capped at ¼ square. Lean is the horizontal distance divided by the camera height. This samples the piece's own body rather than the piece in front of it, which leans into the near half.
5. **Features against the empty square:** colour difference, chromaticity difference, and the change in edge density on log-luminance (the last two are brightness-invariant). A per-camera logistic regression, fitted on the start position (32 occupied, 32 empty), turns them into a probability. For **colour**, the bright upper percentile of the changed pixels plus the fraction brighter than the empty square, fitted on the 16 white and 16 black pieces.
6. **Fuse:** a weighted mean of logits. The weight is marker quality (count, reprojection, staleness) × view weight (1/(1+(lean/0.6)²)) × visibility. Colour is also weighted by P(occupied). A square no camera sees gets 0.5.

## Synthetic results (`interfaces/tests/test_chess_vision.py`)

The test scenes have a 35 mm board, markers placed 2-3 mm off and rotated, and three cameras: overhead at 72 cm, side at about 47° behind white, and left at about 50°. Pieces are two-tier shaded cylinders up to 2.15 squares tall. There are 10 random game positions, each with a random lighting change (gain 0.65-1.25, ±30% gradient, ±10% tint), plus blur, noise and JPEG.

| | occupancy | colour |
|---|---|---|
| fused top + side + left | 100% | 100% |
| top alone | 100% | 100% |
| side alone / left alone | 89.5% / 89.5% | 97.4% / 100% |
| fused side + left (no overhead) | 94.1% | 99.0% |

- **Localisation:** 1.3 mm mean, 3.1 mm max error from the nominal marker geometry, and **0.17 mm mean, 0.36 mm max** after the empty-board grid refinement.
- **End to end:** the before → after changes are exactly the right squares for e2e4, exd5 (d5 flips colour), O-O (e1 f1 g1 h1) and en passant (e5 f5 f6), with all three cameras fused and with the overhead camera alone.
- **Robustness:**
  - A hand over a marker reuses the homography.
  - A lens-covered camera is ignored.
  - A frame with a known pose and no visible markers still works (`PosedFrame`).

## Known limits

- **Synthetic only so far.** Real pieces have carved shapes, gloss and shadows, and real boards have wood grain. The method is relative to each square's own empty appearance, which should carry over, but this is untested on the real board.
- **Colour assumes light vs dark pieces.** Two similarly coloured sets would need a chroma feature.
- **Oblique cameras alone are not enough** for fully automatic play: tall pieces hide the base of the piece behind them. Use an overhead view.
- **The snapshot must be taken with the board clear:** no hands, and the arm parked. Hand and arm detection is build step 4.
- **Promotion:** the colour flips but the piece type is unknown. The game asks which piece.
- **Calibration has no camera-to-arm transform.** The board → arm transform comes from the corner-touch calibration (CHESS.md §2). The board frame here (origin at a1's outer corner, x along the files, y along the ranks) matches `chess_arm.BoardGeometry`'s file/rank convention.

## Arm scan mode (planned, not implemented)

The arm has its own camera: the **USB CAMERA 4K** (Sunplus 1bcf:2d4f, `/dev/video0`, up to 3840×2160 MJPG). It's a wide-angle module **between the gripper fingers**, and both jaws are visible in its frame. The lab console holds the device and serves it as `dashboard` (`http://127.0.0.1:8090/camera.jpg`). With it, the arm could scan the board itself, and no extra camera would be needed. The seams are already in place:

- **`PosedFrame(image, cam_pos_mm, H_img2board)`**, which a source's `read()` may return:
  - The pose is **FK(gripper tip) × the hand-eye transform** (camera relative to the tip, not yet calibrated; see `vision/README.md` "Next steps", step 3), expressed in the board frame through the board ↔ arm corner calibration.
  - `cam_pos_mm` replaces the marker-PnP estimate for oblique sampling and weighting.
  - `H_img2board` (from the pose and intrinsics) is used when the corner markers are out of view. At close range a scan pose may see only part of the board.
- **(a) `ArmScanBoardSensor` (planned).** It plays a scan playable through armctl, which applies the soft limits and needs the person at the arm to approve. The playable visits 3-4 top-down poses over the board. At each pose it grabs one frame from `dashboard`, wraps it in a `PosedFrame`, and fuses them with the same per-square logit fusion. Each scan pose is calibrated as its own named "camera", with its own empty-board reference, under a name like `scan1..scan4`.
  - Wide-angle lens: calibrate the intrinsics and distortion first with `vision/calibrate_intrinsics.py`, and pass `--intrinsics`. Frames are undistorted before localisation.
  - The jaws cover the lower image corners. Add a fixed jaw mask so those pixels count as not valid, through the same `valid` mask that handles out-of-view pixels.
  - Open the gripper fully for scans, so the jaws block less.
- **(b) Optional Claude vision reader (planned).** Given the scan frames (or the fixed cameras' frames) and the legal-move list, it returns the most likely move, constrained to those legal moves. This is a second opinion next to `rank_moves` on the occupancy diff. When the two disagree, the game asks the human with both candidates as buttons. Nothing in this package calls an API.
