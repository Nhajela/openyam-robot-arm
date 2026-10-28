# Chess with a real set: design (2026-10-28)

Goal: a person sits at a real chess board and plays the arm. The arm sees or is told the human's move, thinks (Stockfish), and moves its own pieces. The laptop screen shows the game for the audience: the board, last-move arrows, an evaluation bar, and the "thinking" moment.

**What exists:**
- `chess_arm.py`: rules (python-chess), the move → pick and place planner (captures to a graveyard, castling, en passant), a reach map, and Stockfish or a greedy fallback.
- `grasp.py`: top-down grasps.
- `vision/`: ArUco tags, camera calibration, tag poses.

**What's missing:**
- knowing the human's move
- a calibrated board
- grasps reliable enough for tall, narrow pieces on a crowded board

## 1. Knowing the human's move: three layers, each a fallback for the next

The game never depends on vision being right. We track the game state from the start position, so the only question each turn is *which legal move did they make?* There are usually 20–40 candidates. That's far easier than recognising pieces.

1. **Tap it in (always available, build first).** The phone (or the laptop) shows the board. The human taps the piece and the destination; only legal moves are allowed. Promotions get a picker. This is 100% reliable, and it's the fallback whenever vision is unsure.
2. **Camera + "I moved" button.** After moving, the human presses a big button: on the phone, on the laptop, or a physical button or chess clock. The camera compares the board before and after. If exactly one legal move fits, the screen says "You played Nf3" and the game continues. If it's ambiguous, it asks with the top 2–3 candidates as buttons.
3. **Fully automatic.** The camera waits until the board is still and no hand is over it (frame difference below a threshold for about 1.5 s), then does the same comparison. Confirmation stays on screen for a few seconds so a wrong guess can be corrected with one tap.

### How the comparison works (no piece recognition)

1. **Register the board.** Put four ArUco tags just outside the board's corners (we already print DICT_4X4 tags in `vision/targets/`). Their centres give a homography from the camera image to a top-down 8×8 board image. This needs no lens calibration for a flat board, though it's better with it (`vision/calibrate_intrinsics.py`). Re-detect the tags every frame, so a bumped board or camera doesn't matter.
2. **Per-square change score.** In the rectified image, compare before and after for each square's central disc, at the height where pieces are (more on that under "Camera placement"). Use colour and edge differences; normalising brightness per frame handles light changes.
3. **Match to legal moves.** For each legal move, the squares it changes are known:
   - normal moves and captures: from and to
   - castling: 4 squares
   - en passant: 3 squares

   Score each move by how well its squares match the change scores, minus changes elsewhere. Accept the best move if it beats the runner-up by a clear margin; otherwise ask.
4. **Self-check after the arm's own moves.** The same comparison verifies that the piece the arm moved actually arrived, and that nothing fell or got knocked over. If something is off: "I dropped the knight on f6, can you put it on f6?", and the game waits.

### Camera placement: can the arm use its own camera?

The OpenYAM has no camera of its own. Options, best first:

| Camera | Pros | Cons |
|---|---|---|
| **Fixed overhead, looking straight down** (USB webcam on a boom or tripod, or a phone on a gooseneck stand, 60–80 cm above the board) | sees all 64 squares at once; tall pieces barely hide neighbours; one calibration | needs a stand; the arm hides part of the board while it moves (only look when the arm is parked at "watch" pose, out of view) |
| Fixed at an angle (laptop webcam or a phone at the side) | easy to set up | tall pieces hide the squares behind them; back ranks foreshorten. OK for the "I moved" mode with a confirmation step, poor for fully automatic |
| **Wrist camera on the gripper** (small USB cam, ~2 cm) | the arm looks wherever it needs to; close-ups make grasps precise ("visual servoing" for the last centimetre) | the arm must move over the board to look (slow, and it's the human's space); new cabling; a hand-eye calibration |

**Recommendation:** an overhead phone or webcam for move detection (phonecam already streams phones over Wi-Fi, so `vision/sources.py phone:board` works today). Later, possibly a wrist camera for grasp precision only. The arm parks at a **"watch" pose** to the side, clear of the camera's view of the board, whenever we look.

## 2. The board and the arm agreeing on where squares are

- **Board size vs reach.** Top-down reach (`armctl.kin`) is 0.10–0.52 m from the base, up to 10 cm above the table. A **4–4.5 cm square** board (32–36 cm wide), placed with its near edge about 13 cm from the base, fits with margin. Tournament boards (5–5.7 cm squares) don't fit top-down at the far corners.
- **Calibration.** Touch the gripper tip to the four board corners (jog there with the phone, press "save"). `fit_table_frame` gives the arm ↔ board transform. Four corners rather than three let us also check flatness and fit out the sag.
- **Accuracy budget.** We need about 1 cm at the fingertip. Repeatability is excellent (0.001–0.002 rad, about 1 mm). Absolute accuracy suffers from sag without gravity compensation: about 0.03 rad on j2/j3, roughly 1–1.5 cm at full reach. Corner calibration absorbs most of it. If that isn't enough, use a per-square correction table measured once (touch 9 points), or a wrist camera.

## 3. Grasping chess pieces

- The gripper opens to 9.4 cm, and Staunton pieces are 2–3 cm across at the base, narrower at the neck. **Grasp at the neck** (about 60% of the height, `grasp_frac`). There the piece is narrow and the fingers stay above the neighbouring pieces' wide bases.
- **Finger clearance.** With 4.5 cm squares and pieces 2.8 cm across at the base, the open fingers (piece width + 2 cm) are about 1 cm from the neighbours. Close the fingers along the file or rank that has the most room (choose per move from the board state), and open only as wide as needed.
- **Knights** are asymmetric: grasp across the narrow axis (along the knight's facing, which we don't know). Grasping across the neck at any angle works for most sets; test with our set.
- **Placement:** release a few mm above the board, so a small z error doesn't press the piece down or knock it over.
- **Grip check:** the gripper motor should stall on the piece (`holding()`). If it closes fully, we missed: retry once, then ask for help.
- **Speed:** about 40 s per pick and place at today's caps. Playing faster needs higher caps between the pick and place poses (not near the board) and skipping the "ready" stop between the two halves of a capture. Target: about 20 s per move.

## 4. The experience (laptop always on screen)

- **Chess mode on the display:**
  - a big virtual board mirroring the real one, with last-move arrows
  - the human's clock and the arm's "thinking" moment, with an evaluation bar (from Stockfish)
  - commentary captions ("The arm takes your knight!")
  - an optional Claude-written one-line quip per move
- **Phone:** the board for tap-to-move, an "I moved" button, resign or draw, a difficulty setting (Stockfish skill level 0–20, or a time limit per move).
- **Operator:** pause, undo last move, "the board is wrong, set position" (FEN editor), re-calibrate.

## 5. Build order

Updated 2026-10-28: the primary flow is **layer 2** (real board + chess clock + cameras); tap-to-move is the fallback, not the main UI.

1. **Chess clock in open play (built, 🧪 mock/sim only):**
   - phone: a chess-clock screen (two clock faces, one giant "I've moved" button, haptics), candidate buttons when the camera is unsure, the tap board as the fallback (`openplay/static/chess.js`)
   - laptop display: board panel mirroring the real board, last move, both clocks, eval bar (material), camera strip, uncalibrated warning; operator clock key `c`, Board fixed, End game (`openplay/static/display_chess.js`)
   - server: the game, the clock press → `BoardSensor` snapshots → `rank_moves` / `decide`, the engine's moves on the arm through the usual approval path (`openplay/chess_play.py`, `openplay/server.py`; contract in `openplay/PROTOCOL.md` "Chess")
   - the sensor contract and move matching: `interfaces/chess_vision/sensor.py` (`Snapshot`, `BoardSensor`, `rank_moves`, `decide`, `FakeBoardSensor`)
   - chess is refused on a real arm until `workcell.json` has `calibrated: true`
2. **Board cameras** (`interfaces/chess_vision/camera.py`, `CameraBoardSensor`, being built separately against the same contract): tag registration, rectification, per-square occupancy and colour, JPEG frames for the display. Develop against synthetic renders first, then the real board. Run open play with `--chess-sensor camera`.
3. **On the arm:** calibrate the board corners (`fit_table_frame`), measure the gripper, then the arm's first real chess move with a foam block standing in for a piece.
4. **Self-check after the arm's moves, and recovery prompts** (the same comparison: did the piece arrive?).
5. **Fully automatic detection** (still-board trigger, hand detection, maybe MediaPipe hands): the clock press becomes optional.
6. Optional: a wrist camera for the last centimetre of each grasp.
