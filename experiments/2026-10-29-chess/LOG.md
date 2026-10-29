# Chess with the arm (2026-10-29)

Goal (Naman): pick chess pieces up and place them on other squares, pinching from the top so neighbours don't topple; check each move with the camera; start with pieces within reach, then tough positions.

### 16:15 Board seen from home (nothing moved)
- Roll-up vinyl board in front of the arm, slightly rotated: squares ≈ 5 cm, from x ≈ 0.21 m (behind the jaws) to the far edge at x ≈ 0.59 m (beyond reach); pieces on the extremes ([home-board.jpg](home-board.jpg); top-down with a 5 cm base grid: [home-board-ortho.jpg](home-board-ortho.jpg) from [ortho_grid.py](ortho_grid.py)).
- **YOLO-E on CPU** (the NVIDIA driver isn't loaded: `nvidia-smi` fails; 535 installed, Ubuntu recommends 595 for kernel 7.0): `pick_anything --image home-board.jpg --q=… --prompt "chess piece,pawn,rook,bishop"` → 3 "chess piece" (0.24, 0.21, 0.15), 668 ms; nearest pawn at table (−0.057, −0.091) ([detect-home/look.jpg](detect-home/look.jpg)). First run also downloaded the weights.
- 16:23 Naman: "give me back the phone control" → real stack + open play (with the API key) up; j4 41 °C at start.

### 16:35 First piece move: plan (nothing moved)
- Re-detect ([detect-home/result.txt](detect-home/result.txt)): YOLO-E's best "chess piece" box (0.21) **merges the near pawn with the taller piece behind it** (same file, next square; size 55 × 21 mm); the pawn alone is the 0.15 box: table (−0.061, −0.108), 23 mm, height unseen from above. The far-row pawn (0.24) is refused "too small" at that distance.
- Squares ≈ **4.5 cm** (home-frame columns ~82 px at the pawn's row); pawn base (0.293, +0.034) base; next square toward the arm's left (0.291, +0.079) base = table (−0.107, −0.108), empty.
- Plan: pinch the head from the top (`--grasp-frac` from the measured height), open only ~4 cm (`--open-margin 0.02`) so the fingers clear the piece behind (its base edge ~3.3 cm from the pawn centre), fingers closing across the file (`--finger-yaw 1.571`, base y), `--no-grip-check`, `--min-tip-z 0.03`, then a camera check of both squares.
- Code: `pick_anything` gained `--grasp-frac --height --open-margin --min-tip-z --realign-max --no-grip-check --finger-yaw` and goes to ready first when folded; `target.pick_config` lets the caller override `finger_yaw`. Tests: 28 passed (+ the known pre-existing tracker failure).
- Waiting for Naman's ruler: pawn height and head diameter.

### 16:40 pawn1 aborted before motion (heat) + target fix
- Naman: "you don't need any of it, just figure": height guessed 5 cm (squares 4.5 cm → ~5 cm pawns), `--grasp-frac 0.8` (fingertips at ~4 cm: head/neck).
- `pick_anything --execute --source dashboard --prompt 'chess piece' --height 0.05 --grasp-frac 0.8 --open-margin 0.02 --min-tip-z 0.03 --realign-max 0.012 --no-grip-check --finger-yaw 1.5708 --place=-0.107,-0.108` ([pawn1.log](pawn1.log)): at the first prompt (MOVE to ready) **j4 rotor 59 °C at home** (after the open-play session) → declined, stack stopped, `disable_all` (no faults, j4 57).
- **Problem:** its look picked the merged pawn + piece-behind box again (table (−0.051, −0.079), 63 × 23 mm), between the two pieces. **Fix:** `resolve_prompt(max_size=…)` / `pick_anything --max-size`: a footprint longer than that is refused as "probably two together". On the home frame with `--max-size 0.035` → target = the pawn alone (table (−0.061, −0.108), 23 mm).

### 16:47–16:50 pawn2: centring oscillated (no grasp); detector blind top-down; motors off
- `pick_anything … --max-size 0.035 …` (+ guard, [pawn2-guard.log](pawn2-guard.log)): the look chose the **taller piece behind the pawn** (table (−0.050, −0.065), 21 mm). Ready → search → centring flipped between two estimates ~2.7 cm apart for 7 looks → "no convergence after 7 centring iterations" ([pawn2/run.jsonl](pawn2/run.jsonl)). j4 49 → 51 °C.
- **Insight:** the observations alternate between `detector` (the piece's base, table y ≈ −0.056) and `template (detector missed)` (y ≈ −0.029): in the camera-down frames ([obs_004](pawn2/frames/obs_004_center.jpg), [obs_005](pawn2/frames/obs_005_center.jpg)) the template mark lands on the **empty square behind the piece** (the home-frame template matches the board pattern), and **YOLO-E doesn't see chess pieces from straight above** (no detection on the centred frames even at conf 0.02; it finds them only obliquely).
- **Decision:** for chess, localise the **board**, not the pieces: one camera-down look → checkerboard grid → square centres in the table frame → occupancy per square (piece vs bare square) → pinch at the square centre, place at the target square centre → re-look and check the occupancy changed. Pieces in "tough positions" (crowded) then only need the neighbours' squares, which the grid gives.
- Folded home, stack stopped, `disable_all` (no faults; j4 54 °C).

### 16:55 "Once we know where the piece is, just go for it" (Naman)
- **Built:** `locate.FixedFinder` (always reports the object at a known table xy and height) and `look_and_grasp --finder fixed` (`--expect` is the measured position) + `--finger-yaw`; workcell object `chess_tall` (height guessed 7 cm, 22 mm). The engine's centring/descent/checks then run open-loop on the known position.
- Tall piece: base footprint from three detector looks in pawn2, table (−0.0512, −0.0559) (±1 mm). Target: one square to the arm's left, table (−0.0962, −0.0559) (empty). Fingertips at 0.72 × 7 cm ≈ 5 cm, opening 4.2 cm, closing along base y (across the file, clear of the pawn in front).
- Dry run ([fixed-dry](fixed-dry)): open-loop pick-place valid, 38.8 s, coach ok.

### 16:51–16:55 tall1: stuck in a loop (Naman: "it feels like you're stuck in a loop") → fix
- `look_and_grasp --execute --object chess_tall --finder fixed --expect=-0.0512,-0.0559 --place=-0.0962,-0.0559 --grasp-frac 0.72 --open-margin 0.02 --min-tip-z 0.03 --realign-max 0.012 --no-grip-check --finger-yaw 1.5708` ([tall1-guard.log](tall1-guard.log), [tall1/run.jsonl](tall1/run.jsonl)): ready → camera over → fingers-down approach (tip 0.085) → first descent look: offset 8–9 mm (= the arm's sag: the fixed position never changes) > min(12, 10 − 3) mm → **abort_descent → back up → re-centre → same again, 5+ times** (the abort resets the centring counter). Stopped (declined), folded home.
- **Fix:** the descent's re-align check ignores `kind == "fixed"` detections (no new information); any descent that aborts > 2 times now fails instead of looping. Engine tests 9 passed.
- **Next:** open 7 cm (`--open-margin 0.05`, 2.5 cm clearance: the neighbouring squares across the closing direction are empty), then verify from a second angle: home pose (oblique), YOLO-E, piece on the target square. Naman: "if you can't be confident from one side, get another angle".

### 16:56–17:00 tall2: two grasps, the piece slipped out of the pinch; place ran empty; second-angle check caught it
- Same with `--open-margin 0.05` (7 cm): no loop. Descent to tip 0.017 base (table ~5 cm), close to 0.11: **first close stalled at 0.287 (32 mm wide)**, i.e. on the piece's round head, not its neck; lift → verify "miss" (the fixed finder always says "still on the table", so this verify can't work) → retry → same → "grasp missed (and the retry missed too)" ([tall2/run.jsonl](tall2/run.jsonl)).
- **Mistake (Claude):** the frame after the retry ([tall2-after.jpg](tall2-after.jpg)) looked like the piece held between the fingers; it was standing on the board below them. [seq.py](seq.py) then carried an empty gripper to the target square, opened and went home.
- **Second-angle check** from home with YOLO-E ([verify1/look.jpg](verify1/look.jpg), [verify1.txt](verify1.txt)): the pawn and the tall piece are both still on their squares (one merged box at table (−0.057, −0.104), 4.5 cm), nothing toppled; target square empty. The check works; the move didn't.
- **Insight:** pinching "from the top" on a round-headed piece needs the real neck height: 5 cm on a ~7 cm guess caught the head's widest part (3.2 cm), which slips out when lifted. Also the verify step needs a real re-detection (oblique) for fixed targets.
- Motors off (j4 51 °C).
