# Choreography guidelines for the OpenYAM arm

Used verbatim in the system prompt of `armctl.visual_author` (the visual, multi-pass choreography
author). Edit here; keep it short and concrete. Humans: the same rules make a good hand-authored playable.

## Who is watching, and from where

- The audience stands at the **table's front edge, on the arm's LEFT side (+y)**, about 1–1.5 m away,
  eyes 0.4–0.9 m above the table top (seated to standing). The arm faces down the table (+x). This is
  per-site configuration (`armctl/audience.json`); the render sheets lead with that view.
- Judge every move as a spectator who glances at it for two seconds: **is the intent or emotion obvious
  from the audience's side, at a distance, without the title?** If a move only reads from above or from
  down the table, it does not read.
- Motion straight toward or away from the audience is foreshortened and reads weakly. Motion across
  their view (left-right on the sheet), up-down, and changes of silhouette read strongly.

## What each joint "means" on this arm (from ready [0, 0.5, 0.8, 0, 0, 0])

- **j1 base yaw = the whole body turning.** + turns toward the audience (+y), − toward the glass wall.
  Use it for "looking around", "turning to someone", sweeping gestures. Big and readable.
- **j2 shoulder = leaning.** + leans forward and down (bowing, reaching, sneaking); − leans back and up
  (surprise, pride, recoil).
- **j3 elbow = posture height.** + lifts the forearm and hand up and back (standing tall, cheering);
  − lowers it forward (drooping, sadness, reaching low).
- **j4 wrist pitch = the head nodding.** + looks up, − looks down. The fingertips are the "face"; where
  they point is where the robot is looking.
- **j5 wrist yaw = the head turning.** − turns the head toward the audience, + away. Pair with j1 for
  "look at you": body turns a little, head turns more (and arrives later).
- **j6 wrist roll = head tilt / hand twist.** Moves the fingers, not the tip: it reads only as a small
  accent (a quizzical tilt, a flourish). **It does not carry a wave on its own.**
- **Gripper = mouth / hands.** Open–close snaps are the clearest accent this arm has: talking, laughing,
  clapping, surprise (snap open and hold), sadness (closed). Use grip changes on the beats.

## Principles (animation basics, adapted to a slow arm)

1. **Silhouette first.** Every keyframe should be a pose you could recognise as a still from the audience.
   Consecutive key poses must differ clearly: the hand should travel **at least ~8–10 cm on the audience's
   view** between key poses (the metrics flag < 3 cm as "barely visible"). Use the comfortable range:
   j1 −0.4 to +1.0 (the site limits are asymmetric; + faces the audience), j2 0.3–1.3, j3 0.5–1.9, j4 ±0.9, j5 ±0.8, j6 ±1.0 (always inside the soft limits).
2. **Clear beginning, peak and ending.** Start from ready, build to one unmistakable peak pose (the
   moment someone would photograph), then resolve and return to ready. Hold the peak (a `hold` keyframe,
   0.4–1.0 s) so it registers.
3. **Anticipation.** Before a big move, a small move the other way (dip before rising, lean back before
   lunging, turn away before turning to face). 0.1–0.3 rad is enough.
4. **Follow-through and overlap.** Joints should not all start and stop together. Let the body lead and
   the head (j4/j5) and hand (grip) arrive a beat later, or overshoot slightly and settle.
5. **Contrast.** Mix big and small, fast and slow, motion and stillness. Avoid constant tempo
   (all segments the same length) and symmetric back-and-forth repeated identically; vary the second
   and third repetition (bigger, smaller, a pause, a different accent).
6. **Accents on beats, holds for emphasis.** A short hold after a sharp move makes the move read as
   intentional. A grip snap on the accent makes it read as a "word".
7. **Arcs, not jerks.** Combine j2/j3/j4 so the hand travels on a curve. Prefer `cubic` (smooth stop) or
   `flow` (smooth through the keyframe, stops only where the motion reverses). Never `linear` from rest.
8. **Mood through posture and tempo.** Happy: tall (j3 up), open grip, bouncy faster segments.
   Sad: low (j2 forward, j3 down, j4 down), closed grip, slow, long holds. Surprised: quick recoil back
   and up (j2 −, j4 +), grip snaps open, then freeze. Curious: lean forward slowly, head tilt (j6), small
   pecks. Proud: lean back, head up. Shy: turn away (j1 −) with head down, peek back.
9. **Wrist flips and limits.** Don't spin j6 or j5 through large angles between adjacent keyframes
   just to change a small thing; it looks like a glitch. Stay ≥ 0.2 rad inside every soft limit;
   near-limit poses look strained and leave no margin.
10. **The table.** Keep the hand well above the table (the tip floor is 8 cm; aim for ≥ 15 cm except on
    purpose). Leaning forward (j2 +) with the forearm down (j3 −) drops the hand fast.

## The speed caps are the main constraint

- Default caps are slow (about 0.3 rad/s and 1 rad/s² per joint): a 0.5 rad cubic move needs ≥ 2.5 s.
  So you get only a few big moves per 15 s. **Spend them on the gesture that carries the meaning**, and
  make the rest small accents (grip, j6, j4) that fit in short segments.
- Big amplitude + long time reads better than small amplitude + short time. Don't shrink a move until
  it disappears just to keep a fast tempo.
- Sequence rather than stack: moving many joints at once is not faster (each joint has its own cap), but
  staggering them creates overlap and life.

## The library today (what to beat)

Measured with `experiments/2026-10-29-visual-choreo/library_stats.py` (29 playables):

- **Small amplitudes.** Median range used per joint is j1 0.43, j2 0.30, j3 0.40, j4 0.35, j5 0.00,
  j6 0.24 rad. `wave-hello`/`wave` move j1 by ±0.08 and j6 by ±0.12: from the audience the hand moves
  1–2 cm per wave segment. It does not read as a wave.
- **Constant tempo.** `wave` (tempo variety 0.18), `nod` (0.18), `happy-dance` (0.14),
  `happy-little-dance` (0.00), `gripper-clap` (0.14), `beat-dance-auto-hype` (0.01): metronomic.
- **Symmetric repeats** with no variation (wave, nod, gripper-clap: 3–4 unique poses repeated).
- **No anticipation, few accents.** 13 of 29 have no `hold` at all; 16 of 29 never change the
  gripper.
- **j5 almost unused** (median 0), so "looking at someone" never happens.
- Better examples to learn from: `curious-sniff` (big lean, varied tempo, holds), `sleepy-stretch`
  (large j3/j4 range, holds), `disco-point` (big j1 + j3 silhouette change), `peekaboo` (holds as beats).

## A good structure for a 8–15 s gesture

1. 0–1.5 s: anticipation away from the goal (small).
2. 1.5–5 s: the main move, big, on an arc; head and grip arrive slightly late.
3. Peak: hold 0.5–1 s; one accent (grip snap / head tilt).
4. A varied repeat or a reaction (smaller or bigger, different timing).
5. Resolve to ready with a slower, softer ending.
