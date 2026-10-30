# Visual choreography: "ripple: a wave that travels through the arm from the base to the fingertip (base turns, then shoulder, elbow, wrist, roll, gripper each pass it on with a delay), then travels back from the fingertip to the base, twice, the second time bigger. Lots of movement: every one of the six joints AND the gripper should be doing something through most of the move, overlapping (staggered starts, follow-through), not one joint at a time. Use big, readable amplitudes from the audience; vary the tempo with a build and at least one held accent. 15-25 seconds."

- model `claude-fable-5-1`, 4 call(s), 388.61 s total, stopped: satisfied
- camera: no (override): --camera never
- result: VALID ripple

| pass | valid | readability | silhouette | energy_contrast | timing | safety_margin | satisfied | latency s |
|---|---|---|---|---|---|---|---|---|
| 0 | yes | 4 | 4 | 3 | 4 | 4 | False | 228.15 |
| 1 | yes | 3 | 3 | 3 | 4 | 5 | False | 74.15 |
| 2 | yes | 4 | 4 | 4 | 4 | 5 | False | 44.81 |
| 3 | yes | 4 | 4 | 4 | 4 | 5 | True | 24.29 |

## Pass 0 (drafting)

- Draft: two ripples (out from base to fingertip, back from fingertip to base), first small and quick (1.0 s steps, ~0.25 rad per joint), second big and slow (1.3 s steps, ~0.45 rad per joint).
- Each joint hands the wave on one step after the previous one and moves over two consecutive segments so neighbours overlap; base/shoulder keep creeping slightly after arriving for follow-through.
- Gripper opens as the last link of each outbound wave and snaps shut to start the return; held accents at both fingertip peaks (0.35 s then 0.8 s) and a small base/head anticipation dip before the big wave.
- All cubic segments sized for <=0.78 rad/s^2 and <=0.26 rad/s; all poses >=0.2 rad inside soft limits; ends at ready at 24.75 s.

## Pass 1 (reviewing pass 1/3)

Critique: Valid and the structure reads as intended, but the first ripple is nearly invisible from the audience: six of the pass-1 segments move the hand only 1-5 cm, so the spectator sees a still arm with a faint twitch for the first nine seconds, then a modest second wave. The peak pose (K17) is distinct but not dramatic; the base turn and shoulder lean barely change the silhouette. The speed report shows the active caps are twice what I budgeted for (42 % velocity, 39 % acceleration used), so there is room to roughly double every amplitude at the same tempo. Timing structure (quick small wave, breath, slow big wave, held open-gripper peak) is good and tempo variety is acceptable; safety is excellent with the hand always above 40 cm.

- Re-budgeted every segment to the active 'dance' caps (0.6 rad/s, 2.0 rad/s^2), staying at or below ~77 % velocity and ~73 % acceleration.
- First ripple amplitudes raised from ~0.24 to ~0.42 rad per joint (j1 0.42, j2 +0.42, j3 +0.44, j4/j5/j6 0.40) with 0.95 s steps so each step moves the hand well over 5 cm.
- Second ripple amplitudes roughly doubled: j1 0.84, j2 1.12, j3 1.62, j4 0.78, j5 -0.80, j6 0.80, giving a clearly taller, turned-to-audience peak silhouette.
- Anticipation before the big wave made more visible: shoulder leans back to 0.42 and head dips -0.08 during the 0.3 s breath.
- Peak hold extended to 0.9 s; j4 gets a small follow-through overshoot into the peak; total duration 24.4 s.

## Pass 2 (reviewing pass 2/3)

Critique: Now it reads: the first wave visibly passes base→shoulder→elbow→wrist→hand with the gripper snapping open at the tip, and the second wave is unmistakably bigger, climbing to a tall vertical silhouette (hand at 66-71 cm) with the head turned to the audience and the gripper held open. Build, breath, peak and resolve are all clear, and the tempo shift from 0.95 s to 1.3 s steps is felt. Remaining weaknesses: the very first and last base-only segments move the hand only 2-5 cm on the audience's view (inherent to a base turn from ready, acceptable as the quiet bookends); at the big peak the arm points almost straight at the audience (j1 0.84 ≈ their bearing), so the hand's nod/twist is slightly foreshortened and the hold reads as a vertical bar more than a profile. Safety is excellent: 77 %/70 % of caps, hand never below 36 cm, no near-limit joints.

- Trimmed the big wave's base turn from 0.84 to 0.72 rad (0.35 rad steps) so the peak pose is seen more in profile and the wrist nod/twist foreshortens less.
- Return-pass base steps adjusted accordingly (0.72 → 0.70 → 0.64 → 0.34 → 0); everything else unchanged.

## Pass 3 (reviewing pass 3/3)

Critique: Valid and reads as a ripple: the spectator sees the body turn, then the shoulder lean, the elbow lift the hand 16 cm, the head nod and turn, and the gripper open at the tip; after a breath the same phrase comes back twice as big and slower, rising to a tall standing silhouette (hand at 66-71 cm) with the head turned to them and the gripper held open for almost a second, then folding back link by link to ready. Tempo varies (0.95 s vs 1.3 s steps, three holds), there is a small lean-back/head-dip anticipation before the big wave, and the second wave is clearly the peak. The only soft spots are the quiet base-only bookends (2-4 cm hand travel, inherent to turning the base from ready) and some residual foreshortening at the peak since the arm still points roughly toward the audience; neither undermines the intent. Caps at 77 %/70 %, hand never below 36 cm, every joint well inside its limits.

- Final: candidate unchanged (valid, every score >= 4).
