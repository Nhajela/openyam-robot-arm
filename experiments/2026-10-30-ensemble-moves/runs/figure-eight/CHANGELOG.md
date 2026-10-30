# Visual choreography: "figure-eight: the whole arm traces a huge sideways figure eight in the air in front of the audience while the wrist rolls continuously and the gripper snaps open/closed at each turn of the eight. Lots of movement: every one of the six joints AND the gripper should be doing something through most of the move, overlapping (staggered starts, follow-through), not one joint at a time. Use big, readable amplitudes from the audience; vary the tempo with a build and at least one held accent. 15-25 seconds."

- model `claude-fable-5-1`, 3 call(s), 279.34 s total, stopped: satisfied
- camera: no (override): --camera never
- result: VALID figure-eight

| pass | valid | readability | silhouette | energy_contrast | timing | safety_margin | satisfied | latency s |
|---|---|---|---|---|---|---|---|---|
| 0 | yes | 4 | 4 | 3 | 4 | 4 | False | 200.75 |
| 1 | yes | 4 | 4 | 3 | 3 | 5 | False | 46.4 |
| 2 | yes | 4 | 4 | 4 | 4 | 5 | True | 19.15 |

## Pass 0 (drafting)

- Draft: slow turn-away anticipation with grip snap open and a beat hold
- Lobe 1 (θ180→0) on flow at 2.3 s/segment: j1 sweeps -0.3→+0.9 while j2/j3/j4 arc up-down-mid on a lemniscate, j5 lags j1, j6 rolls -0.45→+0.75, grip alternates each turn
- Held accent at the audience-facing extreme (j1 0.9, head j5 -0.5) with a snap-closed gripper
- Lobe 2 faster (2.1 s/segment) as the build, j6 rolls back the other way, grip keeps alternating
- Soft 2.7 s cubic resolve to ready; all cubic/flow segments sized for ≤0.26 rad/s peak

## Pass 1 (reviewing pass 1/3)

Critique: From the audience the hand really does draw a tall figure eight (overview path), with 25–36 cm of travel per segment, and the gripper snaps at every turn are clear. Weak points: the build is invisible (2.3 s vs 2.1 s per segment reads as one tempo); the audience-facing hold (K7–K9) foreshortens into a stump pointing at the camera, so it relies entirely on the gripper snap to register; the two lobes are identical in size so there is little big/small contrast; speed headroom is large (45 % of cap) so the second lobe can afford to be noticeably quicker. Table clearance (25 cm tip) and limits are comfortable.

- Lobe 1 slower and bigger: 2.4 s per segment, j3/j4 swing ±0.38, j2 ∓0.22
- Lobe 2 clearly faster (1.8 s per segment) with the original ±0.34 swing, so the build is audible as a tempo change
- Hold at the audience-facing extreme kept; snap-closed segment 0.6 s then 0.5 s freeze
- Resolve lengthened to 3.0 s for a softer ending; total 24.5 s

## Pass 2 (reviewing pass 2/3)

Critique: The hand now draws an unmistakable tall figure eight on the audience's view (34–38 cm per segment in the slow first lobe, 25–33 cm in the quicker second), every joint is active through the phrase with the head (j5) lagging the body and the roll (j6) sweeping continuously, and the gripper snaps on every turn. The slow-big then fast-small lobes make the build audible (2.4 s vs 1.8 s per segment, tempo CV 0.48), the audience-facing hold with the snap-closed gripper is a clear accent even though that pose foreshortens, and the ending eases out over 3 s. Limits are well inside the comfortable range, 25 cm tip clearance at the lowest point, 52 % velocity / 33 % acceleration headroom. Nothing important left to change.

- No changes: candidate is valid and reads clearly; returned unchanged
