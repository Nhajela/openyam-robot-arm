# Visual choreography: "tornado: the base sweeps steadily one way while the wrist roll spins the other way, the head bobs and the shoulder/elbow pump in and out, building speed and size, then winding down and settling gracefully. Lots of movement: every one of the six joints AND the gripper should be doing something through most of the move, overlapping (staggered starts, follow-through), not one joint at a time. Use big, readable amplitudes from the audience; vary the tempo with a build and at least one held accent. 15-25 seconds."

- model `claude-fable-5-1`, 3 call(s), 147.89 s total, stopped: satisfied
- camera: no (override): --camera never
- result: VALID tornado

| pass | valid | readability | silhouette | energy_contrast | timing | safety_margin | satisfied | latency s |
|---|---|---|---|---|---|---|---|---|
| 0 | yes | 4 | 4 | 4 | 4 | 4 | False | 59.19 |
| 1 | yes | 4 | 3 | 4 | 4 | 5 | False | 55.73 |
| 2 | yes | 4 | 4 | 4 | 4 | 5 | True | 21.2 |

## Pass 0 (drafting)

- Draft: coil away from audience (j1 -, j6 + wind-up, head down) with a short hold
- Build: three flow segments sweeping j1 -0.3 to +1.0 while j6 unwinds the other way 0.5 to -1.0, shoulder/elbow pumping, head bobbing, j5 alternating, grip pulsing on each lap
- Peak: tallest pose facing the audience with head turned to them, held 0.8 s, then a grip snap shut accent and a second short hold
- Wind-down: two decelerating flow laps back toward centre with dampening bobs, then a 3 s soft cubic settle to ready
- All segment durations sized at >= 5.5 s per rad of the largest joint delta for cap headroom

## Pass 1 (reviewing pass 1/3)

Critique: The build reads well: a coil away, then three laps that grow from 16 to 34 to 45 cm of hand travel while the hand twists the other way, ending in a tall vertical pole facing the audience with the head turned to them, held, then a grip snap. Two weak spots: the anticipation (K0->K1) moves the hand only 4 cm, so the coil is invisible from the front; and the wind-down collapses — K10->K11 and K11->K12 each move the hand only 5 cm because the arm is nearly upright so the base sweep gives almost no lateral travel. The tornado stops abruptly after the peak instead of spiralling down. Safety is excellent (45 % velocity, 25 % accel, 8 cm above the floor at the lowest).

- Anticipation enlarged: base turns further away (j1 -0.4), forearm and head drop (j3 0.6, j4 -0.35), j6 winds up to 0.55; segment lengthened to 3.0 s
- First build lap re-sized to keep every joint delta <= 0.45 rad over 2.6 s
- Wind-down now spirals outward and down: lean forward (j2 0.9 -> 1.1) so the base sweep carries the hand on a wide, visible arc, with a damped elbow bounce and dampening head bob
- Added one more decaying lap (K12) before the 2.6 s soft settle; total 24.5 s
- Peak hold slightly longer after the grip snap (0.4 s) so the snap registers before the spiral-down starts

## Pass 2 (reviewing pass 2/3)

Critique: From the front this now reads as a spiral: a visible low coil away (14 cm), three laps that grow 21 → 29 → 45 cm as the base sweeps toward the audience while the gripper twists the other way and the shoulder/elbow pump, then an unmistakable photograph pose — a tall vertical pole with the head turned to the audience, held, then the mouth snaps shut. The spiral-down is now wide and low (45, 21, 29 cm) with a damped bob before a soft 6 cm settle to ready. All six joints and the gripper are active through most of the piece with staggered reversals; tempo varies (CV 0.49) with three holds. Lowest point is 8 cm above the safe height (tip 20 cm), peak velocity 46 % of cap — comfortable. The one nitpick is that the last bounce (K11->K12) is slightly larger than the lap before it, but it reads as a final flick before settling rather than a flaw.

- No changes: candidate is valid, every score >= 4, and the structure (coil, growing laps, held peak with grip snap, decaying spiral, soft settle) reads clearly from the audience
