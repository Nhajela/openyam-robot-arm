# Visual choreography: "a happy wave"

- model `claude-fable-5-1`, 2 call(s), 71.36 s total, stopped: passes exhausted
- camera: no (override): --camera never
- result: VALID happy-wave

| pass | valid | readability | silhouette | energy_contrast | timing | safety_margin | satisfied | latency s |
|---|---|---|---|---|---|---|---|---|
| 0 | yes (retimed) | 4 | 4 | 3 | 4 | 4 | False | 30.51 |
| 1 | yes | 3 | 3 | 3 | 4 | 3 | False | 31.52 |

## Pass 0 (drafting)

- Draft: small dip anticipation, big rise and turn toward the audience with head turning to them and grip snapping open, hold, three varied wave beats on j5/j6/j1 with grip accents, hold, slow resolve to ready.

## Pass 1 (reviewing pass 1/1)

Critique: The rise to a tall pose facing the audience reads well (52 cm of travel, hold, open grip), but the actual wave is invisible: each wave segment moves the hand only 6 cm because the arm is stacked vertically over the base and j5/j6 only twiddle the fingers. The ending is a nice long resolve. Retimer pushed velocity to 100 % of cap, so no headroom.

- Peak pose now reaches outward (j2 0.8, j3 1.3) so the hand has a long lever from the base
- Wave is carried by big j1 base-yaw swings 0.5<->1.0 rad (left-right across the audience's view), with j5/j6/j4 as follow-through accents
- Third swing slightly bigger with a head lift; grip half-closes on the middle beat
- Segment durations lengthened to keep ~10 % velocity headroom instead of relying on retime
- Return to ready lengthened to 5.6 s for a soft ending
